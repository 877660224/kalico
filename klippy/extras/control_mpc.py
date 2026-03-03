import logging
import math
import threading
import time

import numpy as np

AMBIENT_TEMP = 25.0
PIN_MIN_TIME = 0.100

FILAMENT_TEMP_SRC_AMBIENT = "ambient"
FILAMENT_TEMP_SRC_FIXED = "fixed"
FILAMENT_TEMP_SRC_SENSOR = "sensor"


class ControlMPC:
    def __init__(self, profile, heater, load_clean=False, register=True):
        self.profile = profile
        self._load_profile()
        self.heater = heater
        self.heater_max_power = heater.get_max_power() * self.const_heater_power

        self.want_ambient_refresh = self.ambient_sensor is not None
        self.state_block_temp = (
            AMBIENT_TEMP if load_clean else self._heater_temp()
        )
        self.state_sensor_temp = self.state_block_temp
        self.state_ambient_temp = AMBIENT_TEMP

        self.last_power = 0.0
        self.last_loss_ambient = 0.0
        self.last_loss_filament = 0.0
        self.last_time = 0.0
        self.last_temp_time = 0.0

        self.printer = heater.printer
        self.toolhead = None

        if not register:
            return

        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "MPC_CALIBRATE",
            "HEATER",
            heater.get_name(),
            self.cmd_MPC_CALIBRATE,
            desc=self.cmd_MPC_CALIBRATE_help,
        )
        gcode.register_mux_command(
            "MPC_SET",
            "HEATER",
            heater.get_name(),
            self.cmd_MPC_SET,
            desc=self.cmd_MPC_SET_help,
        )

    cmd_MPC_SET_help = "Set MPC parameter"

    def cmd_MPC_SET(self, gcmd):
        self.const_filament_diameter = gcmd.get_float(
            "FILAMENT_DIAMETER", self.const_filament_diameter
        )
        self.const_filament_density = gcmd.get_float(
            "FILAMENT_DENSITY", self.const_filament_density
        )
        self.const_filament_heat_capacity = gcmd.get_float(
            "FILAMENT_HEAT_CAPACITY", self.const_filament_heat_capacity
        )

        self.const_block_heat_capacity = gcmd.get_float(
            "BLOCK_HEAT_CAPACITY", self.const_block_heat_capacity
        )
        self.const_sensor_responsiveness = gcmd.get_float(
            "SENSOR_RESPONSIVENESS", self.const_sensor_responsiveness
        )
        self.const_ambient_transfer = gcmd.get_float(
            "AMBIENT_TRANSFER", self.const_ambient_transfer
        )

        if gcmd.get("FAN_AMBIENT_TRANSFER", None):
            try:
                self.const_fan_ambient_transfer = [
                    float(v)
                    for v in gcmd.get("FAN_AMBIENT_TRANSFER").split(",")
                ]
            except ValueError:
                raise gcmd.error(
                    f"Error on '{gcmd._commandline}': unable to parse FAN_AMBIENT_TRANSFER\n"
                    "Must be a comma-separated list of values ('0.05,0.07,0.08')"
                )

        temp = gcmd.get("FILAMENT_TEMP", None)
        if temp is not None:
            temp = temp.lower().strip()
            if temp == "sensor":
                self.filament_temp_src = (FILAMENT_TEMP_SRC_SENSOR,)
            elif temp == "ambient":
                self.filament_temp_src = (FILAMENT_TEMP_SRC_AMBIENT,)
            else:
                try:
                    value = float(temp)
                except ValueError:
                    raise gcmd.error(
                        f"Error on '{gcmd._commandline}': unable to parse FILAMENT_TEMP\n"
                        "Valid options are 'sensor', 'ambient', or number."
                    )
                self.filament_temp_src = (FILAMENT_TEMP_SRC_FIXED, value)

        self._update_filament_const()

    cmd_MPC_CALIBRATE_help = "Run MPC calibration"

    def cmd_MPC_CALIBRATE(self, gcmd):
        cal = MpcCalibrate(self.printer, self.heater, self)
        cal.run(gcmd)

    # Helpers

    def _heater_temp(self):
        return self.heater.get_temp(self.heater.reactor.monotonic())[0]

    def _load_profile(self):
        self.const_block_heat_capacity = self.profile["block_heat_capacity"]
        self.const_ambient_transfer = self.profile["ambient_transfer"]
        self.const_target_reach_time = self.profile["target_reach_time"]
        self.const_heater_power = self.profile["heater_power"]
        self.const_smoothing = self.profile["smoothing"]
        self.const_sensor_responsiveness = self.profile["sensor_responsiveness"]
        self.const_min_ambient_change = self.profile["min_ambient_change"]
        self.const_steady_state_rate = self.profile["steady_state_rate"]
        self.const_filament_diameter = self.profile["filament_diameter"]
        self.const_filament_density = self.profile["filament_density"]
        self.const_filament_heat_capacity = self.profile[
            "filament_heat_capacity"
        ]
        self.const_maximum_retract = self.profile["maximum_retract"]
        self.filament_temp_src = self.profile["filament_temp_src"]
        self._update_filament_const()
        self.ambient_sensor = self.profile["ambient_temp_sensor"]
        self.cooling_fan = self.profile["cooling_fan"]
        self.const_fan_ambient_transfer = self.profile["fan_ambient_transfer"]

    def is_valid(self):
        return (
            self.const_block_heat_capacity is not None
            and self.const_ambient_transfer is not None
            and self.const_sensor_responsiveness is not None
        )

    def check_valid(self):
        if self.is_valid():
            return
        name = self.heater.get_name()
        raise self.printer.command_error(
            f"Cannot activate '{name}' as MPC control is not fully configured.\n\n"
            f"Run 'MPC_CALIBRATE' or ensure 'block_heat_capacity', 'sensor_responsiveness', and "
            f"'ambient_transfer' settings are defined for '{name}'."
        )

    def _update_filament_const(self):
        radius = self.const_filament_diameter / 2.0
        self.const_filament_cross_section_heat_capacity = (
            (radius * radius)  # mm^2
            * math.pi  # 1
            / 1000.0  # mm^3 => cm^3
            * self.const_filament_density  # g/cm^3
            * self.const_filament_heat_capacity  # J/g/K
        )

    # Control interface

    def temperature_update(self, read_time, temp, target_temp):
        if not self.is_valid():
            self.heater.set_pwm(read_time, 0.0)
            return

        dt = read_time - self.last_temp_time
        if self.last_temp_time == 0.0 or dt < 0.0 or dt > 1.0:
            dt = 0.1

        # Extruder position
        extrude_speed_prev = 0.0
        extrude_speed_next = 0.0
        if target_temp != 0.0:
            if self.toolhead is None:
                self.toolhead = self.printer.lookup_object("toolhead")
            if self.toolhead is not None:
                extruder = self.toolhead.get_extruder()
                if (
                    hasattr(extruder, "find_past_position")
                    and extruder.get_heater() == self.heater
                ):
                    pos = extruder.find_past_position(read_time)

                    pos_prev = extruder.find_past_position(read_time - dt)
                    pos_moved = max(-self.const_maximum_retract, pos - pos_prev)
                    extrude_speed_prev = pos_moved / dt

                    pos_next = extruder.find_past_position(read_time + dt)
                    pos_move = max(-self.const_maximum_retract, pos_next - pos)
                    extrude_speed_next = pos_move / dt

        # Modulate ambient transfer coefficient with fan speed
        ambient_transfer = self.const_ambient_transfer
        if self.cooling_fan and len(self.const_fan_ambient_transfer) > 1:
            fan_speed = max(
                0.0, min(1.0, self.cooling_fan.get_status(read_time)["speed"])
            )
            fan_break = fan_speed * (len(self.const_fan_ambient_transfer) - 1)
            below = self.const_fan_ambient_transfer[math.floor(fan_break)]
            above = self.const_fan_ambient_transfer[math.ceil(fan_break)]
            if below != above:
                frac = fan_break % 1.0
                ambient_transfer = below * (1 - frac) + frac * above
            else:
                ambient_transfer = below

        # Simulate

        # Expected power by heating at last power setting
        expected_heating = self.last_power
        # Expected power from block to ambient
        block_ambient_delta = self.state_block_temp - self.state_ambient_temp
        expected_ambient_transfer = block_ambient_delta * ambient_transfer
        expected_filament_transfer = (
            block_ambient_delta
            * extrude_speed_prev
            * self.const_filament_cross_section_heat_capacity
        )

        # Expected block dT since last period
        expected_block_dT = (
            (
                expected_heating
                - expected_ambient_transfer
                - expected_filament_transfer
            )
            * dt
            / self.const_block_heat_capacity
        )
        self.state_block_temp += expected_block_dT

        # Expected sensor dT since last period
        expected_sensor_dT = (
            (self.state_block_temp - self.state_sensor_temp)
            * self.const_sensor_responsiveness
            * dt
        )
        self.state_sensor_temp += expected_sensor_dT

        # Correct

        smoothing = 1 - (1 - self.const_smoothing) ** dt
        adjustment_dT = (temp - self.state_sensor_temp) * smoothing
        self.state_block_temp += adjustment_dT
        self.state_sensor_temp += adjustment_dT

        if self.want_ambient_refresh:
            temp = self.ambient_sensor.get_temp(read_time)[0]
            if temp != 0.0:
                self.state_ambient_temp = temp
                self.want_ambient_refresh = False
        if (self.last_power > 0 and self.last_power < 1.0) or abs(
            expected_block_dT + adjustment_dT
        ) < self.const_steady_state_rate * dt:
            if adjustment_dT > 0.0:
                ambient_delta = max(
                    adjustment_dT, self.const_min_ambient_change * dt
                )
            else:
                ambient_delta = min(
                    adjustment_dT, -self.const_min_ambient_change * dt
                )
            self.state_ambient_temp += ambient_delta

        # Output

        # Amount of power needed to reach the target temperature in the desired time

        heating_power = (
            (target_temp - self.state_block_temp)
            * self.const_block_heat_capacity
            / self.const_target_reach_time
        )
        # Losses (+ = lost from block, - = gained to block)
        block_ambient_delta = self.state_block_temp - self.state_ambient_temp
        loss_ambient = block_ambient_delta * ambient_transfer
        block_filament_delta = self.state_block_temp - self.filament_temp(
            read_time, self.state_ambient_temp
        )
        loss_filament = (
            block_filament_delta
            * extrude_speed_next
            * self.const_filament_cross_section_heat_capacity
        )

        if target_temp != 0.0:
            # The required power is the desired heating power + compensation for all the losses
            power = max(
                0.0,
                min(
                    self.heater_max_power,
                    heating_power + loss_ambient + loss_filament,
                ),
            )
        else:
            power = 0

        duty = power / self.const_heater_power

        # logging.info(
        #     "mpc: [%.3f/%.3f] %.2f => %.2f / %.2f / %.2f = %.2f[%.2f+%.2f+%.2f] / %.2f, dT %.2f, E %.2f=>%.2f",
        #     dt,
        #     smoothing,
        #     temp,
        #     self.state_block_temp,
        #     self.state_sensor_temp,
        #     self.state_ambient_temp,
        #     power,
        #     heating_power,
        #     loss_ambient,
        #     loss_filament,
        #     duty,
        #     adjustment_dT,
        #     extrude_speed_prev,
        #     extrude_speed_next,
        # )

        self.last_power = power
        self.last_loss_ambient = loss_ambient
        self.last_loss_filament = loss_filament
        self.last_temp_time = read_time
        self.heater.set_pwm(read_time, duty)

    def filament_temp(self, read_time, ambient_temp):
        src = self.filament_temp_src
        if src[0] == FILAMENT_TEMP_SRC_FIXED:
            return src[1]
        elif (
            src[0] == FILAMENT_TEMP_SRC_SENSOR
            and self.ambient_sensor is not None
        ):
            return self.ambient_sensor.get_temp(read_time)[0]
        else:
            return ambient_temp

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return abs(target_temp - smoothed_temp) > 1.0

    def update_smooth_time(self):
        pass

    def get_profile(self):
        return self.profile

    def get_type(self):
        return "mpc"

    def get_status(self, eventtime):
        return {
            "temp_block": self.state_block_temp,
            "temp_sensor": self.state_sensor_temp,
            "temp_ambient": self.state_ambient_temp,
            "power": self.last_power,
            "loss_ambient": self.last_loss_ambient,
            "loss_filament": self.last_loss_filament,
            "filament_temp": self.filament_temp_src,
            "filament_heat_capacity": self.const_filament_heat_capacity,
            "filament_density": self.const_filament_density,
        }


class MpcCalibrate:
    def __init__(self, printer, heater, orig_control):
        self.printer = printer
        self.heater = heater
        self.orig_control = orig_control

    def save_heatup_data(self, gcmd, samples):
        """
        Save heatup test data to CSV file
        """
        import os
        import time
        
        # Get home directory and create data path
        home_dir = os.path.expanduser("~")
        data_dir = os.path.join(home_dir, "printer_data", "data")
        
        # Create directory if it doesn't exist (including parent directories)
        os.makedirs(data_dir, exist_ok=True)
        
        # Generate filename with timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"nenocontrol_{self.heater.get_name()}_{timestamp}.csv"
        file_path = os.path.join(data_dir, filename)
        
        try:
            # Write data to CSV
            with open(file_path, 'w') as f:
                # Write header
                f.write("time,temperature\n")
                
                # Write data - convert to relative time if we have samples
                if samples:
                    start_time = samples[0][0]
                    for t, temp in samples:
                        rel_time = t - start_time
                        f.write(f"{rel_time:.3f},{temp:.2f}\n")
                else:
                    logging.warning("No samples to save in save_heatup_data")
            
            # Log and respond to user
            logging.info(f"Heatup data saved to: {file_path}")
            gcmd.respond_info(f"Heatup test data saved to {file_path}")
        except Exception as e:
            logging.error(f"Failed to save heatup data: {e}")
            gcmd.respond_info(f"Warning: Failed to save heatup data: {e}")
        
    def run(self, gcmd):
        use_analytic = gcmd.get("USE_DELTA", None) is not None
        ambient_max_measure_time = gcmd.get_float(
            "AMBIENT_MAX_MEASURE_TIME", 20.0, above=0.0
        )
        ambient_measure_sample_time = gcmd.get_float(
            "AMBIENT_MEASURE_SAMPLE_TIME", 5.0, below=ambient_max_measure_time
        )
        fan_breakpoints = gcmd.get_int("FAN_BREAKPOINTS", 3, minval=2)
        default_target_temp = (
            90.0 if self.heater.get_name() == "heater_bed" else 200.0
        )
        target_temp = gcmd.get_float("TARGET", default_target_temp, minval=60.0)
        threshold_temp = gcmd.get_float(
            "THRESHOLD", max(50.0, min(100, target_temp - 100.0))
        )

        control = TuningControl(self.heater)
        old_control = self.heater.set_control(control)
        try:
            ambient_temp = self.await_ambient(gcmd, control, threshold_temp)
            samples = self.heatup_test(gcmd, target_temp, control)
            self.save_heatup_data(gcmd, samples)
            first_res = self.process_first_pass(
                samples,
                self.orig_control.heater_max_power,
                ambient_temp,
                threshold_temp,
                use_analytic,
            )
            logging.info("First pass: %s", first_res)

            profile = dict(self.orig_control.profile)
            for key in [
                "block_heat_capacity",
                "ambient_transfer",
                "sensor_responsiveness",
            ]:
                profile[key] = first_res[key]
            new_control = ControlMPC(profile, self.heater, False, False)
            new_control.state_block_temp = first_res["post_block_temp"]
            new_control.state_sensor_temp = first_res["post_sensor_temp"]
            new_control.state_ambient_temp = ambient_temp
            self.heater.set_control(new_control)

            transfer_res = self.transfer_test(
                gcmd,
                ambient_max_measure_time,
                ambient_measure_sample_time,
                fan_breakpoints,
                first_res,
            )
            second_res = self.process_second_pass(
                first_res,
                transfer_res,
                ambient_temp,
                self.orig_control.heater_max_power,
            )
            logging.info("Second pass: %s", second_res)

            block_heat_capacity = (
                second_res["block_heat_capacity"]
                if use_analytic
                else first_res["block_heat_capacity"]
            )
            sensor_responsiveness = (
                second_res["sensor_responsiveness"]
                if use_analytic
                else first_res["sensor_responsiveness"]
            )
            ambient_transfer = second_res["ambient_transfer"]
            fan_ambient_transfer = ", ".join(
                [f"{p:.6g}" for p in second_res["fan_ambient_transfer"]]
            )

            cfgname = self.heater.get_name()
            gcmd.respond_info(
                f"Finished MPC calibration of heater '{cfgname}'\n"
                "Measured:\n "
                f"  block_heat_capacity={block_heat_capacity:#.6g} [J/K]\n"
                f"  sensor_responsiveness={sensor_responsiveness:#.6g} [K/s/K]\n"
                f"  ambient_transfer={ambient_transfer:#.6g} [W/K]\n"
                f"  fan_ambient_transfer={fan_ambient_transfer} [W/K]\n"
            )

            configfile = self.heater.printer.lookup_object("configfile")
            configfile.set(cfgname, "control", "mpc")
            configfile.set(
                cfgname, "block_heat_capacity", f"{block_heat_capacity:#.6g}"
            )
            configfile.set(
                cfgname,
                "sensor_responsiveness",
                f"{sensor_responsiveness:#.6g}",
            )
            configfile.set(
                cfgname, "ambient_transfer", f"{ambient_transfer:#.6g}"
            )
            configfile.set(
                cfgname,
                "fan_ambient_transfer",
                fan_ambient_transfer,
            )

        except self.printer.command_error as e:
            raise gcmd.error("%s failed: %s" % (gcmd.get_command(), e))
        finally:
            self.heater.set_control(old_control)
            self.heater.alter_target(0.0)

    def wait_stable(self, cycles=5):
        """
        We wait for the extruder to cycle x amount of times above and below the target
        doing this should ensure the temperature is stable enough to give a good result
        as a fallback if it stays within 0.1 degree for ~30 seconds it is also accepted
        """

        below_target = True
        above_target = 0
        on_target = 0
        starttime = self.printer.reactor.monotonic()

        def process(eventtime):
            nonlocal below_target, above_target, on_target
            temp, target = self.heater.get_temp(eventtime)
            if below_target and temp > target + 0.015:
                above_target += 1
                below_target = False
            elif not below_target and temp < target - 0.015:
                below_target = True
            if (
                above_target >= cycles
                and (self.printer.reactor.monotonic() - starttime) > 30.0
            ):
                return False
            if above_target > 0 and abs(target - temp) < 0.1:
                on_target += 1
            else:
                on_target = 0
            if on_target >= 150:  # in case the heating is super consistent
                return False
            return True

        self.printer.wait_while(process, True, 0.2)

    def wait_settle(self, max_rate):
        last_temp = None
        next_check = None
        samples = []

        def process(eventtime):
            temp, _ = self.heater.get_temp(eventtime)
            samples.append((eventtime, temp))
            while samples[0][0] < eventtime - 10.0:
                samples.pop(0)
            dT = samples[-1][1] - samples[0][1]
            dt = samples[-1][0] - samples[0][0]
            if dt < 8.0:
                return True
            rate = abs(dT / dt)
            return not rate < max_rate

        self.printer.wait_while(process)
        return samples[-1][1]

    def await_ambient(self, gcmd, control, minimum_temp):
        self.heater.alter_target(1.0)  # Turn on fan to increase settling speed
        if self.orig_control.ambient_sensor is not None:
            # If we have an ambient sensor we won't waste time waiting for ambient.
            # We do however need to wait for sub minimum_temp(we pick -5 C relative).
            reported = [False]
            target = minimum_temp - 5

            def process(eventtime):
                temp, _ = self.heater.get_temp(eventtime)
                ret = temp > target
                if ret and not reported[0]:
                    gcmd.respond_info(
                        f"Waiting for heater to drop below {target} degrees Celsius"
                    )
                    reported[0] = True
                return ret

            self.printer.wait_while(process)
            self.heater.alter_target(0.0)
            return self.orig_control.ambient_sensor.get_temp(
                self.heater.reactor.monotonic()
            )[0]

        gcmd.respond_info("Waiting for heater to settle at ambient temperature")
        ambient_temp = self.wait_settle(0.01)
        self.heater.alter_target(0.0)
        return ambient_temp

    def heatup_test(self, gcmd, target_temp, control):
        gcmd.respond_info(
            "Performing heatup test, target is %.1f degrees" % (target_temp,)
        )
        control.set_output(self.heater.get_max_power(), target_temp)

        control.logging = True

        def process(eventtime):
            temp, _ = self.heater.get_temp(eventtime)
            return temp < target_temp

        self.printer.wait_while(process)
        control.logging = False
        self.heater.alter_target(0.0)

        log = control.log
        control.log = []
        return log

    def transfer_test(
        self,
        gcmd,
        ambient_max_measure_time,
        ambient_measure_sample_time,
        fan_breakpoints,
        first_pass_results,
    ):
        target_temp = round(first_pass_results["post_block_temp"])
        self.heater.set_temp(target_temp)
        gcmd.respond_info(
            "Performing ambient transfer tests, target is %.1f degrees"
            % (target_temp,)
        )

        self.wait_stable(5)

        fan = self.orig_control.cooling_fan

        fan_powers = []
        if fan is None:
            power_base = self.measure_power(
                ambient_max_measure_time, ambient_measure_sample_time
            )
            gcmd.respond_info(f"Average stable power: {power_base} W")
        else:
            for idx in range(0, fan_breakpoints):
                speed = idx / (fan_breakpoints - 1)
                curtime = self.heater.reactor.monotonic()
                print_time = fan.get_mcu().estimated_print_time(curtime)
                fan.set_speed(print_time + PIN_MIN_TIME, speed)
                gcmd.respond_info("Waiting for temperature to stabilize")
                self.wait_stable(3)
                gcmd.respond_info(
                    f"Temperature stable, measuring power usage with {speed * 100.0:.0f}% fan speed"
                )
                power = self.measure_power(
                    ambient_max_measure_time, ambient_measure_sample_time
                )
                gcmd.respond_info(
                    f"{speed * 100.0:.0f}% fan average power: {power:.2f} W"
                )
                fan_powers.append((speed, power))
            curtime = self.heater.reactor.monotonic()
            print_time = fan.get_mcu().estimated_print_time(curtime)
            fan.set_speed(print_time + PIN_MIN_TIME, 0.0)
            power_base = fan_powers[0][1]

        return {
            "target_temp": target_temp,
            "base_power": power_base,
            "fan_powers": fan_powers,
        }

    def measure_power(self, max_time, sample_time):
        samples = []
        time = [0]
        last_time = [None]

        def process(eventtime):
            dt = eventtime - (
                last_time[0] if last_time[0] is not None else eventtime
            )
            last_time[0] = eventtime
            status = self.heater.get_status(eventtime)
            samples.append((dt, status["control_stats"]["power"] * dt))
            time[0] += dt
            return time[0] < max_time

        self.printer.wait_while(process)

        total_energy = 0
        total_time = 0
        for dt, energy in reversed(samples):
            total_energy += energy
            total_time += dt
            if total_time > sample_time:
                break

        return total_energy / total_time

    def fastest_rate(self, samples):
        best = [-1, 0, 0]
        base_t = samples[0][0]
        for idx in range(2, len(samples)):
            dT = samples[idx][1] - samples[idx - 2][1]
            dt = samples[idx][0] - samples[idx - 2][0]
            rate = dT / dt
            if rate > best[0]:
                sample = samples[idx - 1]
                best = [sample[0] - base_t, sample[1], rate]
        return best

    def process_first_pass(
        self,
        all_samples,
        heater_power,
        ambient_temp,
        threshold_temp,
        use_analytic,
    ):
        # Find a continous segment of samples that all lie in the threshold.. range
        best_lower = None
        for idx in range(0, len(all_samples)):
            if all_samples[idx][1] > threshold_temp and best_lower is None:
                best_lower = idx
            elif all_samples[idx][1] < threshold_temp:
                best_lower = None

        t1_time = all_samples[best_lower][0] - all_samples[0][0]

        samples = all_samples[best_lower:]
        pitch = math.floor((len(samples) - 1) / 2)
        # We pick samples 0, pitch, and 2pitch, ensuring matching time spacing
        dt = samples[pitch][0] - samples[0][0]
        t1 = samples[0][1]
        t2 = samples[pitch][1]
        t3 = samples[2 * pitch][1]

        asymp_T = (t2 * t2 - t1 * t3) / (2.0 * t2 - t1 - t3)
        block_responsiveness = -math.log((t2 - asymp_T) / (t1 - asymp_T)) / dt
        ambient_transfer = heater_power / (asymp_T - ambient_temp)

        block_heat_capacity = -1.0
        sensor_responsiveness = -1.0
        start_temp = all_samples[0][1]

        # Asymptotic method
        if use_analytic:
            block_heat_capacity = ambient_transfer / block_responsiveness
            sensor_responsiveness = block_responsiveness / (
                1.0
                - (start_temp - asymp_T)
                * math.exp(-block_responsiveness * t1_time)
                / (t1 - asymp_T)
            )

        # Differential method
        if (
            not use_analytic
            or block_heat_capacity < 0
            or sensor_responsiveness < 0
        ):
            fastest_rate = self.fastest_rate(samples)
            block_heat_capacity = heater_power / fastest_rate[2]
            sensor_responsiveness = fastest_rate[2] / (
                fastest_rate[2] * fastest_rate[0]
                + ambient_temp
                - fastest_rate[0]
            )

        heat_time = all_samples[-1][0] - all_samples[0][0]
        post_block_temp = asymp_T + (start_temp - asymp_T) * math.exp(
            -block_responsiveness * heat_time
        )
        post_sensor_temp = all_samples[-1][1]

        return {
            "post_block_temp": post_block_temp,
            "post_sensor_temp": post_sensor_temp,
            "block_responsiveness": block_responsiveness,
            "ambient_transfer": ambient_transfer,
            "block_heat_capacity": block_heat_capacity,
            "sensor_responsiveness": sensor_responsiveness,
            "asymp_temp": asymp_T,
            "t1": t1,
            "t1_time": t1_time,
            "t2": t2,
            "start_temp": start_temp,
            "dt": dt,
        }

    def process_second_pass(
        self, first_res, transfer_res, ambient_temp, heater_power
    ):
        target_ambient_temp = transfer_res["target_temp"] - ambient_temp
        ambient_transfer = transfer_res["base_power"] / target_ambient_temp
        asymp_T = ambient_temp + heater_power / ambient_transfer
        block_responsiveness = (
            -math.log((first_res["t2"] - asymp_T) / (first_res["t1"] - asymp_T))
            / first_res["dt"]
        )
        block_heat_capacity = ambient_transfer / block_responsiveness
        sensor_responsiveness = block_responsiveness / (
            1.0
            - (first_res["start_temp"] - asymp_T)
            * math.exp(-block_responsiveness * first_res["t1_time"])
            / (first_res["t1"] - asymp_T)
        )

        fan_ambient_transfer = [
            power / target_ambient_temp
            for (_speed, power) in transfer_res["fan_powers"]
        ]

        return {
            "ambient_transfer": ambient_transfer,
            "block_responsiveness": block_responsiveness,
            "block_heat_capacity": block_heat_capacity,
            "sensor_responsiveness": sensor_responsiveness,
            "asymp_temp": asymp_T,
            "fan_ambient_transfer": fan_ambient_transfer,
        }


class TuningControl:
    def __init__(self, heater):
        self.value = 0.0
        self.target = None
        self.heater = heater
        self.log = []
        self.logging = False

    def temperature_update(self, read_time, temp, target_temp):
        if self.logging:
            self.log.append((read_time, temp))
        self.heater.set_pwm(read_time, self.value)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return self.value != 0.0 or self.target != 0

    def set_output(self, value, target):
        self.value = value
        self.target = target
        self.heater.set_temp(target)

    def get_profile(self):
        return {"name": "tuning"}

    def get_type(self):
        return "tuning"


class AdaptiveMPC:
    """
    自适应MPC控制器类
    
    该类实现数据驱动的自适应参数辨识与MPC控制的集成，是Klipper/Kalico
    温度控制系统的核心组件之一。通过持续采集温度数据并评估数据质量，
    周期性地优化热参数，实现更精确的温度控制。
    
    设计目的:
        传统MPC控制器使用固定参数，无法适应热系统特性的变化（如环境温度变化、
        加热器老化、散热条件改变等）。本类通过数据驱动的参数辨识方法，
        实现参数的自动优化和更新，提高控制精度和鲁棒性。
    
    核心特性:
        - 参数辨识与实时控制解耦：辨识过程在后台运行，不影响实时控制
        - 数据质量评估与分级存储：按动态特性将数据分为A/B/C/D四级
        - 周期性批量参数更新：支持定时触发和误差触发两种更新机制
        - 参数验证与版本管理：新参数需通过物理合理性验证，保留历史版本
        - 平滑过渡机制：参数更新采用指数平滑，避免突变导致控制震荡
        - 故障自动恢复：连续失败时自动回滚到上一有效版本
    
    使用场景:
        - 3D打印机加热头温度控制
        - 热床温度控制
        - 其他需要精确温度控制的加热系统
    
    继承关系:
        该类不继承自其他类，但内部组合了ControlMPC实例作为基础控制器。
    
    线程安全:
        数据缓冲区操作使用threading.Lock保护，确保多线程环境下的数据一致性。
    
    示例:
        >>> profile = {
        ...     "block_heat_capacity": 15.0,
        ...     "ambient_transfer": 0.08,
        ...     "heater_power": 40.0,
        ...     "adaptive_enabled": True,
        ... }
        >>> controller = AdaptiveMPC(profile, heater)
        >>> controller.temperature_update(read_time, temp, target_temp)
    
    属性:
        profile (dict): 控制器配置参数字典
        heater (Heater): 加热器对象实例
        printer (Printer): 打印机主对象
        reactor (Reactor): 事件反应器，用于定时任务调度
        gcode (GCode): G-code命令处理器
        base_mpc (ControlMPC): 基础MPC控制器实例
        data_buffer (dict): 数据缓存，按键值"A"/"B"/"C"分级存储
        data_buffer_lock (threading.Lock): 数据缓冲区线程锁
        param_history (list): 参数版本历史记录列表
        current_params (dict): 当前使用的热参数字典
        is_adapting (bool): 是否正在进行参数适应过程
        adaptation_enabled (bool): 自适应功能是否启用
        last_adaptation_time (float): 上次参数更新时间戳
        adaptation_failures (int): 连续参数更新失败次数
        temp_error_window (list): 温度预测误差滑动窗口
    
    异常:
        该类本身不主动抛出异常，但内部方法可能捕获并记录异常，
        不会影响控制器的正常运行。
    
    注意事项:
        - 参数辨识需要足够的动态数据，建议在温度变化过程中采集
        - 避免在稳态温度下长时间运行，此时数据质量较低
        - 参数更新间隔不宜过短，避免频繁计算影响控制性能
    
    版本历史:
        v1.0 - 初始版本，实现基础自适应功能
    """
    
    DATA_QUALITY_A = "A"
    DATA_QUALITY_B = "B"
    DATA_QUALITY_C = "C"
    DATA_QUALITY_D = "D"
    
    def __init__(self, profile, heater, load_clean=False, register=True):
        """
        初始化自适应MPC控制器实例
        
        参数:
            profile (dict): 控制器配置参数字典，包含以下必需键:
                - block_heat_capacity (float): 加热块热容 (J/K)
                - ambient_transfer (float): 环境散热系数 (W/K)
                - heater_power (float): 加热器功率 (W)
                - sensor_responsiveness (float): 传感器响应系数
                可选键:
                - adaptive_enabled (bool): 是否启用自适应功能，默认True
                - adaptive_min_data (int): 最小数据量阈值，默认1000
                - adaptive_min_hq_data (int): 最小高质量数据量，默认300
                - adaptive_interval (float): 更新间隔(秒)，默认600.0
                - adaptive_max_change (float): 最大参数变化比例，默认0.5
                - adaptive_rmse_threshold (float): RMSE阈值(°C)，默认2.0
                - adaptive_max_failures (int): 最大连续失败次数，默认3
                - adaptive_smoothing (bool): 是否启用参数平滑，默认True
                - adaptive_smoothing_tau (float): 平滑时间常数，默认30.0
            heater (Heater): 加热器对象实例，需提供以下接口:
                - get_name(): 获取加热器名称
                - printer: 打印机主对象引用
            load_clean (bool, optional): 是否以清洁状态加载，默认False。
                为True时，控制器状态初始化为环境温度。
            register (bool, optional): 是否注册G-code命令，默认True。
                为False时，不注册命令处理函数。
        
        返回:
            None
        
        异常:
            无主动抛出异常，但可能因配置错误导致后续操作失败。
        
        示例:
            >>> profile = {"block_heat_capacity": 15.0, ...}
            >>> controller = AdaptiveMPC(profile, heater, load_clean=False)
        
        注意:
            初始化时会创建后台定时任务，用于周期性检查参数更新条件。
        """
        self.profile = profile
        self.heater = heater
        self.printer = heater.printer
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        
        self._load_adaptive_config()
        
        self.base_mpc = ControlMPC(profile, heater, load_clean, False)
        
        self.data_buffer = {
            self.DATA_QUALITY_A: [],
            self.DATA_QUALITY_B: [],
            self.DATA_QUALITY_C: [],
        }
        self.data_buffer_lock = threading.Lock()
        
        self.param_history = []
        self.current_params = self._extract_params_from_profile(profile)
        self.param_history.append({
            "params": dict(self.current_params),
            "timestamp": time.time(),
            "rmse": None,
            "status": "initial",
        })
        
        self.is_adapting = False
        self.adaptation_enabled = self.adaptive_config.get("enabled", True)
        self.last_adaptation_time = 0.0
        self.adaptation_failures = 0
        
        self.last_sample_time = 0.0
        self.sample_count = 0
        
        self.temp_error_window = []
        self.temp_error_window_size = 100
        
        if register:
            self._register_commands()
        
        self._start_background_tasks()
    
    def _load_adaptive_config(self):
        """
        加载自适应配置参数
        
        从profile配置字典中提取自适应相关参数，并设置默认值。
        该方法在初始化时被调用，用于配置参数辨识行为。
        
        参数:
            无（使用self.profile作为数据源）
        
        返回:
            None（结果存储在self.adaptive_config中）
        
        配置项说明:
            enabled (bool): 是否启用自适应功能
            min_data_count (int): 触发更新所需最小数据量
            min_high_quality_data (int): 触发更新所需最小高质量数据量
            update_interval (float): 定时更新间隔(秒)
            max_param_change (float): 单次参数最大变化比例
            rmse_threshold (float): RMSE阈值触发更新(°C)
            max_failures (int): 最大连续失败次数
            param_smoothing (bool): 是否启用参数平滑
            smoothing_tau (float): 平滑时间常数(秒)
        """
        self.adaptive_config = {
            "enabled": self.profile.get("adaptive_enabled", True),
            "min_data_count": self.profile.get("adaptive_min_data", 1000),
            "min_high_quality_data": self.profile.get("adaptive_min_hq_data", 300),
            "update_interval": self.profile.get("adaptive_interval", 600.0),
            "max_param_change": self.profile.get("adaptive_max_change", 0.5),
            "rmse_threshold": self.profile.get("adaptive_rmse_threshold", 2.0),
            "recovery_temp_margin": self.profile.get("adaptive_recovery_margin", 100.0),
            "max_failures": self.profile.get("adaptive_max_failures", 3),
            "param_smoothing": self.profile.get("adaptive_smoothing", True),
            "smoothing_tau": self.profile.get("adaptive_smoothing_tau", 30.0),
        }
    
    def _extract_params_from_profile(self, profile):
        """
        从配置字典中提取热参数
        
        从profile中提取MPC控制所需的热参数，包括热容、散热系数等。
        
        参数:
            profile (dict): 配置字典，包含热参数
        
        返回:
            dict: 热参数字典，包含以下键:
                - block_heat_capacity (float): 加热块热容 (J/K)
                - ambient_transfer (float): 环境散热系数 (W/K)
                - sensor_responsiveness (float): 传感器响应系数
                - heater_power (float): 加热器功率 (W)
        """
        return {
            "block_heat_capacity": profile.get("block_heat_capacity"),
            "ambient_transfer": profile.get("ambient_transfer"),
            "sensor_responsiveness": profile.get("sensor_responsiveness"),
            "heater_power": profile.get("heater_power"),
        }
    
    def _register_commands(self):
        """
        注册G-code命令处理函数
        
        向G-code处理器注册以下命令:
            - ADAPTIVE_MPC_SET: 设置自适应参数
            - ADAPTIVE_MPC_STATUS: 获取状态信息
            - ADAPTIVE_MPC_SWITCH: 切换控制器模式
        
        参数:
            无
        
        返回:
            None
        
        注意:
            命令使用MUX机制，支持多加热器场景
        """
        self.gcode.register_mux_command(
            "ADAPTIVE_MPC_SET",
            "HEATER",
            self.heater.get_name(),
            self.cmd_ADAPTIVE_MPC_SET,
            desc=self.cmd_ADAPTIVE_MPC_SET_help,
        )
        self.gcode.register_mux_command(
            "ADAPTIVE_MPC_STATUS",
            "HEATER",
            self.heater.get_name(),
            self.cmd_ADAPTIVE_MPC_STATUS,
            desc=self.cmd_ADAPTIVE_MPC_STATUS_help,
        )
        self.gcode.register_mux_command(
            "ADAPTIVE_MPC_SWITCH",
            "HEATER",
            self.heater.get_name(),
            self.cmd_ADAPTIVE_MPC_SWITCH,
            desc=self.cmd_ADAPTIVE_MPC_SWITCH_help,
        )
    
    cmd_ADAPTIVE_MPC_SET_help = "设置自适应MPC参数"
    
    def cmd_ADAPTIVE_MPC_SET(self, gcmd):
        """
        处理 ADAPTIVE_MPC_SET 命令
        
        该命令用于在运行时修改自适应MPC控制器的配置参数。
        支持动态启用/禁用自适应功能、调整更新间隔和RMSE阈值。
        
        参数:
            gcmd (GCodeCommand): G-code命令对象，包含以下可选参数:
                - ENABLED (int): 启用状态，0=禁用，1=启用
                - UPDATE_INTERVAL (float): 更新间隔(秒)
                - RMSE_THRESHOLD (float): RMSE阈值(°C)
        
        返回:
            None（通过gcmd.respond_info输出结果）
        
        异常:
            无主动抛出异常，参数验证由G-code解析器处理。
        
        使用示例:
            ADAPTIVE_MPC_SET HEATER=extruder ENABLED=1
            ADAPTIVE_MPC_SET HEATER=extruder UPDATE_INTERVAL=300 RMSE_THRESHOLD=1.5
        
        注意:
            参数修改立即生效，无需重启系统
        """
        enabled = gcmd.get_int("ENABLED", None, minval=0, maxval=1)
        if enabled is not None:
            self.adaptation_enabled = bool(enabled)
            gcmd.respond_info(f"自适应MPC: {'启用' if self.adaptation_enabled else '禁用'}")
        
        interval = gcmd.get_float("UPDATE_INTERVAL", None, above=0.0)
        if interval is not None:
            self.adaptive_config["update_interval"] = interval
            gcmd.respond_info(f"更新间隔设置为: {interval}秒")
        
        threshold = gcmd.get_float("RMSE_THRESHOLD", None, above=0.0)
        if threshold is not None:
            self.adaptive_config["rmse_threshold"] = threshold
            gcmd.respond_info(f"RMSE阈值设置为: {threshold}°C")
    
    cmd_ADAPTIVE_MPC_STATUS_help = "获取自适应MPC状态"
    
    def cmd_ADAPTIVE_MPC_STATUS(self, gcmd):
        """
        处理 ADAPTIVE_MPC_STATUS 命令
        
        该命令用于查询自适应MPC控制器的当前运行状态，包括数据采集情况、
        参数版本、预测误差等信息。
        
        参数:
            gcmd (GCodeCommand): G-code命令对象
        
        返回:
            None（通过gcmd.respond_info输出状态信息）
        
        输出内容:
            - 启用状态: 自适应功能是否启用
            - 正在适应: 是否正在进行参数优化
            - 数据量: 各质量等级的数据量统计
            - 当前RMSE: 当前预测误差的均方根
            - 参数版本: 当前参数版本号
            - 上次更新: 距上次参数更新的时间
            - 连续失败: 连续更新失败次数
        
        使用示例:
            ADAPTIVE_MPC_STATUS HEATER=extruder
        """
        status = self.get_status(self.reactor.monotonic())
        
        gcmd.respond_info(
            f"自适应MPC状态 ({self.heater.get_name()}):\n"
            f"  启用状态: {'启用' if status['enabled'] else '禁用'}\n"
            f"  正在适应: {'是' if status['adapting'] else '否'}\n"
            f"  数据量: A级={status['data_count_a']}, B级={status['data_count_b']}, C级={status['data_count_c']}\n"
            f"  当前RMSE: {status['current_rmse']:.2f}°C\n"
            f"  参数版本: {status['param_version']}\n"
            f"  上次更新: {status['last_update']}秒前\n"
            f"  连续失败: {status['failures']}次"
        )
    
    cmd_ADAPTIVE_MPC_SWITCH_help = "切换控制器类型"
    
    def cmd_ADAPTIVE_MPC_SWITCH(self, gcmd):
        """
        处理 ADAPTIVE_MPC_SWITCH 命令
        
        该命令用于在自适应模式和标准模式之间切换。
        在自适应模式下，控制器会自动优化参数；在标准模式下，使用固定参数。
        
        参数:
            gcmd (GCodeCommand): G-code命令对象，包含以下参数:
                - MODE (str): 目标模式，可选值:
                    - "adaptive": 自适应模式，启用参数自动优化
                    - "standard": 标准模式，使用固定参数
        
        返回:
            None（通过gcmd.respond_info输出结果）
        
        异常:
            gcmd.error: 当MODE参数无效时抛出
        
        使用示例:
            ADAPTIVE_MPC_SWITCH HEATER=extruder MODE=adaptive
            ADAPTIVE_MPC_SWITCH HEATER=extruder MODE=standard
        
        注意:
            切换操作会被记录到系统日志中
        """
        mode = gcmd.get("MODE", "adaptive").lower()
        
        if mode not in ["adaptive", "standard"]:
            raise gcmd.error(f"无效的模式: {mode}，可选: adaptive, standard")
        
        if mode == "adaptive":
            self.adaptation_enabled = True
            gcmd.respond_info(f"已切换到自适应MPC模式")
        else:
            self.adaptation_enabled = False
            gcmd.respond_info(f"已切换到标准MPC模式")
        
        self._log_switch_event(mode)
    
    def _log_switch_event(self, mode):
        """记录切换事件"""
        logging.info(
            f"AdaptiveMPC: 控制器切换事件 - 加热器: {self.heater.get_name()}, "
            f"模式: {mode}, 时间: {time.time()}"
        )
    
    def _start_background_tasks(self):
        """启动后台任务"""
        self.adaptation_timer = self.reactor.register_timer(
            self._adaptation_check_callback,
            self.reactor.NOW
        )
    
    def _adaptation_check_callback(self, eventtime):
        """周期性检查是否需要参数更新"""
        if not self.adaptation_enabled or self.is_adapting:
            return eventtime + 60.0
        
        time_since_last = eventtime - self.last_adaptation_time
        if time_since_last < self.adaptive_config["update_interval"]:
            return eventtime + 60.0
        
        should_update, reason = self._check_update_conditions(eventtime)
        
        if should_update:
            logging.info(f"AdaptiveMPC: 触发参数更新 - 原因: {reason}")
            self._trigger_adaptation(eventtime)
        
        return eventtime + 60.0
    
    def _check_update_conditions(self, eventtime):
        """
        检查是否满足参数更新条件
        
        返回：
            tuple: (是否更新, 原因说明)
        """
        total_data = sum(len(buf) for buf in self.data_buffer.values())
        hq_data = len(self.data_buffer[self.DATA_QUALITY_A])
        
        if total_data < self.adaptive_config["min_data_count"]:
            return False, f"数据量不足 ({total_data}/{self.adaptive_config['min_data_count']})"
        
        if hq_data < self.adaptive_config["min_high_quality_data"]:
            return False, f"高质量数据不足 ({hq_data}/{self.adaptive_config['min_high_quality_data']})"
        
        current_rmse = self._calculate_current_rmse()
        if current_rmse > self.adaptive_config["rmse_threshold"]:
            return True, f"RMSE超限 ({current_rmse:.2f}°C > {self.adaptive_config['rmse_threshold']}°C)"
        
        time_since_last = eventtime - self.last_adaptation_time
        if time_since_last >= self.adaptive_config["update_interval"]:
            return True, f"定时更新 (距上次{time_since_last:.0f}秒)"
        
        return False, "无需更新"
    
    def _calculate_current_rmse(self):
        """计算当前预测误差的RMSE"""
        if len(self.temp_error_window) < 10:
            return 0.0
        
        errors = self.temp_error_window[-self.temp_error_window_size:]
        return math.sqrt(sum(e**2 for e in errors) / len(errors))
    
    def _trigger_adaptation(self, eventtime):
        """触发参数适应过程"""
        self.is_adapting = True
        self.last_adaptation_time = eventtime
        
        try:
            new_params = self._optimize_parameters()
            
            if new_params is None:
                self.adaptation_failures += 1
                logging.warning("AdaptiveMPC: 参数优化失败")
                return
            
            if self._validate_parameters(new_params):
                self._apply_new_parameters(new_params)
                self.adaptation_failures = 0
                logging.info(f"AdaptiveMPC: 参数更新成功 - {new_params}")
            else:
                self.adaptation_failures += 1
                logging.warning("AdaptiveMPC: 参数验证失败")
                
                if self.adaptation_failures >= self.adaptive_config["max_failures"]:
                    self._rollback_parameters()
        
        except Exception as e:
            logging.error(f"AdaptiveMPC: 适应过程异常 - {e}")
            self.adaptation_failures += 1
        
        finally:
            self.is_adapting = False
    
    def _optimize_parameters(self):
        """
        优化热参数
        
        使用数据缓冲区中的数据进行参数辨识
        """
        all_data = []
        with self.data_buffer_lock:
            for quality, buffer in self.data_buffer.items():
                weight = self._get_quality_weight(quality)
                for sample in buffer:
                    sample_copy = dict(sample)
                    sample_copy["weight"] = weight
                    all_data.append(sample_copy)
        
        if len(all_data) < 100:
            return None
        
        params = dict(self.current_params)
        
        steady_state_data = [s for s in all_data if abs(s.get("dT_dt", 0)) < 0.02]
        if len(steady_state_data) >= 20:
            new_ambient_transfer = self._estimate_ambient_transfer(steady_state_data)
            if new_ambient_transfer is not None:
                params["ambient_transfer"] = new_ambient_transfer
        
        dynamic_data = [s for s in all_data if abs(s.get("dT_dt", 0)) >= 0.1]
        if len(dynamic_data) >= 50:
            new_heat_capacity = self._estimate_heat_capacity(dynamic_data)
            if new_heat_capacity is not None:
                params["block_heat_capacity"] = new_heat_capacity
        
        return params
    
    def _get_quality_weight(self, quality):
        """获取数据质量权重"""
        weights = {
            self.DATA_QUALITY_A: 1.0,
            self.DATA_QUALITY_B: 0.6,
            self.DATA_QUALITY_C: 0.3,
        }
        return weights.get(quality, 0.1)
    
    def _estimate_ambient_transfer(self, steady_data):
        """从稳态数据估计环境散热系数"""
        power_temp_pairs = []
        for sample in steady_data:
            power = sample.get("power_watts", 0)
            temp = sample.get("temperature", 0)
            if power > 0 and temp > AMBIENT_TEMP + 20:
                power_temp_pairs.append((power, temp))
        
        if len(power_temp_pairs) < 10:
            return None
        
        total_power = sum(p for p, _ in power_temp_pairs)
        total_temp_diff = sum(t - AMBIENT_TEMP for _, t in power_temp_pairs)
        
        if total_temp_diff > 0:
            return total_power / total_temp_diff
        return None
    
    def _estimate_heat_capacity(self, dynamic_data):
        """从动态数据估计热容"""
        valid_samples = [s for s in dynamic_data if s.get("power_watts", 0) > 0]
        
        if len(valid_samples) < 20:
            return None
        
        hc_estimates = []
        for sample in valid_samples:
            power = sample.get("power_watts", 0)
            dT_dt = sample.get("dT_dt", 0)
            if dT_dt > 0.01:
                hc_estimates.append(power / dT_dt)
        
        if len(hc_estimates) < 10:
            return None
        
        hc_estimates.sort()
        n = len(hc_estimates)
        trimmed = hc_estimates[n//4 : 3*n//4]
        return sum(trimmed) / len(trimmed)
    
    def _validate_parameters(self, new_params):
        """
        验证新参数的有效性
        
        检查：
        1. 参数正值
        2. 参数变化幅度
        3. 物理合理性
        """
        for key, value in new_params.items():
            if value is None or value <= 0:
                logging.warning(f"AdaptiveMPC: 参数{key}无效: {value}")
                return False
        
        max_change = self.adaptive_config["max_param_change"]
        for key in ["block_heat_capacity", "ambient_transfer", "sensor_responsiveness"]:
            old_val = self.current_params.get(key, 0)
            new_val = new_params.get(key, 0)
            if old_val > 0:
                change = abs(new_val - old_val) / old_val
                if change > max_change:
                    logging.warning(
                        f"AdaptiveMPC: 参数{key}变化过大: {change*100:.1f}% > {max_change*100:.1f}%"
                    )
                    return False
        
        if new_params["block_heat_capacity"] < 5 or new_params["block_heat_capacity"] > 100:
            logging.warning(f"AdaptiveMPC: 热容超出合理范围: {new_params['block_heat_capacity']}")
            return False
        
        return True
    
    def _apply_new_parameters(self, new_params):
        """应用新参数"""
        old_params = dict(self.current_params)
        
        if self.adaptive_config["param_smoothing"]:
            smoothed_params = self._smooth_parameters(old_params, new_params)
        else:
            smoothed_params = new_params
        
        self.current_params = smoothed_params
        
        self.base_mpc.const_block_heat_capacity = smoothed_params["block_heat_capacity"]
        self.base_mpc.const_ambient_transfer = smoothed_params["ambient_transfer"]
        self.base_mpc.const_sensor_responsiveness = smoothed_params["sensor_responsiveness"]
        
        self.param_history.append({
            "params": dict(smoothed_params),
            "timestamp": time.time(),
            "rmse": self._calculate_current_rmse(),
            "status": "active",
        })
        
        if len(self.param_history) > 10:
            self.param_history = self.param_history[-10:]
        
        self._clear_data_buffer()
    
    def _smooth_parameters(self, old_params, new_params):
        """平滑参数过渡"""
        tau = self.adaptive_config["smoothing_tau"]
        alpha = 1 - math.exp(-1.0 / tau)
        
        smoothed = {}
        for key in old_params:
            if key in new_params:
                smoothed[key] = old_params[key] + alpha * (new_params[key] - old_params[key])
            else:
                smoothed[key] = old_params[key]
        
        return smoothed
    
    def _rollback_parameters(self):
        """回滚到上一个有效参数版本"""
        for record in reversed(self.param_history[:-1]):
            if record["status"] == "active":
                self.current_params = dict(record["params"])
                
                self.base_mpc.const_block_heat_capacity = self.current_params["block_heat_capacity"]
                self.base_mpc.const_ambient_transfer = self.current_params["ambient_transfer"]
                self.base_mpc.const_sensor_responsiveness = self.current_params["sensor_responsiveness"]
                
                self.param_history.append({
                    "params": dict(self.current_params),
                    "timestamp": time.time(),
                    "rmse": None,
                    "status": "rolled_back",
                })
                
                logging.info("AdaptiveMPC: 参数已回滚")
                self.adaptation_failures = 0
                return
        
        logging.warning("AdaptiveMPC: 无有效历史参数可回滚")
    
    def _clear_data_buffer(self):
        """清空数据缓冲区"""
        with self.data_buffer_lock:
            for key in self.data_buffer:
                self.data_buffer[key] = []
    
    def _assess_data_quality(self, sample):
        """
        评估数据质量等级
        
        返回：
            str: 质量等级 (A/B/C/D)
        """
        dT_dt = abs(sample.get("dT_dt", 0))
        dP_dt = abs(sample.get("dP_dt", 0))
        
        if dT_dt >= 0.5 or dP_dt >= 5.0:
            return self.DATA_QUALITY_A
        elif dT_dt >= 0.1 or dP_dt >= 1.0:
            return self.DATA_QUALITY_B
        elif dT_dt >= 0.02:
            return self.DATA_QUALITY_C
        else:
            return self.DATA_QUALITY_D
    
    def temperature_update(self, read_time, temp, target_temp):
        """
        温度更新回调 - 主要控制入口
        
        该方法是控制器的核心入口，由加热器在每次温度采样时调用。
        主要功能包括：
        1. 数据采集：记录温度、功率等信息
        2. 数据质量评估：根据动态特性评估数据质量
        3. 误差计算：计算预测误差用于RMSE评估
        4. 控制执行：委托给基础MPC控制器执行控制
        
        参数:
            read_time (float): 温度读取时间戳（reactor时间，单位：秒）
            temp (float): 当前温度值（单位：°C）
            target_temp (float): 目标温度值（单位：°C）
        
        返回:
            None（控制输出通过heater.set_pwm执行）
        
        副作用:
            - 更新数据缓冲区（self.data_buffer）
            - 更新温度误差窗口（self.temp_error_window）
            - 更新上次采样时间和温度
            - 调用基础MPC控制器的temperature_update方法
        
        线程安全:
            数据缓冲区操作使用self.data_buffer_lock保护
        
        注意:
            如果正在进行参数适应（self.is_adapting为True），
            则跳过数据采集，仅执行基础控制
        """
        if self.is_adapting:
            self.base_mpc.temperature_update(read_time, temp, target_temp)
            return
        
        dt = read_time - self.last_sample_time
        if self.last_sample_time == 0.0 or dt < 0.0 or dt > 1.0:
            dt = 0.1
        self.last_sample_time = read_time
        
        sample = {
            "time": read_time,
            "temperature": temp,
            "target": target_temp,
            "power_watts": self.base_mpc.last_power,
            "pwm": self.base_mpc.last_power / self.base_mpc.const_heater_power if self.base_mpc.const_heater_power > 0 else 0,
        }
        
        if hasattr(self, "_last_temp"):
            dT = temp - self._last_temp
            sample["dT_dt"] = dT / dt if dt > 0 else 0
        else:
            sample["dT_dt"] = 0
        
        if hasattr(self, "_last_power"):
            dP = self.base_mpc.last_power - self._last_power
            sample["dP_dt"] = dP / dt if dt > 0 else 0
        else:
            sample["dP_dt"] = 0
        
        self._last_temp = temp
        self._last_power = self.base_mpc.last_power
        
        if self.adaptation_enabled:
            quality = self._assess_data_quality(sample)
            if quality != self.DATA_QUALITY_D:
                with self.data_buffer_lock:
                    self.data_buffer[quality].append(sample)
                    
                    max_buffer = 5000 if quality == self.DATA_QUALITY_A else 3000
                    if len(self.data_buffer[quality]) > max_buffer:
                        self.data_buffer[quality] = self.data_buffer[quality][-max_buffer:]
        
        predicted_temp = self.base_mpc.state_sensor_temp
        error = temp - predicted_temp
        self.temp_error_window.append(error)
        if len(self.temp_error_window) > self.temp_error_window_size:
            self.temp_error_window.pop(0)
        
        self.base_mpc.temperature_update(read_time, temp, target_temp)
    
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        """
        检查控制器是否忙碌
        
        该方法用于判断控制器是否正在进行温度调节。
        委托给基础MPC控制器实现。
        
        参数:
            eventtime (float): 当前事件时间戳（reactor时间）
            smoothed_temp (float): 平滑后的温度值（°C）
            target_temp (float): 目标温度值（°C）
        
        返回:
            bool: True表示控制器正在调节温度，False表示已达到稳态
        """
        return self.base_mpc.check_busy(eventtime, smoothed_temp, target_temp)
    
    def update_smooth_time(self):
        """
        更新平滑时间参数
        
        当加热器的平滑时间配置发生变化时调用此方法。
        委托给基础MPC控制器实现。
        
        参数:
            无
        
        返回:
            None
        """
        self.base_mpc.update_smooth_time()
    
    def get_profile(self):
        """
        获取控制器配置参数
        
        返回当前控制器的完整配置参数字典，包括基础MPC参数
        和自适应功能启用状态。
        
        参数:
            无
        
        返回:
            dict: 配置参数字典，包含:
                - 基础MPC参数（block_heat_capacity等）
                - adaptive_enabled (bool): 自适应功能启用状态
        """
        profile = dict(self.base_mpc.get_profile())
        profile["adaptive_enabled"] = self.adaptation_enabled
        return profile
    
    def get_type(self):
        """
        获取控制器类型标识
        
        用于在系统中标识控制器类型。
        
        参数:
            无
        
        返回:
            str: 控制器类型字符串 "adaptive_mpc"
        """
        return "adaptive_mpc"
    
    def get_status(self, eventtime):
        """
        获取控制器完整状态信息
        
        返回控制器的详细状态信息，用于状态查询和监控。
        
        参数:
            eventtime (float): 当前事件时间戳（reactor时间）
        
        返回:
            dict: 状态信息字典，包含:
                - 基础MPC状态（继承自base_mpc.get_status()）
                - enabled (bool): 自适应功能启用状态
                - adapting (bool): 是否正在进行参数适应
                - data_count_a (int): A级数据量
                - data_count_b (int): B级数据量
                - data_count_c (int): C级数据量
                - current_rmse (float): 当前预测误差RMSE
                - param_version (int): 参数版本号
                - last_update (float): 距上次更新的时间（秒）
                - failures (int): 连续失败次数
                - controller_mode (str): 当前模式（"adaptive"或"standard"）
        """
        base_status = self.base_mpc.get_status(eventtime)
        
        return {
            **base_status,
            "enabled": self.adaptation_enabled,
            "adapting": self.is_adapting,
            "data_count_a": len(self.data_buffer[self.DATA_QUALITY_A]),
            "data_count_b": len(self.data_buffer[self.DATA_QUALITY_B]),
            "data_count_c": len(self.data_buffer[self.DATA_QUALITY_C]),
            "current_rmse": self._calculate_current_rmse(),
            "param_version": len(self.param_history),
            "last_update": eventtime - self.last_adaptation_time if self.last_adaptation_time > 0 else 0,
            "failures": self.adaptation_failures,
            "controller_mode": "adaptive" if self.adaptation_enabled else "standard",
        }


class MPCControllerManager:
    """
    MPC控制器管理器
    
    该类实现标准MPC与自适应MPC之间的无缝切换功能，是Klipper/Kalico
    温度控制系统的高级管理组件。通过组合模式同时持有两种控制器实例，
    支持在运行时动态切换控制器类型，满足不同工况下的控制需求。
    
    设计目的:
        在实际3D打印过程中，不同阶段对温度控制的需求不同：
        - 预热阶段：需要快速响应，可使用标准MPC
        - 打印阶段：需要精确控制，可使用自适应MPC
        - 调试阶段：需要稳定参数，可使用标准MPC
        本类提供了灵活的切换机制，无需重启系统即可切换控制器。
    
    核心特性:
        - 运行时动态切换：通过G-code命令实时切换控制器类型
        - 状态平滑传递：切换时自动传递温度状态和参数，确保无扰动
        - 切换过程无扰动：切换操作原子化，不影响正在进行的控制
        - 完整的事件记录：记录所有切换事件，便于故障排查和性能分析
        - 双控制器并行：同时维护两个控制器实例，切换开销极低
    
    使用场景:
        - 需要在打印过程中切换控制策略
        - 调试和对比不同控制器的性能
        - 在自适应参数收敛后切换到稳定模式
        - 根据打印阶段自动调整控制策略
    
    继承关系:
        该类不继承自其他类，采用组合模式持有ControlMPC和AdaptiveMPC实例。
    
    线程安全:
        切换操作通过switch_in_progress标志保护，防止并发切换。
    
    示例:
        >>> profile = {"initial_mode": "adaptive", ...}
        >>> manager = MPCControllerManager(profile, heater)
        >>> manager.temperature_update(read_time, temp, target_temp)
        >>> # 通过G-code切换: MPC_SWITCH HEATER=extruder MODE=standard
    
    属性:
        profile (dict): 控制器配置参数字典
        heater (Heater): 加热器对象实例
        printer (Printer): 打印机主对象
        reactor (Reactor): 事件反应器
        gcode (GCode): G-code命令处理器
        standard_mpc (ControlMPC): 标准MPC控制器实例
        adaptive_mpc (AdaptiveMPC): 自适应MPC控制器实例
        current_mode (str): 当前模式，"standard"或"adaptive"
        active_controller: 当前活动的控制器实例
        switch_history (list): 切换历史记录列表
        switch_in_progress (bool): 是否正在进行切换操作
        state_transfer_enabled (bool): 是否启用状态传递
    
    异常:
        切换过程中可能抛出gcmd.error异常（无效模式参数等）。
    
    注意事项:
        - 切换操作是原子性的，不会中断正在进行的控制
        - 状态传递确保切换后控制连续性
        - 建议在温度稳定时进行切换，避免在快速升温/降温时切换
    
    版本历史:
        v1.0 - 初始版本，实现基础切换功能
    """
    
    MODE_STANDARD = "standard"
    MODE_ADAPTIVE = "adaptive"
    
    def __init__(self, profile, heater, load_clean=False, register=True):
        """
        初始化MPC控制器管理器实例
        
        创建并初始化标准MPC和自适应MPC两个控制器实例，
        根据配置设置初始活动控制器。
        
        参数:
            profile (dict): 控制器配置参数字典，包含:
                - 基础MPC参数（block_heat_capacity等）
                - initial_mode (str, optional): 初始模式，"standard"或"adaptive"，默认"adaptive"
                - 自适应MPC相关参数（当使用adaptive模式时）
            heater (Heater): 加热器对象实例，需提供:
                - get_name(): 获取加热器名称
                - printer: 打印机主对象引用
            load_clean (bool, optional): 是否以清洁状态加载，默认False
            register (bool, optional): 是否注册G-code命令，默认True
        
        返回:
            None
        
        异常:
            无主动抛出异常，但配置错误可能导致后续操作失败。
        
        示例:
            >>> profile = {
            ...     "initial_mode": "adaptive",
            ...     "block_heat_capacity": 15.0,
            ...     "adaptive_enabled": True,
            ... }
            >>> manager = MPCControllerManager(profile, heater)
        
        注意:
            初始化时会创建两个完整的控制器实例，占用一定内存资源
        """
        self.profile = profile
        self.heater = heater
        self.printer = heater.printer
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        
        self.standard_mpc = ControlMPC(profile, heater, load_clean, False)
        self.adaptive_mpc = AdaptiveMPC(profile, heater, load_clean, False)
        
        initial_mode = profile.get("initial_mode", self.MODE_ADAPTIVE)
        self.current_mode = initial_mode
        
        if self.current_mode == self.MODE_ADAPTIVE:
            self.active_controller = self.adaptive_mpc
        else:
            self.active_controller = self.standard_mpc
        
        self.switch_history = []
        self.switch_in_progress = False
        self.state_transfer_enabled = True
        
        if register:
            self._register_commands()
        
        self._log_controller_status("初始化完成")
    
    def _register_commands(self):
        """
        注册G-code命令处理函数
        
        向G-code处理器注册以下命令:
            - MPC_SWITCH: 切换控制器类型
            - MPC_MODE: 查询或设置当前模式
            - MPC_MANAGER_STATUS: 获取管理器状态
        
        参数:
            无
        
        返回:
            None
        
        注意:
            命令使用MUX机制，支持多加热器场景
        """
        self.gcode.register_mux_command(
            "MPC_SWITCH",
            "HEATER",
            self.heater.get_name(),
            self.cmd_MPC_SWITCH,
            desc=self.cmd_MPC_SWITCH_help,
        )
        self.gcode.register_mux_command(
            "MPC_MODE",
            "HEATER",
            self.heater.get_name(),
            self.cmd_MPC_MODE,
            desc=self.cmd_MPC_MODE_help,
        )
        self.gcode.register_mux_command(
            "MPC_MANAGER_STATUS",
            "HEATER",
            self.heater.get_name(),
            self.cmd_MPC_MANAGER_STATUS,
            desc=self.cmd_MPC_MANAGER_STATUS_help,
        )
    
    cmd_MPC_SWITCH_help = "切换MPC控制器类型"
    
    def cmd_MPC_SWITCH(self, gcmd):
        """
        处理 MPC_SWITCH 命令
        
        该命令用于在标准MPC和自适应MPC之间切换。
        支持配置是否在切换时传递状态。
        
        参数:
            gcmd (GCodeCommand): G-code命令对象，包含以下参数:
                - MODE (str): 目标模式，可选值:
                    - "standard": 标准MPC模式
                    - "adaptive": 自适应MPC模式
                - STATE_TRANSFER (int, optional): 是否传递状态，0=否，1=是，默认1
        
        返回:
            None（通过gcmd.respond_info输出结果）
        
        异常:
            gcmd.error: 当切换正在进行或MODE参数无效时抛出
        
        使用示例:
            MPC_SWITCH HEATER=extruder MODE=adaptive
            MPC_SWITCH HEATER=extruder MODE=standard STATE_TRANSFER=0
        
        注意:
            - 切换操作是原子性的，耗时通常小于5ms
            - 如果目标模式与当前模式相同，则不执行切换
        """
        if self.switch_in_progress:
            raise gcmd.error("切换操作正在进行中，请稍候")
        
        mode = gcmd.get("MODE", self.MODE_ADAPTIVE).lower()
        if mode not in [self.MODE_STANDARD, self.MODE_ADAPTIVE]:
            raise gcmd.error(f"无效的模式: {mode}，可选: standard, adaptive")
        
        state_transfer = gcmd.get_int("STATE_TRANSFER", 1, minval=0, maxval=1)
        self.state_transfer_enabled = bool(state_transfer)
        
        if mode == self.current_mode:
            gcmd.respond_info(f"控制器已处于 {mode} 模式，无需切换")
            return
        
        self._perform_switch(mode, gcmd)
    
    cmd_MPC_MODE_help = "获取或设置当前MPC模式"
    
    def cmd_MPC_MODE(self, gcmd):
        """
        处理 MPC_MODE 命令
        
        该命令用于查询当前模式或设置新模式。
        当不提供MODE参数时，仅显示当前状态。
        
        参数:
            gcmd (GCodeCommand): G-code命令对象，包含以下可选参数:
                - MODE (str, optional): 目标模式，不提供则仅查询状态
                    - "standard": 标准MPC模式
                    - "adaptive": 自适应MPC模式
        
        返回:
            None（通过gcmd.respond_info输出结果）
        
        异常:
            gcmd.error: 当MODE参数无效时抛出
        
        使用示例:
            ; 查询当前模式
            MPC_MODE HEATER=extruder
            
            ; 设置新模式
            MPC_MODE HEATER=extruder MODE=adaptive
        """
        mode = gcmd.get("MODE", None)
        
        if mode is None:
            gcmd.respond_info(
                f"当前MPC模式: {self.current_mode}\n"
                f"活动控制器: {self.active_controller.get_type()}"
            )
            return
        
        mode = mode.lower()
        if mode not in [self.MODE_STANDARD, self.MODE_ADAPTIVE]:
            raise gcmd.error(f"无效的模式: {mode}，可选: standard, adaptive")
        
        if mode == self.current_mode:
            gcmd.respond_info(f"控制器已处于 {mode} 模式")
            return
        
        self._perform_switch(mode, gcmd)
    
    cmd_MPC_MANAGER_STATUS_help = "获取MPC管理器状态"
    
    def cmd_MPC_MANAGER_STATUS(self, gcmd):
        """
        处理 MPC_MANAGER_STATUS 命令
        
        该命令用于查询MPC控制器管理器的详细状态信息，
        包括当前模式、切换历史等。
        
        参数:
            gcmd (GCodeCommand): G-code命令对象
        
        返回:
            None（通过gcmd.respond_info输出状态信息）
        
        输出内容:
            - 当前模式: 当前活动的控制器模式
            - 活动控制器: 当前活动控制器的类型标识
            - 切换次数: 历史切换总次数
            - 状态传递: 是否启用状态传递功能
            - 切换历史: 最近5次切换的详细记录
        
        使用示例:
            MPC_MANAGER_STATUS HEATER=extruder
        """
        status = self.get_manager_status(self.reactor.monotonic())
        
        gcmd.respond_info(
            f"MPC控制器管理器状态 ({self.heater.get_name()}):\n"
            f"  当前模式: {status['current_mode']}\n"
            f"  活动控制器: {status['active_controller']}\n"
            f"  切换次数: {status['switch_count']}\n"
            f"  状态传递: {'启用' if status['state_transfer_enabled'] else '禁用'}\n"
            f"  切换历史:\n{status['switch_history_formatted']}"
        )
    
    def _perform_switch(self, target_mode, gcmd=None):
        """
        执行控制器切换的核心方法
        
        该方法实现控制器切换的完整流程，包括：
        1. 设置切换标志防止并发切换
        2. 选择目标控制器实例
        3. 执行状态传递（如果启用）
        4. 更新活动控制器引用
        5. 记录切换历史
        6. 处理异常情况
        
        参数:
            target_mode (str): 目标模式，"standard"或"adaptive"
            gcmd (GCodeCommand, optional): G-code命令对象，用于输出结果
        
        返回:
            None
        
        副作用:
            - 更新self.active_controller
            - 更新self.current_mode
            - 添加记录到self.switch_history
            - 写入系统日志
        
        异常:
            切换失败时会回滚到原控制器，并通过gcmd.error报告错误
        
        注意:
            该方法是内部方法，应通过cmd_MPC_SWITCH或cmd_MPC_MODE调用
        """
        self.switch_in_progress = True
        old_mode = self.current_mode
        old_controller = self.active_controller
        switch_start_time = time.time()
        
        try:
            if target_mode == self.MODE_ADAPTIVE:
                new_controller = self.adaptive_mpc
            else:
                new_controller = self.standard_mpc
            
            if self.state_transfer_enabled:
                self._transfer_state(old_controller, new_controller)
            
            self.active_controller = new_controller
            self.current_mode = target_mode
            
            switch_duration = time.time() - switch_start_time
            
            switch_record = {
                "timestamp": switch_start_time,
                "from_mode": old_mode,
                "to_mode": target_mode,
                "duration": switch_duration,
                "state_transferred": self.state_transfer_enabled,
                "success": True,
            }
            self.switch_history.append(switch_record)
            
            if len(self.switch_history) > 20:
                self.switch_history = self.switch_history[-20:]
            
            self._log_switch_event(old_mode, target_mode, switch_duration)
            
            if gcmd:
                gcmd.respond_info(
                    f"控制器切换成功: {old_mode} -> {target_mode}\n"
                    f"切换耗时: {switch_duration*1000:.2f}ms\n"
                    f"状态传递: {'是' if self.state_transfer_enabled else '否'}"
                )
        
        except Exception as e:
            self.active_controller = old_controller
            self.current_mode = old_mode
            
            switch_record = {
                "timestamp": switch_start_time,
                "from_mode": old_mode,
                "to_mode": target_mode,
                "duration": time.time() - switch_start_time,
                "state_transferred": False,
                "success": False,
                "error": str(e),
            }
            self.switch_history.append(switch_record)
            
            logging.error(f"MPCControllerManager: 切换失败 - {e}")
            
            if gcmd:
                raise gcmd.error(f"控制器切换失败: {e}")
        
        finally:
            self.switch_in_progress = False
    
    def _transfer_state(self, from_controller, to_controller):
        """
        在控制器之间传递状态
        
        该方法实现控制器切换时的状态传递，确保切换后控制连续性。
        传递的状态包括：
        1. 温度状态（加热块温度、传感器温度、环境温度）
        2. 功率状态（上次输出功率）
        3. 热参数（热容、散热系数等）
        
        参数:
            from_controller: 源控制器实例
            to_controller: 目标控制器实例
        
        返回:
            None
        
        传递的状态属性:
            - state_block_temp: 加热块温度状态
            - state_sensor_temp: 传感器温度状态
            - state_ambient_temp: 环境温度状态
            - target_temp: 目标温度
            - last_power: 上次输出功率
            - const_block_heat_capacity: 热容参数
            - const_ambient_transfer: 散热系数参数
            - const_sensor_responsiveness: 传感器响应系数
            - const_heater_power: 加热器功率
        
        注意:
            - 该方法会自动处理AdaptiveMPC的base_mpc嵌套结构
            - 状态传递失败不会中断切换流程
        """
        state_attrs = [
            "state_block_temp",
            "state_sensor_temp",
            "state_ambient_temp",
            "target_temp",
        ]
        
        for attr in state_attrs:
            if hasattr(from_controller, attr):
                value = getattr(from_controller, attr)
                if hasattr(to_controller, attr):
                    setattr(to_controller, attr, value)
                elif hasattr(to_controller, "base_mpc") and hasattr(to_controller.base_mpc, attr):
                    setattr(to_controller.base_mpc, attr, value)
        
        if hasattr(from_controller, "last_power"):
            last_power = from_controller.last_power
            if hasattr(to_controller, "last_power"):
                to_controller.last_power = last_power
            elif hasattr(to_controller, "base_mpc") and hasattr(to_controller.base_mpc, "last_power"):
                to_controller.base_mpc.last_power = last_power
        
        if hasattr(from_controller, "const_block_heat_capacity"):
            params_to_sync = [
                "const_block_heat_capacity",
                "const_ambient_transfer",
                "const_sensor_responsiveness",
                "const_heater_power",
            ]
            for param in params_to_sync:
                if hasattr(from_controller, param):
                    value = getattr(from_controller, param)
                    if hasattr(to_controller, param):
                        setattr(to_controller, param, value)
                    elif hasattr(to_controller, "base_mpc") and hasattr(to_controller.base_mpc, param):
                        setattr(to_controller.base_mpc, param, value)
        
        logging.debug(
            f"MPCControllerManager: 状态传递完成 - "
            f"block_temp={getattr(from_controller, 'state_block_temp', 'N/A')}, "
            f"sensor_temp={getattr(from_controller, 'state_sensor_temp', 'N/A')}"
        )
    
    def _log_switch_event(self, from_mode, to_mode, duration):
        """记录切换事件"""
        logging.info(
            f"MPCControllerManager: 控制器切换 - "
            f"加热器: {self.heater.get_name()}, "
            f"模式: {from_mode} -> {to_mode}, "
            f"耗时: {duration*1000:.2f}ms"
        )
    
    def _log_controller_status(self, message):
        """记录控制器状态"""
        logging.info(
            f"MPCControllerManager: {message} - "
            f"加热器: {self.heater.get_name()}, "
            f"模式: {self.current_mode}, "
            f"控制器: {self.active_controller.get_type()}"
        )
    
    def temperature_update(self, read_time, temp, target_temp):
        """
        温度更新回调 - 委托给活动控制器
        
        该方法是管理器的主要控制入口，由加热器在每次温度采样时调用。
        通过委托模式将调用转发给当前活动的控制器实例。
        
        参数:
            read_time (float): 温度读取时间戳（reactor时间，单位：秒）
            temp (float): 当前温度值（单位：°C）
            target_temp (float): 目标温度值（单位：°C）
        
        返回:
            None（控制输出通过heater.set_pwm执行）
        
        委托行为:
            - 当current_mode为"adaptive"时，委托给adaptive_mpc
            - 当current_mode为"standard"时，委托给standard_mpc
        """
        self.active_controller.temperature_update(read_time, temp, target_temp)
    
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        """
        检查控制器是否忙碌
        
        委托给当前活动控制器实现，用于判断是否正在进行温度调节。
        
        参数:
            eventtime (float): 当前事件时间戳（reactor时间）
            smoothed_temp (float): 平滑后的温度值（°C）
            target_temp (float): 目标温度值（°C）
        
        返回:
            bool: True表示控制器正在调节温度，False表示已达到稳态
        """
        return self.active_controller.check_busy(eventtime, smoothed_temp, target_temp)
    
    def update_smooth_time(self):
        """
        更新平滑时间参数
        
        委托给当前活动控制器实现。
        当加热器的平滑时间配置发生变化时调用此方法。
        
        参数:
            无
        
        返回:
            None
        """
        self.active_controller.update_smooth_time()
    
    def get_profile(self):
        """
        获取控制器配置参数
        
        返回当前活动控制器的配置参数，并附加当前模式信息。
        
        参数:
            无
        
        返回:
            dict: 配置参数字典，包含:
                - 活动控制器的所有配置参数
                - current_mode (str): 当前模式
        """
        profile = dict(self.active_controller.get_profile())
        profile["current_mode"] = self.current_mode
        return profile
    
    def get_type(self):
        """
        获取控制器类型标识
        
        返回包含当前模式的类型标识字符串。
        
        参数:
            无
        
        返回:
            str: 控制器类型字符串，格式为"managed_{mode}_mpc"
                 例如："managed_adaptive_mpc"或"managed_standard_mpc"
        """
        return f"managed_{self.current_mode}_mpc"
    
    def get_status(self, eventtime):
        """
        获取控制器状态信息
        
        返回当前活动控制器的状态信息，并附加管理器相关状态。
        
        参数:
            eventtime (float): 当前事件时间戳（reactor时间）
        
        返回:
            dict: 状态信息字典，包含:
                - 活动控制器的所有状态信息
                - current_mode (str): 当前模式
                - active_controller (str): 活动控制器类型
                - switch_count (int): 切换次数
                - state_transfer_enabled (bool): 状态传递启用状态
        """
        base_status = self.active_controller.get_status(eventtime)
        
        return {
            **base_status,
            "current_mode": self.current_mode,
            "active_controller": self.active_controller.get_type(),
            "switch_count": len(self.switch_history),
            "state_transfer_enabled": self.state_transfer_enabled,
        }
    
    def get_manager_status(self, eventtime):
        """
        获取管理器详细状态信息
        
        返回管理器的详细状态信息，包括格式化的切换历史记录。
        该方法主要用于MPC_MANAGER_STATUS命令的输出。
        
        参数:
            eventtime (float): 当前事件时间戳（reactor时间）
        
        返回:
            dict: 管理器状态字典，包含:
                - current_mode (str): 当前模式
                - active_controller (str): 活动控制器类型
                - switch_count (int): 历史切换总次数
                - state_transfer_enabled (bool): 状态传递启用状态
                - switch_history_formatted (str): 格式化的切换历史（最近5条）
        """
        history_lines = []
        for record in self.switch_history[-5:]:
            status = "成功" if record["success"] else f"失败({record.get('error', 'unknown')})"
            history_lines.append(
                f"    - {record['from_mode']} -> {record['to_mode']}: "
                f"{status}, {record['duration']*1000:.2f}ms"
            )
        
        return {
            "current_mode": self.current_mode,
            "active_controller": self.active_controller.get_type(),
            "switch_count": len(self.switch_history),
            "state_transfer_enabled": self.state_transfer_enabled,
            "switch_history_formatted": "\n".join(history_lines) if history_lines else "    无切换记录",
        }


