# AGENTS.md

## Scope
- Applies to the whole repository.

## What this repo is
- Kalico is a community-maintained fork of Klipper. Treat upstream Klipper assumptions carefully; this repo adds behavior and modules beyond mainline Klipper (`README.md`, `docs/Kalico_Additions.md`).
- The repo contains both host-side Python code and MCU firmware code. Many changes require checking both sides.

## High-value directories
- `klippy/`: host firmware runtime in Python.
  - Entry point is `python -m klippy` / `klippy/klippy.py`, which dispatches to `klippy.printer.main()`.
  - `klippy/extras/` contains auto-loaded host modules.
  - `klippy/plugins/` is also scanned at startup; plugin names can override `extras` only when `danger_options.allow_plugin_override` is enabled in config (`klippy/printer.py`).
- `src/`: MCU firmware C sources. Architecture-specific code lives under subdirs like `src/avr`, `src/stm32`, etc.
- `test/`: pytest-based test suite plus custom `.test` regression collector for Klippy integration-style tests.
- `test/configs/`: canonical MCU `.config` fixtures used for firmware compile coverage in CI.
- `docs/`: user/dev docs. `docs/Code_Overview.md`, `docs/Debugging.md`, and `docs/CONTRIBUTING.md` are the main sources of truth for architecture and contribution expectations.
- `docs/_kalico/`: MkDocs project for documentation site.
- `scripts/`: build/test helpers, regression runners, CI container assets, debug tools.

## Developer environment and core commands
- Python baseline is `>=3.9` (`pyproject.toml`).
- Install dev deps with uv, not ad-hoc pip, if you need the repo-managed environment:
  - `uv sync --dev`
- Ruff is the only configured formatter/linter in this repo:
  - `uv run ruff check .`
  - `uv run ruff format .`
- Pre-commit runs Ruff with `--fix` and `ruff-format` and excludes `docs/`, `config/`, and `lib/` (`.pre-commit-config.yaml`).
  - `uv run pre-commit run --all-files`

## Test and verification workflow
- Fast host-side pytest:
  - `uv run pytest`
- CI runs pytest across Python 3.9 through 3.14 inside the Docker build image, usually with xdist (`.github/workflows/ci-build_test.yaml`).
- The custom `.test` regression suite is collected by `test/klippy/conftest.py`. Those tests invoke `python -m klippy ...` and require MCU dictionary files via `--dictdir` / `DICTDIR`.
- `test/conftest.py` eagerly builds/loads `klippy.chelper` via `klippy.chelper.get_ffi()`. If tests fail very early, suspect chelper/native build prerequisites first.
- Useful focused test invocations:
  - Single pytest file: `uv run pytest test/test_autosave.py`
  - Single pytest test: `uv run pytest test/test_autosave.py::test_autosave_includes`
  - Klippy `.test` regression subset (requires dictionaries): `uv run pytest test/klippy -k bed_mesh`
- Full CI-style local verification is driven by the container:
  - `docker build -f scripts/Dockerfile-build -t dangerklippers/klipper-build:latest .`
  - Then, for example: `docker run -v ${PWD}:/klipper dangerklippers/klipper-build:latest --python 3.12 py.test -n auto`
- `scripts/ci-build.sh` is the executable source of truth for firmware+pytest CI behavior:
  - It compiles every `test/configs/*.config`
  - Copies generated `out/klipper.dict` files into `DICTDIR`
  - Then runs `py.test`

## Firmware build facts that are easy to miss
- Top-level `Makefile` builds MCU firmware, not the host Python package.
- Common firmware flow:
  - `make menuconfig`
  - `make`
- Build artifacts go under `out/`. `docs/Code_Overview.md` notes final outputs like `out/klipper.bin` on ARM or `out/klipper.elf.hex` on AVR.
- The build includes `src/extras/Makefile` automatically. Firmware extensions under `src/extras/<name>/` need both `Kconfig` and `Makefile` wiring to participate in build/menuconfig (`docs/Code_Overview.md`).

## Host module conventions
- New host modules should usually be added under `klippy/extras/` and loaded through config sections.
- Kalico auto-loads modules by section name:
  - `[my_module]` -> `load_config(config)` in `klippy/extras/my_module.py`
  - `[my_module name]` -> `load_config_prefix(config)`
- This convention is real and widely used across `klippy/extras/`; follow existing modules instead of inventing alternate registration patterns.
- `docs/Code_Overview.md` is the best local guide for module lifecycle, event hooks, and object lookup conventions. Read it before adding or restructuring extras.

## Testing helpers and quirks
- `test/conftest.py` creates a symlink from `test/klippy_testing_plugin.py` into `klippy/plugins/testing.py` for the test session. Do not break this plugin-loading path when changing plugin/module discovery.
- The `test/klippy_testing/` shims are used by unit-style tests that instantiate partial printer/config objects without a full runtime.
- Old-style regression helper `scripts/test_klippy.py` still exists and is referenced in `docs/Debugging.md`, but pytest collection under `test/klippy/conftest.py` is the current in-repo mechanism.

## Docs workflow
- Docs are built from `docs/` using the MkDocs project in `docs/_kalico/`.
- Strict docs build command:
  - `cd docs/_kalico && uv run mkdocs build --strict`
- If you add a new documentation page, also update `docs/_kalico/mkdocs.yml` nav; `docs/CONTRIBUTING.md` explicitly calls this out.

## Required doc updates for user-facing changes
- `docs/CONTRIBUTING.md` is explicit: user-facing code changes must update the reference docs.
- At minimum, update the matching source when relevant:
  - G-code / command params -> `docs/G-Codes.md`
  - Config modules / params -> `docs/Config_Reference.md`
  - Status variables -> `docs/Status_Reference.md`
  - Webhooks / API params -> `docs/API_Server.md`
  - Breaking config/command changes -> `docs/Config_Changes.md`

## Style and contribution constraints worth preserving
- Follow surrounding file style rather than enforcing generic modern Python cleanup. The project explicitly prefers consistency with existing code flow/format (`docs/CONTRIBUTING.md`).
- Avoid mixing whitespace-only edits with functional changes.
- Fix root causes; contribution guidance explicitly expects defect fixes to target the underlying cause.
- New Python/C source files should carry the existing copyright-header style.

## Commit / PR expectations
- Commit subject format is `module: Capitalized, short summary` where `module` is usually a repo file or directory name (`docs/CONTRIBUTING.md`).
- Commits are expected to be single-topic and independently sensible.
- Signed-off-by lines are required by project contribution policy.

## Useful investigation commands
- Whitespace check mentioned in docs: `./scripts/check_whitespace.sh`
- Batch-run Klippy against a config/gcode/dictionary for debugging:
  - `python ./klippy/klippy.py ~/printer.cfg -i test.gcode -o test.serial -v -d out/klipper.dict`
- Convert generated serial output to readable text:
  - `python ./klippy/parsedump.py out/klipper.dict test.serial > test.txt`

## Practical agent guidance
- When changing module loading, config parsing, plugin discovery, or printer startup flow, inspect `klippy/printer.py` first.
- When changing motion/kinematics behavior, read `docs/Code_Overview.md` and relevant files in `klippy/kinematics/` and `klippy/chelper/` before editing.
- When changing firmware build behavior, verify against `scripts/ci-build.sh`, `scripts/Dockerfile-build`, and `test/configs/*.config` instead of guessing from README prose.

## Personal development workspace
- **Personal config examples**: Store in `config/myconfig/` directory.
  - Example: `config/myconfig/alps_serial_example.cfg`
  - These are personal test configs, NOT committed to mainline `config/` directory.
- **Personal dev docs**: Store in `docs/mydocs/` directory.
  - Example: `docs/mydocs/MPC v2 算法技术文档.md`
  - These are personal notes/experiments, NOT part of official `docs/` documentation.
- **Purpose**: Keep experimental work isolated from production code and official documentation.
- **Git tracking**: These directories may be gitignored or kept in separate branches for personal use.
