# 投影梯度下降法求解MPC问题

## 1. 问题定义

### 1.1 MPC优化问题

模型预测控制(MPC)的核心是在每个控制周期求解以下优化问题：

$$
\min_{u_0, u_1, \ldots, u_{N_c-1}} J = \sum_{k=1}^{N_p} w_t (T_s(k) - T_{ref})^2 + \sum_{k=0}^{N_p-1} w_c u_k^2 + \sum_{k=0}^{N_c-2} w_r (u_{k+1} - u_k)^2
$$

**约束条件：**

$$
u_{min} \leq u_k \leq u_{max}, \quad k = 0, 1, \ldots, N_c - 1
$$

**零阶保持策略：**

$$
u_k = u_{N_c-1}, \quad k = N_c, N_c+1, \ldots, N_p - 1
$$

### 1.2 变量定义

| 符号 | 含义 | 单位 |
|------|------|------|
| $N_p$ | 预测时域 (Prediction Horizon) | 步 |
| $N_c$ | 控制时域 (Control Horizon) | 步 |
| $u_k$ | 第 $k$ 步的控制量（功率） | W |
| $T_s(k)$ | 第 $k$ 步的传感器温度预测值 | °C |
| $T_{ref}$ | 目标温度 | °C |
| $w_t$ | 跟踪误差权重 | - |
| $w_c$ | 控制量权重 | - |
| $w_r$ | 控制变化率权重 | - |
| $u_{min}, u_{max}$ | 控制量上下界 | W |

---

## 2. 三节点热力学模型

### 2.1 模型结构

```
[加热器 T_h] --θ₁--> [加热块 T_b] --θ₃--> [传感器 T_s]
     |                   |                    |
   θ₈·P               θ₂·ΔT                θ₄·ΔT
     |                   |                    |
   热源              散热损耗              温度测量
```

### 2.2 状态方程（欧拉法离散化）

**加热器温度：**

$$
T_h(k+1) = T_h(k) + \Delta t \cdot [\theta_8 \cdot P(k) - \theta_1 \cdot (T_h(k) - T_b(k))]
$$

**加热块温度：**

$$
T_b(k+1) = T_b(k) + \Delta t \cdot [q_{hb} - q_{bs} - P_{conv} - P_{cond} - P_{rad} - P_{ext}]
$$

其中：

$$
\begin{aligned}
q_{hb} &= \theta_2 \cdot (T_h - T_b) & \text{(加热器到加热块传热)} \\
q_{bs} &= \theta_3 \cdot (T_b - T_s) & \text{(加热块到传感器传热)} \\
P_{conv} &= \theta_5 \cdot (T_b - T_{env}) & \text{(对流散热)} \\
P_{cond} &= \theta_6 \cdot (T_b - T_{cold}) & \text{(传导散热)} \\
P_{rad} &= \theta_7 \cdot (T_b^4 - T_{env}^4) & \text{(辐射散热)} \\
P_{ext} &= v_f \cdot c_p \cdot \theta_9 \cdot (T_b - T_{filament}) & \text{(挤出散热)}
\end{aligned}
$$

**传感器温度：**

$$
T_s(k+1) = T_s(k) + \Delta t \cdot \theta_4 \cdot (T_b(k) - T_s(k))
$$

### 2.3 模型参数

| 参数 | 物理含义 | 典型值 |
|------|----------|--------|
| $\theta_1$ | 加热器到加热块热传导系数 | 5.03e-2 1/s |
| $\theta_2$ | 加热块从加热器获热系数 | 2.81e-1 1/s |
| $\theta_3$ | 加热块到传感器热传导系数 | 1.07e-2 1/s |
| $\theta_4$ | 传感器响应系数 | 1.37e-1 1/s |
| $\theta_5$ | 对流换热系数 | 3.20e-3 W/K |
| $\theta_6$ | 传导散热系数 | 2.33e-2 W/K |
| $\theta_7$ | 辐射散热系数 | 2.57e-11 W/K⁴ |
| $\theta_8$ | 加热器功率转换系数 | 8.00e-2 K/(W·s) |
| $\theta_9$ | 挤出散热系数 | $\theta_8 \cdot \theta_2 / \theta_1$ |

---

## 3. 投影梯度下降算法

### 3.1 算法原理

投影梯度下降法是求解带约束优化问题的有效方法，其核心思想是：

1. **梯度下降**：沿负梯度方向更新解
2. **投影操作**：将解投影回可行域

### 3.2 梯度计算

采用**有限差分法**近似计算目标函数关于控制序列的梯度：

$$
\frac{\partial J}{\partial u_i} \approx \frac{J(u + \epsilon e_i) - J(u)}{\epsilon}
$$

其中：
- $\epsilon$ 为差分步长，通常取 $10^{-4}$
- $e_i$ 为第 $i$ 个单位向量

**梯度向量：**

$$
\nabla J = \left[ \frac{\partial J}{\partial u_0}, \frac{\partial J}{\partial u_1}, \ldots, \frac{\partial J}{\partial u_{N_c-1}} \right]^T
$$

### 3.3 Armijo线搜索

为保证每次迭代目标函数下降，采用Armijo条件确定步长：

**Armijo条件：**

$$
J(u - \alpha \nabla J) \leq J(u) - c_1 \alpha \|\nabla J\|^2
$$

其中：
- $\alpha$ 为步长
- $c_1$ 为Armijo参数，通常取 $10^{-4}$
- $\|\nabla J\|$ 为梯度范数

**回溯策略：**

$$
\alpha_{k+1} = \rho \cdot \alpha_k, \quad \rho \in (0, 1)
$$

通常取 $\rho = 0.5$，初始步长 $\alpha_0 = 1.0$。

### 3.4 投影操作

将控制序列投影到可行域 $[u_{min}, u_{max}]$：

$$
\text{Proj}_{[u_{min}, u_{max}]}(u_i) = \begin{cases}
u_{min} & \text{if } u_i < u_{min} \\
u_i & \text{if } u_{min} \leq u_i \leq u_{max} \\
u_{max} & \text{if } u_i > u_{max}
\end{cases}
$$

---

## 4. 改进算法流程

### 4.1 算法伪代码

```
算法: 改进的投影梯度下降法求解MPC

输入: 初始状态 (T_h, T_b, T_s), 目标温度 T_ref, 初始控制序列 u_init
输出: 最优控制序列 best_u, 收敛标志 converged, 迭代次数 iterations

1. 初始化:
   u = u_init
   best_cost = J(u)
   best_u = u
   converged = False
   iterations = 0

2. 主循环 (for it = 1 to max_iter):
   
   2.1 计算梯度:
       grad = ∇J(u)
   
   2.2 检查梯度收敛:
       if ||grad|| < tol:
           converged = True
           break
   
   2.3 Armijo线搜索:
       alpha = line_search(u, grad)
   
   2.4 更新控制序列:
       u = u - alpha * grad
   
   2.5 投影到可行域:
       u = Proj(u, [u_min, u_max])
   
   2.6 最优解追踪:
       cost = J(u)
       if cost < best_cost:
           improvement = best_cost - cost
           best_cost = cost
           best_u = u
           
           if improvement < tol:
               converged = True
               break
   
   2.7 iterations = it

3. 返回 (best_u, converged, iterations)
```

### 4.2 关键改进点

| 改进 | 原算法 | 改进后 |
|------|--------|--------|
| **最优解追踪** | 返回最后迭代解 | 返回历史最优解 |
| **收敛条件** | 仅梯度范数 | 梯度范数 + 目标函数改进 |
| **稳定性** | 可能震荡 | 保证返回最优解 |

---

## 5. 收敛性分析

### 5.1 收敛条件

**条件1：梯度范数收敛**

$$
\|\nabla J\| < \epsilon_{grad}
$$

表示当前点接近局部最优解。

**条件2：目标函数改进收敛**

$$
J_{best}^{old} - J_{best}^{new} < \epsilon_{cost}
$$

表示目标函数改进已足够小。

### 5.2 收敛性保证

在以下条件下，算法保证收敛：

1. **目标函数凸性**：目标函数在可行域内为凸函数
2. **Lipschitz连续梯度**：存在 $L > 0$ 使得 $\|\nabla J(x) - \nabla J(y)\| \leq L\|x - y\|$
3. **步长条件**：$\alpha \leq 1/L$

**收敛速率：**

对于凸目标函数，投影梯度下降法具有 $O(1/k)$ 的收敛速率：

$$
J(u_k) - J(u^*) \leq \frac{\|u_0 - u^*\|^2}{2k\alpha}
$$

---

## 6. 热启动策略

### 6.1 热启动原理

利用MPC的时移特性，将上一时刻的解作为当前时刻的初始猜测：

$$
u_{init}^{(k)} = [u_1^{(k-1)}, u_2^{(k-1)}, \ldots, u_{N_c-1}^{(k-1)}, u_{N_c-1}^{(k-1)}]
$$

### 6.2 热启动优势

| 指标 | 冷启动 | 热启动 |
|------|--------|--------|
| 初始解质量 | 较差 | 接近最优 |
| 迭代次数 | 多 (~100) | 少 (~10) |
| 计算时间 | 长 (~18ms) | 短 (~1ms) |
| 收敛稳定性 | 可能未收敛 | 通常收敛 |

### 6.3 热启动效果

```
周期 1: 冷启动 (~18ms) → 得到初始解
周期 2: 热启动 (~1ms)  → 从上次解继续优化
周期 3: 热启动 (~1ms)  → ...
周期 N: 热启动 (~1ms)  → ...
```

---

## 7. Numba加速实现

### 7.1 加速策略

| 技术 | 说明 | 加速比 |
|------|------|--------|
| `@jit(nopython=True)` | 编译为机器码 | 10-100x |
| `cache=True` | 缓存编译结果 | 避免重复编译 |
| `fastmath=True` | 放宽浮点精度 | 1.5-2x |
| `parallel=True` | 并行计算 | 多核加速 |

### 7.2 性能基准

| 函数 | 耗时 |
|------|------|
| 单步模型预测 | 0.0005 ms |
| MPC目标函数 | 0.0014 ms |
| EKF状态更新 | 0.0044 ms |
| MPC求解(冷启动) | 18.28 ms |
| MPC求解(热启动) | 0.76 ms |

---

## 8. 应用场景

### 8.1 适用场景

- **3D打印机温度控制**：挤出头、热床温度精确控制
- **实时嵌入式系统**：计算资源受限的实时控制
- **多变量控制**：可扩展到多输入多输出系统

### 8.2 参数调优建议

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| $N_p$ | 20-30 | 预测时域，越大越精确但计算量增加 |
| $N_c$ | 5-10 | 控制时域，通常为 $N_p$ 的 1/3 |
| $w_t$ | 10.0 | 跟踪权重，越大跟踪越紧 |
| $w_c$ | 0.001 | 控制权重，越大功率越保守 |
| $w_r$ | 0.1 | 变化率权重，越大功率变化越平滑 |
| `tol` | 1e-3 | 收敛容差，与梯度精度匹配 |
| `max_iter` | 100 | 最大迭代次数 |

---

## 9. 参考文献

1. Boyd, S., & Vandenberghe, L. (2004). *Convex Optimization*. Cambridge University Press.

2. Nocedal, J., & Wright, S. J. (2006). *Numerical Optimization*. Springer.

3. Camacho, E. F., & Bordons, C. (2007). *Model Predictive Control*. Springer.

4. Rawlings, J. B., & Mayne, D. Q. (2017). *Model Predictive Control: Theory and Design*. Nob Hill Publishing.

---

## 附录A: 目标函数展开

### A.1 跟踪误差项

$$
J_{track} = w_t \sum_{k=1}^{N_p} (T_s(k) - T_{ref})^2
$$

### A.2 控制量惩罚项

$$
J_{control} = w_c \sum_{k=0}^{N_p-1} u_k^2
$$

### A.3 变化率惩罚项

$$
J_{rate} = w_r \sum_{k=0}^{N_c-2} (u_{k+1} - u_k)^2 + w_r (u_0 - u_{last})^2
$$

### A.4 总目标函数

$$
J = J_{track} + J_{control} + J_{rate}
$$

---

## 附录B: 代码实现

详见 `control_mpc.py` 文件中的以下函数：

- `_numba_model_step()` - 单步模型预测
- `_numba_mpc_objective()` - MPC目标函数
- `_compute_gradient()` - 梯度计算
- `_line_search()` - Armijo线搜索
- `_project_to_bounds()` - 投影操作
- `_solve_mpc_pgd()` - 完整求解器
