# MPC v2 算法技术文档

## 模型预测控制算法数学原理详解

---

## 1. 概述

本文档详细阐述 MPC v2（Model Predictive Control version 2）算法的数学实现原理。该算法应用于3D打印机热端温度控制，采用三状态热动力学模型、扩展卡尔曼滤波器（EKF）进行状态估计，以及投影梯度下降法求解约束优化问题。

---

## 2. 热动力学系统建模

### 2.1 状态空间描述

系统采用三状态变量描述热端的热动力学行为：

$$
\mathbf{x} = \begin{bmatrix} T_h \\ T_b \\ T_s \end{bmatrix}
$$

其中：
- $T_h$：加热器芯体温度（°C）
- $T_b$：热块温度（°C）
- $T_s$：传感器测量温度（°C）

### 2.2 热动力学微分方程

系统状态演化由以下微分方程组描述：

#### 2.2.1 加热器温度动态

$$
\frac{dT_h}{dt} = \theta_8 \cdot P - \theta_1 \cdot (T_h - T_b)
$$

其中：
- $P$：输入功率（W）
- $\theta_8$：加热器功率转换系数
- $\theta_1$：加热器与热块间的热传导系数

#### 2.2.2 热块温度动态

$$
\frac{dT_b}{dt} = q_{hb} - q_{bs} - P_{conv} - P_{cond} - P_{rad} - P_{ext}
$$

各项热流功率定义如下：

**热传导项：**

$$
q_{hb} = \theta_2 \cdot (T_h - T_b)
$$

$$
q_{bs} = \theta_3 \cdot (T_b - T_s)
$$

**对流热损耗：**

$$
P_{conv} = \theta_5 \cdot (T_b - T_{env})
$$

**传导热损耗（至冷端）：**

$$
P_{cond} = \theta_6 \cdot (T_b - T_{cold})
$$

**辐射热损耗：**

$$
P_{rad} = \theta_7 \cdot (T_{b,K}^4 - T_{env,K}^4)
$$

其中温度转换为开尔文温标：

$$
T_{b,K} = T_b + 273.15
$$

$$
T_{env,K} = T_{env} + 273.15
$$

**挤出热损耗：**

$$
P_{ext} = v_f \cdot c_p \cdot \theta_9 \cdot (T_b - T_{filament})
$$

其中：
- $v_f$：耗材进给速度（mm³/s）
- $c_p$：耗材比热容（J/(mm³·K)）
- $T_{filament}$：耗材入口温度

#### 2.2.3 传感器温度动态

$$
\frac{dT_s}{dt} = \theta_4 \cdot (T_b - T_s)
$$

其中 $\theta_4$ 为传感器响应系数，表征传感器热惯性的大小。

### 2.3 参数物理意义汇总

| 参数 | 物理意义 | 单位 |
|------|----------|------|
| $\theta_1$ | 加热器→热块热传导系数 | s⁻¹ |
| $\theta_2$ | 热块←加热器热传导系数 | s⁻¹ |
| $\theta_3$ | 热块→传感器热传导系数 | s⁻¹ |
| $\theta_4$ | 传感器响应系数 | s⁻¹ |
| $\theta_5$ | 对流热损耗系数 | s⁻¹ |
| $\theta_6$ | 传导热损耗系数 | s⁻¹ |
| $\theta_7$ | 辐射热损耗系数 | K⁻³·s⁻¹ |
| $\theta_8$ | 加热器功率系数 | K·s⁻¹·W⁻¹ |
| $\theta_9$ | 挤出热损耗系数 | mm⁻³ |

### 2.4 离散化模型

采用前向欧拉法对连续时间模型进行离散化：

$$
\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \cdot f(\mathbf{x}_k, u_k, \mathbf{p})
$$

其中 $\Delta t$ 为采样周期，$u_k = P_k$ 为控制输入，$\mathbf{p}$ 为参数向量。

离散状态更新方程：

$$
T_h^{k+1} = T_h^k + \Delta t \cdot \left[ \theta_8 \cdot P_k - \theta_1 \cdot (T_h^k - T_b^k) \right]
$$

$$
T_b^{k+1} = T_b^k + \Delta t \cdot \left[ \theta_2 \cdot (T_h^k - T_b^k) - \theta_3 \cdot (T_b^k - T_s^k) - \theta_5 \cdot (T_b^k - T_{env}) - \theta_6 \cdot (T_b^k - T_{cold}) - \theta_7 \cdot (T_{b,K}^4 - T_{env,K}^4) - v_f \cdot c_p \cdot \theta_9 \cdot (T_b^k - T_{filament}) \right]
$$

$$
T_s^{k+1} = T_s^k + \Delta t \cdot \theta_4 \cdot (T_b^k - T_s^k)
$$

### 2.5 状态约束

系统状态需满足物理约束：

$$
T_{min} \leq T_h, T_b, T_s \leq T_{max}
$$

其中 $T_{min} = 0°C$，$T_{max} = 500°C$。

---

## 3. 扩展卡尔曼滤波器（EKF）

### 3.1 状态估计问题建模

由于仅能直接测量传感器温度 $T_s$，需通过状态估计技术推断不可观测状态 $T_h$ 和 $T_b$。

**状态向量：**

$$
\hat{\mathbf{x}} = \begin{bmatrix} \hat{T}_h \\ \hat{T}_b \\ \hat{T}_s \end{bmatrix}
$$

**观测向量：**

$$
\mathbf{z} = T_s^{measured}
$$

**观测矩阵：**

$$
\mathbf{H} = \begin{bmatrix} 0 & 0 & 1 \end{bmatrix}
$$

### 3.2 状态转移雅可比矩阵

非线性状态转移函数 $\mathbf{x}_{k+1} = f(\mathbf{x}_k, u_k)$ 的雅可比矩阵：

$$
\mathbf{F} = \frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix}
\frac{\partial T_h^{k+1}}{\partial T_h^k} & \frac{\partial T_h^{k+1}}{\partial T_b^k} & \frac{\partial T_h^{k+1}}{\partial T_s^k} \\[10pt]
\frac{\partial T_b^{k+1}}{\partial T_h^k} & \frac{\partial T_b^{k+1}}{\partial T_b^k} & \frac{\partial T_b^{k+1}}{\partial T_s^k} \\[10pt]
\frac{\partial T_s^{k+1}}{\partial T_h^k} & \frac{\partial T_s^{k+1}}{\partial T_b^k} & \frac{\partial T_s^{k+1}}{\partial T_s^k}
\end{bmatrix}
$$

各元素解析表达式：

$$
F_{11} = 1 - \theta_1 \cdot \Delta t
$$

$$
F_{12} = \theta_1 \cdot \Delta t
$$

$$
F_{13} = 0
$$

$$
F_{21} = \theta_2 \cdot \Delta t
$$

$$
F_{22} = 1 - \left( \theta_3 + \theta_5 + \theta_6 + 4\theta_7 T_{b,K}^3 + v_f \cdot c_p \cdot \theta_9 \right) \cdot \Delta t
$$

$$
F_{23} = -\theta_3 \cdot \Delta t
$$

$$
F_{31} = 0
$$

$$
F_{32} = \theta_4 \cdot \Delta t
$$

$$
F_{33} = 1 - \theta_4 \cdot \Delta t
$$

雅可比矩阵完整形式：

$$
\mathbf{F} = \begin{bmatrix}
1 - \theta_1 \Delta t & \theta_1 \Delta t & 0 \\[8pt]
\theta_2 \Delta t & 1 - (\theta_3 + \theta_5 + \theta_6 + 4\theta_7 T_{b,K}^3 + v_f c_p \theta_9) \Delta t & -\theta_3 \Delta t \\[8pt]
0 & \theta_4 \Delta t & 1 - \theta_4 \Delta t
\end{bmatrix}
$$

### 3.3 EKF预测步骤

**状态预测：**

$$
\hat{\mathbf{x}}_{k|k-1} = f(\hat{\mathbf{x}}_{k-1|k-1}, u_{k-1})
$$

**协方差预测：**

$$
\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^T + \mathbf{Q}
$$

其中 $\mathbf{Q}$ 为过程噪声协方差矩阵（3×3对称正定矩阵）。

### 3.4 EKF更新步骤

**新息（观测残差）：**

$$
\mathbf{y}_k = \mathbf{z}_k - \mathbf{H} \hat{\mathbf{x}}_{k|k-1} = T_s^{measured} - \hat{T}_{s,k|k-1}
$$

**新息协方差：**

$$
\mathbf{S}_k = \mathbf{H} \mathbf{P}_{k|k-1} \mathbf{H}^T + R
$$

其中 $R$ 为观测噪声方差（标量）。

**卡尔曼增益：**

$$
\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}^T \mathbf{S}_k^{-1}
$$

**状态更新：**

$$
\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k
$$

**协方差更新：**

$$
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}
$$

### 3.5 噪声协方差矩阵

**过程噪声协方差矩阵 $\mathbf{Q}$：**

$$
\mathbf{Q} = \begin{bmatrix}
\sigma_h^2 & 0 & 0 \\
0 & \sigma_b^2 & 0 \\
0 & 0 & \sigma_s^2
\end{bmatrix}
$$

**观测噪声方差 $R$：**

$R$ 为传感器测量噪声方差，表征温度传感器的测量不确定性。

---

## 4. 模型预测控制（MPC）优化问题

### 4.1 预测时域与控制时域

- **预测时域（Prediction Horizon）**：$N_p$ 步
- **控制时域（Control Horizon）**：$N_c$ 步，通常 $N_c \leq N_p$

#### 4.1.1 零阶保持策略

控制输入采用零阶保持（Zero-Order Hold, ZOH）策略。在控制时域 $N_c$ 内，优化求解 $N_c$ 个独立控制量；在控制时域之外的预测时域内，控制量保持为最后一个控制值不变：

$$
u_k = u_{N_c-1}, \quad \forall k \geq N_c
$$

**数学形式化描述：**

完整的预测时域控制序列 $\mathbf{u}_{full} \in \mathbb{R}^{N_p}$ 由优化变量 $\mathbf{u} \in \mathbb{R}^{N_c}$ 扩展得到：

$$
u_{full,k} = \begin{cases}
u_k & k = 0, 1, \ldots, N_c-1 \\
u_{N_c-1} & k = N_c, N_c+1, \ldots, N_p-1
\end{cases}
$$

**零阶保持的物理意义：**

1. **减少优化变量**：仅优化 $N_c$ 个变量而非 $N_p$ 个，降低计算复杂度
2. **控制平滑性**：避免预测时域末端控制量的剧烈变化
3. **稳定性保证**：恒定控制输入有助于终端约束的满足

**零阶保持下的控制序列扩展：**

$$
\mathbf{u}_{full} = \begin{bmatrix} u_0 & u_1 & \cdots & u_{N_c-1} & u_{N_c-1} & \cdots & u_{N_c-1} \end{bmatrix}^T \in \mathbb{R}^{N_p}
$$

### 4.2 目标函数

MPC优化问题的目标函数采用归一化形式：

$$
J(\mathbf{u}) = J_{track} + J_{terminal} + J_{rate}
$$

#### 4.2.1 跟踪误差代价

$$
J_{track} = w_t \sum_{k=1}^{N_p} \left( \frac{T_s(k) - T_{set}}{T_{ref}} \right)^2
$$

其中：
- $T_{set}$：目标温度设定值
- $T_{ref} = \max(|T_{set}|, 100)$：归一化参考温度

#### 4.2.2 终端代价

$$
J_{terminal} = w_{terminal} \left( \frac{T_s(N_p) - T_{set}}{T_{ref}} \right)^2
$$

终端代价强化预测时域末端的状态约束。

#### 4.2.3 控制变化率惩罚

$$
J_{rate} = w_r \left( \frac{u(0) - u_{last}}{P_{max}} \right)^2 + w_r \sum_{k=1}^{N_c-1} \left( \frac{u(k) - u(k-1)}{P_{max}} \right)^2
$$

其中：
- $u_{last}$：上一时刻控制量
- $P_{max}$：最大功率（用于归一化）

#### 4.2.4 完整目标函数

$$
J(\mathbf{u}) = w_t \sum_{k=1}^{N_p} \left( \frac{T_s(k) - T_{set}}{T_{ref}} \right)^2 + w_{terminal} \left( \frac{T_s(N_p) - T_{set}}{T_{ref}} \right)^2 + w_r \sum_{k=0}^{N_c-1} \left( \frac{\Delta u(k)}{P_{max}} \right)^2
$$

其中 $\Delta u(0) = u(0) - u_{last}$，$\Delta u(k) = u(k) - u(k-1)$（$k \geq 1$）。

### 4.3 约束条件

#### 4.3.1 控制输入约束

$$
P_{min} \leq u(k) \leq P_{max}, \quad \forall k \in \{0, 1, \ldots, N_c-1\}
$$

通常 $P_{min} = 0$，$P_{max}$ 为加热器最大功率。

#### 4.3.2 状态约束

$$
T_{min} \leq T_h(k), T_b(k), T_s(k) \leq T_{max}
$$

### 4.4 优化问题形式化

完整的MPC优化问题可表述为：

$$
\min_{\mathbf{u} \in \mathbb{R}^{N_c}} J(\mathbf{u})
$$

$$
\text{s.t.} \quad \mathbf{x}(k+1) = f(\mathbf{x}(k), u(k)), \quad k = 0, 1, \ldots, N_p-1
$$

$$
\mathbf{x}(0) = \hat{\mathbf{x}}_{current}
$$

$$
u(k) = u(N_c-1), \quad k \geq N_c
$$

$$
P_{min} \leq u(k) \leq P_{max}
$$

---

## 5. 投影梯度下降法求解器

### 5.1 算法框架

针对带约束的非线性优化问题，采用投影梯度下降法（Projected Gradient Descent, PGD）求解。

### 5.2 梯度计算

采用有限差分法近似计算目标函数关于控制序列的梯度：

$$
\frac{\partial J}{\partial u_i} \approx \frac{J(\mathbf{u} + \epsilon \mathbf{e}_i) - J(\mathbf{u})}{\epsilon}
$$

其中：
- $\mathbf{e}_i$：第 $i$ 个单位向量
- $\epsilon$：有限差分步长（通常取 $10^{-4}$）

### 5.3 投影操作

将控制序列投影到可行域：

$$
\text{Proj}_{\mathcal{U}}(\mathbf{u}) = \arg\min_{\mathbf{v} \in \mathcal{U}} \|\mathbf{v} - \mathbf{u}\|^2
$$

其中可行域 $\mathcal{U} = \{\mathbf{u} \in \mathbb{R}^{N_c} : P_{min} \leq u_i \leq P_{max}, \forall i\}$。

投影操作的解析解：

$$
u_i^{proj} = \max(P_{min}, \min(P_{max}, u_i))
$$

### 5.4 回溯线搜索

采用Armijo条件确定步长 $\alpha$：

**Armijo条件：**

$$
J(\mathbf{u} - \alpha \nabla J) \leq J(\mathbf{u}) - c_1 \alpha \|\nabla J\|^2
$$

其中 $c_1 \in (0, 1)$ 为Armijo参数（通常取 $10^{-4}$）。

**回溯策略：**

1. 初始化步长 $\alpha = \alpha_0$（通常 $\alpha_0 = 1.0$）
2. 若不满足Armijo条件，则 $\alpha \leftarrow \rho \cdot \alpha$（$\rho \in (0, 1)$，通常取 $0.5$）
3. 重复直至满足条件或达到最大迭代次数

### 5.5 算法流程

**输入：** 初始状态 $\mathbf{x}_0$，目标温度 $T_{set}$，初始控制序列 $\mathbf{u}^0$

**输出：** 最优控制序列 $\mathbf{u}^*$

**算法步骤：**

1. **初始化：** $\mathbf{u} \leftarrow \mathbf{u}^0$，计算初始目标函数值 $J_0 = J(\mathbf{u})$，记录最优解 $\mathbf{u}^* = \mathbf{u}$，$J^* = J_0$

2. **迭代循环**（$k = 0, 1, \ldots, k_{max}$）：

   a. 计算梯度 $\nabla J(\mathbf{u})$
   
   b. 检查收敛条件：若 $\|\nabla J(\mathbf{u})\| < \epsilon_{tol}$，则收敛，退出循环
   
   c. 回溯线搜索确定步长 $\alpha$
   
   d. 梯度下降：$\mathbf{u} \leftarrow \mathbf{u} - \alpha \nabla J(\mathbf{u})$
   
   e. 投影到可行域：$\mathbf{u} \leftarrow \text{Proj}_{\mathcal{U}}(\mathbf{u})$
   
   f. 计算新目标函数值 $J_{new} = J(\mathbf{u})$
   
   g. 更新最优解：若 $J_{new} < J^*$，则 $\mathbf{u}^* \leftarrow \mathbf{u}$，$J^* \leftarrow J_{new}$
   
   h. 检查改进量：若 $J^* - J_{new} < \epsilon_{tol}$，则收敛，退出循环

3. **返回：** $\mathbf{u}^*$

### 5.6 收敛性分析

投影梯度下降法在凸优化问题中具有全局收敛性。对于非凸问题（如本MPC问题），算法可收敛到局部最优解。

**收敛条件：**

- 目标函数 $J(\mathbf{u})$ 在可行域上连续可微
- 梯度 $\nabla J(\mathbf{u})$ 满足Lipschitz连续条件
- 步长 $\alpha$ 满足Armijo条件

**收敛速率：**

对于强凸目标函数，线性收敛：

$$
J(\mathbf{u}^k) - J(\mathbf{u}^*) \leq (1 - \mu/L)^k (J(\mathbf{u}^0) - J(\mathbf{u}^*))
$$

其中 $\mu$ 为强凸参数，$L$ 为Lipschitz常数。

---

## 6. 系统整体架构

### 6.1 控制循环

在每个控制周期内，系统执行以下步骤：

1. **状态估计：** EKF融合传感器测量值，估计完整状态向量 $\hat{\mathbf{x}} = [\hat{T}_h, \hat{T}_b, \hat{T}_s]^T$

2. **参数更新：** 根据当前工况更新环境参数（$T_{env}$, $T_{cold}$, $v_f$, $T_{filament}$）

3. **MPC优化：** 基于当前状态估计，求解优化问题得到最优控制序列 $\mathbf{u}^*$

4. **控制执行：** 应用控制序列的第一个元素 $u^*(0)$ 作为当前时刻的控制输入

5. **滚动时域：** 下一时刻重复上述过程

### 6.2 模块间数学关系

```
┌─────────────────────────────────────────────────────────────────┐
│                        MPC v2 控制系统                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌───────────────┐      ┌───────────────┐      ┌─────────────┐ │
│  │   传感器测量   │ ──→  │     EKF       │ ──→  │  状态估计   │ │
│  │   T_s^{meas}  │      │   状态估计器   │      │  x̂ = [T̂_h  │ │
│  └───────────────┘      └───────────────┘      │     T̂_b     │ │
│                                ↑                │     T̂_s]    │ │
│                                │                └─────────────┘ │
│                         ┌──────┴──────┐                │        │
│                         │  过程噪声Q   │                ↓        │
│                         │  观测噪声R   │      ┌───────────────┐  │
│                         └─────────────┘      │   MPC求解器   │  │
│                                              │               │  │
│  ┌───────────────┐                          │  ┌─────────┐  │  │
│  │   热动力学模型 │ ───────────────────────→ │  │目标函数 │  │  │
│  │   f(x, u, p)  │                          │  │  J(u)   │  │  │
│  └───────────────┘                          │  └─────────┘  │  │
│         │                                   │       ↓       │  │
│         │                                   │  ┌─────────┐  │  │
│         ↓                                   │  │约束条件 │  │  │
│  ┌───────────────┐                          │  │P_min≤u≤P_max│ │
│  │   模型参数θ   │ ───────────────────────→ │  └─────────┘  │  │
│  │   θ_1...θ_9  │                          │       ↓       │  │
│  └───────────────┘                          │  ┌─────────┐  │  │
│                                              │  │PGD求解  │  │  │
│                                              │  └─────────┘  │  │
│                                              └───────┬───────┘  │
│                                                      │          │
│                                                      ↓          │
│                                              ┌───────────────┐  │
│                                              │ 最优控制序列  │  │
│                                              │   u^*(0)     │  │
│                                              └───────────────┘  │
│                                                      │          │
└──────────────────────────────────────────────────────┼──────────┘
                                                       │
                                                       ↓
                                              ┌───────────────┐
                                              │   加热器执行   │
                                              │   P = u^*(0)  │
                                              └───────────────┘
```


---

## 7. 计算优化策略

### 7.1 内存预分配原理

在实时控制系统中，动态内存分配会引入不可预测的延迟。MPC v2 采用内存预分配策略，在初始化阶段一次性分配所有必需的数组空间，避免在控制循环中进行内存分配操作。

### 7.2 预分配数据结构

#### 7.2.1 控制序列存储

$$
\mathbf{u}_{cache} \in \mathbb{R}^{N_c}, \quad \mathbf{u}_{prev} \in \mathbb{R}^{N_c}, \quad \mathbf{u}_{power} \in \mathbb{R}^{N_p}
$$

其中：
- $\mathbf{u}_{cache}$：当前控制序列缓存
- $\mathbf{u}_{prev}$：上一时刻优化结果
- $\mathbf{u}_{power}$：扩展至预测时域的功率序列

#### 7.2.2 状态轨迹存储

$$
\mathbf{T}_h^{traj} \in \mathbb{R}^{N_p+1}, \quad \mathbf{T}_b^{traj} \in \mathbb{R}^{N_p+1}, \quad \mathbf{T}_s^{traj} \in \mathbb{R}^{N_p+1}
$$

状态轨迹数组存储预测时域内各时刻的状态值，数组长度为 $N_p+1$（包含初始状态）。

#### 7.2.3 梯度存储

$$
\mathbf{g}_{cache} \in \mathbb{R}^{N_c}
$$

梯度缓存用于存储目标函数关于控制序列的梯度向量。

### 7.3 热启动优化

#### 7.3.1 热启动原理

MPC 的滚动时域特性使得相邻时刻的优化问题具有相似性。利用上一时刻的优化结果作为当前时刻的初始解，可显著减少迭代次数。

**热启动策略：**

$$
\mathbf{u}^{(k)}_{init} = \mathcal{S}(\mathbf{u}^{(k-1)*})
$$

其中 $\mathcal{S}(\cdot)$ 为移位算子：

$$
\mathcal{S}(\mathbf{u})_i = \begin{cases}
u_{i+1} & i = 0, 1, \ldots, N_c-2 \\
u_{N_c-1} & i = N_c-1
\end{cases}
$$

#### 7.3.2 热启动条件

热启动标志 $\phi_{hot} \in \{0, 1\}$ 指示是否可用热启动：

$$
\phi_{hot}^{(k)} = \begin{cases}
1 & \text{若 } k > 0 \text{ 且上一时刻优化收敛} \\
0 & \text{否则}
\end{cases}
$$

#### 7.3.3 初始解选择

$$
\mathbf{u}^{(k)}_{init} = \begin{cases}
\mathcal{S}(\mathbf{u}^{(k-1)*}) & \phi_{hot}^{(k)} = 1 \\
\mathbf{u}_{heuristic} & \phi_{hot}^{(k)} = 0
\end{cases}
$$

启发式初始解 $\mathbf{u}_{heuristic}$ 根据当前温度偏差确定：

$$
\mathbf{u}_{heuristic} = \begin{cases}
0.8 \cdot P_{max} \cdot \mathbf{1} & T_{set} - T_s > \Delta T_{large} \\
0.5 \cdot P_{max} \cdot \mathbf{1} & T_{set} - T_s > \Delta T_{medium} \\
0.3 \cdot P_{max} \cdot \mathbf{1} & T_{set} - T_s > \Delta T_{small} \\
\max(u_{last}, 0.1 \cdot P_{max}) \cdot \mathbf{1} & \text{其他}
\end{cases}
$$

其中 $\mathbf{1}$ 为全1向量。

### 7.4 内存预分配的数学意义

#### 7.4.1 时间复杂度分析

**无预分配情况：**

每次控制周期需执行 $O(N_c + N_p)$ 次内存分配操作，引入额外时间开销。

**预分配情况：**

初始化时执行 $O(N_c + N_p)$ 次内存分配，控制周期内仅执行数组访问操作，时间复杂度降为 $O(1)$。

#### 7.4.2 空间复杂度

总预分配空间：

$$
S_{total} = 2N_c + N_p + 3(N_p+1) + N_c = 3N_c + 4N_p + 3
$$

以典型参数 $N_c = 10$，$N_p = 30$ 为例：

$$
S_{total} = 3 \times 10 + 4 \times 30 + 3 = 153 \text{ 个浮点数}
$$

内存占用约 $153 \times 8 = 1224$ 字节（双精度浮点数）。

### 7.5 数据复用策略

#### 7.5.1 轨迹预测复用

在梯度计算过程中，多次调用轨迹预测函数。预分配的轨迹数组避免了重复内存分配：

$$
\mathbf{T}_s^{traj} = [\hat{T}_s(0), \hat{T}_s(1), \ldots, \hat{T}_s(N_p)]
$$

#### 7.5.2 梯度缓存复用

梯度向量在迭代过程中被多次计算和访问：

$$
\nabla J = \left[ \frac{\partial J}{\partial u_0}, \frac{\partial J}{\partial u_1}, \ldots, \frac{\partial J}{\partial u_{N_c-1}} \right]^T
$$

---

## 8. 数学符号汇总

| 符号 | 含义 | 单位 |
|------|------|------|
| $T_h$ | 加热器温度 | °C |
| $T_b$ | 热块温度 | °C |
| $T_s$ | 传感器温度 | °C |
| $T_{env}$ | 环境温度 | °C |
| $T_{cold}$ | 冷端温度 | °C |
| $T_{filament}$ | 耗材入口温度 | °C |
| $T_{set}$ | 目标设定温度 | °C |
| $P$ | 加热功率 | W |
| $v_f$ | 耗材进给速度 | mm³/s |
| $c_p$ | 耗材比热容 | J/(mm³·K) |
| $\theta_i$ | 模型参数（$i=1,\ldots,9$） | 各异 |
| $N_p$ | 预测时域 | 步 |
| $N_c$ | 控制时域 | 步 |
| $w_t$ | 跟踪误差权重 | - |
| $w_{terminal}$ | 终端代价权重 | - |
| $w_r$ | 控制变化率权重 | - |
| $\mathbf{Q}$ | 过程噪声协方差矩阵 | (°C)² |
| $R$ | 观测噪声方差 | (°C)² |
| $\mathbf{P}$ | 状态估计协方差矩阵 | (°C)² |
| $\mathbf{F}$ | 状态转移雅可比矩阵 | - |
| $\mathbf{H}$ | 观测矩阵 | - |
| $\mathbf{K}$ | 卡尔曼增益 | - |
| $\alpha$ | 梯度下降步长 | - |
| $\epsilon$ | 有限差分步长 | W |
| $\mathbf{u}_{cache}$ | 控制序列缓存 | W |
| $\mathbf{u}_{prev}$ | 上一次优化结果 | W |
| $\mathbf{u}_{full}$ | 零阶保持扩展后的完整控制序列 | W |
| $\mathbf{T}_h^{traj}$ | 加热器温度轨迹 | °C |
| $\mathbf{T}_b^{traj}$ | 热块温度轨迹 | °C |
| $\mathbf{T}_s^{traj}$ | 传感器温度轨迹 | °C |
| $\mathbf{g}_{cache}$ | 梯度缓存 | W⁻¹ |
| $\mathcal{S}(\cdot)$ | 移位算子 | - |
| $\phi_{hot}$ | 热启动标志 | - |

---

## 9. 理论分析

### 9.1 系统可观性分析

由于观测矩阵 $\mathbf{H} = [0, 0, 1]$，仅观测传感器温度 $T_s$。系统的可观性取决于可观性矩阵：

$$
\mathcal{O} = \begin{bmatrix}
\mathbf{H} \\
\mathbf{H}\mathbf{F} \\
\mathbf{H}\mathbf{F}^2
\end{bmatrix}
$$

当 $\text{rank}(\mathcal{O}) = 3$ 时，系统完全可观。对于本热动力学模型，在参数 $\theta_1, \theta_2, \theta_3, \theta_4$ 非零的条件下，系统满足可观性条件。

### 9.2 系统稳定性

**开环稳定性：**

热动力学系统为耗散系统，在无外部输入时，温度将趋于环境温度 $T_{env}$。

**闭环稳定性：**

MPC控制器通过优化目标函数隐式保证闭环稳定性。终端代价 $J_{terminal}$ 的引入强化了稳定性保证。

### 9.3 鲁棒性分析

EKF的状态估计对传感器噪声和模型不确定性具有鲁棒性。协方差矩阵 $\mathbf{P}$ 的演化反映了估计的不确定性：

- 较大的 $\mathbf{P}$ 值表示状态估计不确定性高
- 卡尔曼增益 $\mathbf{K}$ 自动平衡预测与观测的权重

---

## 10. 结论

MPC v2算法通过以下数学框架实现精确的温度控制：

1. **三状态热动力学模型**：精确描述加热器、热块和传感器之间的热传递过程，包含传导、对流、辐射和挤出热损耗

2. **扩展卡尔曼滤波器**：从单一温度测量推断完整系统状态，提供最优状态估计及其不确定性量化

3. **模型预测控制**：通过滚动时域优化，在满足约束条件下最小化跟踪误差和控制变化率

4. **投影梯度下降法**：高效求解带约束的非线性优化问题，保证解的可行性和收敛性

5. **内存预分配与热启动**：通过预分配数据结构和热启动策略，显著降低计算延迟，满足实时控制需求

该算法的数学严谨性确保了温度控制的精确性、稳定性和鲁棒性，适用于3D打印机热端等需要精确温度控制的工业应用场景。

---

*文档版本：1.1*  
*适用算法版本：MPC v2*
