# 基于双环 PID 与 LQR 的车辆轨迹跟踪仿真

本项目构建了一个“纵向双环 PID + 横向 LQR”的闭环控制仿真系统。系统以圆弧参考轨迹为目标，完成了轨迹匹配、纵向速度控制、横向转向控制、车辆状态更新与结果可视化。横向部分基于动力学自行车模型建立 LQR 控制问题，通过离散代数 Riccati 方程（DARE）迭代求解最优反馈增益；纵向部分采用位置外环与速度内环的双环 PID 结构。最终效果可实现在存在初始横向偏差与航向偏差条件下，车辆稳定收敛至参考轨迹并在终点停车，同时生成完整仿真动画。

## 1. 系统总体方案

### 1.1 总体结构

控制主循环执行流程：

1. 车辆状态与参考轨迹匹配，获得 Frenet 误差 $(s,d)$；
2. 纵向控制器计算加速度命令 $a_{cmd}$；
3. 横向 LQR 计算转角命令 $\delta$；
4. 基于车辆运动学更新状态；
5. 渲染并保存仿真帧；
6. 结束后生成 GIF。

### 1.2 代码模块对应

- `main.cpp`：仿真流程与参数装配
- `trajectory.cc/.h`：轨迹生成、按弧长插值、匹配
- `pid_controller.cc/.h`：PID 算法实现
- `lqr_controller.cc/.h`：LQR 建模与求解
- `visualizer.cc/.h`：绘图渲染
- `scripts/make_gif.py`：GIF 合成

## 2. 编译和运行

### 2.1 依赖

- C++17, CMake >= 3.14
- Python3 + Pillow
- Eigen3

### 2.2 编译

```bash
cmake -S . -B build
cmake --build build -j
```

### 2.3 运行

```bash
./build/sim_demo
# 或无图形界面
SIM_HEADLESS=1 ./build/sim_demo
```

### 2.4 输出

- 帧图：`output/frames/frame_XXXX.png`
- GIF：`output/demo.gif`

## 3. 数学模型与控制方法

### 3.1 轨迹表示与匹配

参考轨迹离散点包含：

$$
\{x, y, \theta, \kappa, s, v, a, t\}
$$

匹配阶段在历史索引附近窗口搜索最近点，并做切向投影：

$$
s_{match}=\mathrm{clip}(s_i + \Delta x\cos\theta_i + \Delta y\sin\theta_i)
$$

$$
d = \cos\theta_i\,\Delta y - \sin\theta_i\,\Delta x
$$

再通过 `InterpolateByS` 得到连续参考点，减小离散抖动。

### 3.2 纵向控制：双环 PID

#### 3.2.1 控制律

外环（位置环）：

$$
e_s = s_{ref} - s, \quad u_s = PID_s(e_s)
$$

内环（速度环）：

$$
e_v = v_{ref} - v, \quad u_v = \mathrm{clip}(u_s + e_v, -3, 3)
$$

$$
a_{cmd} = PID_v(u_v) + a_{ref}
$$

并施加加速度约束：

$$
a_{cmd} \in [-4.0, 2.5]
$$

#### 3.2.2 PID 离散实现

$$
u(k)=K_p e(k)+I(k)+K_d\frac{e(k)-e(k-1)}{\Delta t}
$$

$$
I(k)=\mathrm{sat}\big(I(k-1)+K_i e(k)\Delta t\big)
$$

其中首个采样点不计算微分项以避免尖峰。

#### 3.2.3 终点速度约束

为保证末端收敛与停车质量，设置速度包络：

$$
v_{ref} \le \sqrt{2a_b s_{remain}}
$$

$$
v_{ref} \le k_d d_{end}
$$

且当 $s_{remain}\le 1.0\,m$ 时强制 $v_{ref}=0$。

### 3.3 横向控制：LQR（动力学自行车模型）

#### 3.3.1 状态变量与输入

定义状态向量：

$$
x=[e_y,\dot e_y,e_\psi,\dot e_\psi]^T
$$

控制输入：前轮转角 $u=\delta$。

#### 3.3.2 连续时间状态空间方程

采用小角度线性化动力学自行车模型：

$$
\dot x = A(v)x + B\delta
$$

$$
A(v)=
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\frac{C_f+C_r}{mv} & \frac{C_f+C_r}{m} & \frac{L_rC_r-L_fC_f}{mv} \\
0 & 0 & 0 & 1 \\
0 & \frac{L_rC_r-L_fC_f}{I_zv} & \frac{L_fC_f-L_rC_r}{I_z} & -\frac{L_f^2C_f+L_r^2C_r}{I_zv}
\end{bmatrix}
$$

$$
B=
\begin{bmatrix}
0 \\
\frac{C_f}{m} \\
0 \\
\frac{L_fC_f}{I_z}
\end{bmatrix}
$$

参数含义：

- $m$：整车质量
- $L_f, L_r$：质心到前后轴距离
- $I_z$：横摆转动惯量
- $C_f, C_r$：前后轮侧偏刚度

为避免低速分母奇异，工程实现采用：

$$
v_{lqr}=\max(v, v_{min}), \quad v_{min}=1.0\,m/s
$$

#### 3.3.3 离散化

采样周期 $T_s=0.01\,s$，采用 Tustin 变换：

$$
A_d=(I-\tfrac{T_s}{2}A)^{-1}(I+\tfrac{T_s}{2}A)
$$

$$
B_d \approx BT_s
$$

#### 3.3.4 代价函数

$$
J=\sum_{k=0}^{\infty}(x_k^TQx_k + u_k^TRu_k)
$$

其中

$$
Q=\mathrm{diag}(q_{11},q_{22},q_{33},q_{44}), \quad R=[r]
$$

本项目参数：

$$
Q=\mathrm{diag}(0.01,0,0.05,0), \quad R=0.05
$$

#### 3.3.5 Riccati 方程求解

离散代数 Riccati 方程（DARE）：

$$
P=A_d^TPA_d-A_d^TPB_d(R+B_d^TPB_d)^{-1}B_d^TPA_d+Q
$$

迭代流程：

1. 初始化 $P_0=Q$
2. 按 DARE 更新 $P_{k+1}$
3. 若 $\|P_{k+1}-P_k\|_\infty<\epsilon$ 则收敛，否则继续
4. 达最大迭代步数仍未满足阈值时给出告警

最优反馈增益：

$$
K=(R+B_d^TPB_d)^{-1}B_d^TPA_d
$$

反馈控制项：

$$
\delta_{fb}=-Kx
$$

#### 3.3.6 曲率前馈与限幅

为减小曲线段稳态偏差，采用

$$
\delta = \delta_{fb}+\delta_{ff}
$$

并施加转角约束：

$$
\delta \in [-0.5,0.5]\;\text{rad}
$$

## 4. 仿真设置

### 4.1 场景与初值

- 参考轨迹：半径约 32m，长度约 3/8 圆
- 控制周期：$T_s=0.01\,s$
- 初始偏差：横向偏移约 3m，航向偏差约 -0.12rad

### 4.2 关键参数

- 纵向外环 PID：`Kp=15, Ki=0.2, Kd=0.01`（积分关闭）
- 纵向内环 PID：`Kp=1.5, Ki=0.5, Kd=0.0`（积分开启）
- LQR：`Q=diag(0.01,0,0.05,0), R=0.05`
- 转角限幅：`±0.5 rad`
- 加速度限幅：`[-4.0, 2.5] m/s^2`

## 5. 调参方法与经验总结

### 5.1 双环 PID 调参流程（先内后外）

#### 阶段 A：速度内环

目标：速度响应快、超调小、无持续振荡。

- 先调 `Kp`：增大到接近振荡边缘后回退 20%~30%
- 再调 `Ki`：消除稳态误差
- 最后视需要加 `Kd`：抑制超调（噪声敏感时不宜过大）

#### 阶段 B：位置外环

目标：消除位置偏差且不破坏内环稳定性。

- 先调 `Kp` 增强收敛速度
- 小步增加 `Ki` 去除残余偏差
- 用 `Kd` 缓解外环激进导致的速度波动

#### 常见问题与对策

- 速度抖动：减小外环 `Kp` 或减小外环输出影响
- 终点拖尾：加大末端速度衰减/制动约束
- 终点猛刹：降低外环激进度，平缓 `v_ref` 收敛曲线

### 5.2 LQR 调参流程（Q/R 权衡）

- 增 `q11`：更重横向贴合，转向更积极
- 增 `q33`：更重航向一致性，姿态更稳
- 增 `r`：控制更平滑，但跟踪变“软”

推荐顺序：

1. 固定 `r`，先调 `q11/q33` 到稳定跟踪
2. 再调 `r` 平衡“精度-平顺”
3. 若低速抖动，先检查 `v_min` 与离散化稳定性

## 6. 仿真结果

最终运行输出：

- `Simulation finished. GIF: output/demo.gif`
- `final s=74.0355, final v=0, ref_end_s=75`
- GIF 帧数 `145`，帧率 `12 FPS`

说明车辆可从较大初始偏差收敛至轨迹附近，并在终点停车。

![仿真结果](output/demo.gif)
