# STM32平衡车系统说明（当前代码）

更新日期：2026-04-17

## 1. 你当前最关心的问题：车身角度内环PD怎么调、在哪里调

本项目车身角度内环控制在 User/upstandingcar.c 的 AngleControl() 中：

BST_fAngleControlOut = (BST_fCarAngle - BST_fAngleRef) * BST_fCarAngle_P + gyro[0] * BST_fCarAngle_D;

### 1.1 参数位置（编译时）

在 User/upstandingcar.c 中的全局变量默认值：

- BST_fCarAngle_P = 170.0
- BST_fCarAngle_D = 0.24

这两个就是车身角度内环PD，不是左右轮转速PID。

### 1.2 参数位置（运行时覆盖）

同样在 AngleControl() 内支持运行时覆盖：

- 当 flagbt == 1 时：BST_fCarAngle_P = y1 * 1.71875
- 当 flagbt == 2 时：BST_fCarAngle_D = (z1 - 64) * 0.15625

说明：当前仓库代码里没有看到 flagbt、y1、z1 的串口赋值实现，如果你在上位机调参，通常是外部通信代码或调试器在改这些变量。

### 1.3 针对“释放初始角不为0导致大幅前后摆动”的调参顺序

建议按下面顺序，单次小步改动并观察 5s 静稳阶段：

1. 先固定 D，调 P 到“能快速拉回但不连续放大摆动”。
2. 在该 P 基础上逐步增加 D，直到摆动衰减明显、过冲减小。
3. 若出现高频抖动（发颤），先减 D，再微减 P。
4. 若出现低频大幅摇摆（慢慢来回甩），先加 D，再少量加 P。
5. 若回正很慢，适当加 P；若“冲过头再回来”明显，适当减 P 并加 D。

建议步长（基于当前默认值）

- P：每次改 5 到 10
- D：每次改 0.02 到 0.05

可作为起步的尝试区间

- P：150 到 210
- D：0.18 到 0.50

### 1.4 与该问题强相关的其他量（不要忽略）

- 零倾角偏置 CAR_ZERO_ANGLE（User/upstandingcar.h）
  - 如果机械零位和代码零位不一致，再好的PD也会长期偏摆。
- 角度参考限幅 CAR_ANGLE_REF_MAX/MIN（默认 ±6 度）
  - 外环给出的倾角目标被限幅，过小会限制加速纠偏能力，过大可能引入冲击。
- 启动静稳时长 STARTUP_BALANCE_TICKS（User/main.c）
  - 当前是 1000 tick；系统1 tick=5ms，即约5秒。

## 2. 当前代码控制架构与执行节拍

### 2.1 主流程（User/main.c）

1. 完成硬件初始化与姿态初始化。
2. 先执行 AutoRun_SetFixed(0, 0) 进入静止控制。
3. 主循环中持续调用 MPU6050_Pose() 更新姿态。
4. 启动阶段累计 1000 tick（约5秒）后，进入直行任务 MotionTask_SetStraightWithObs(...)。
5. 循环调用 CarStateOut() 生成速度/转向指令，并通过 SendAutoUp()上传数据。

### 2.2 中断节拍（User/stm32f10x_it.c）

SysTick周期为 5ms（SysTick_Config(SystemCoreClock / 200)）：

1. 每 5ms：GetMotorPulse()、AngleControl()、MotorOutput()。
2. 每 8 个 tick（40ms）：SpeedControl()。
3. 每 2 个 tick（10ms）：触发超声测距与距离处理。

### 2.3 串级控制链路

1. 速度外环（SpeedControl）根据编码器速度/位置误差生成 BST_fSpeedControlOutNew。
2. 将外环输出转换为角度参考 BST_fAngleRef，并限幅在 CAR_ANGLE_REF_MAX/MIN。
3. 角度内环（AngleControl）计算 BST_fAngleControlOut。
4. 电机输出（MotorOutput）融合角度输出与方向输出，限幅后驱动左右轮。

## 3. 功能实现说明

### 3.1 自动模式入口

控制模式由 g_enAutoRunMode 选择，统一在 CarStateOut() 中分发：

1. enAutoRunStraight：直行 + 超声波避障修正。
2. enAutoRunFigureEight：8字轨迹（由左右弧段任务交替构成）。
3. enAutoRunTask：单任务模式（直行/弧线/转弯/原地转向）。
4. enAutoRunFixed：固定速度与固定转向。

### 3.2 直行避障逻辑（MotionApplyObstacleAvoid）

1. 有效距离区间：ULTRA_OBS_VALID_MIN_CM 到 ULTRA_OBS_SLOW_CM。
2. 小于等于 STOP 阈值并达到确认次数：速度与方向置零。
3. 介于 STOP 与 SLOW 之间并达到确认次数：执行右避障速度与右转指令。
4. 右转指令会再按速度比例限幅，防止转向过激。

### 3.3 8字轨迹逻辑

1. 8字由两段弧线任务交替执行。
2. 可配置基速、转向幅度、单弧持续时间、加速步长、起始方向。
3. 兼容接口 AutoRun_SetFigureEight() 内部会换算为弧段参数。

### 3.4 单任务运动接口

已实现接口：

1. MotionTask_SetStraight
2. MotionTask_SetStraightWithObs
3. MotionTask_SetArc
4. MotionTask_SetTurn
5. MotionTask_SetSpin
6. MotionTask_Stop

## 4. 关键参数阈值与含义

### 4.1 角度与输出相关

| 参数 | 默认值 | 含义 | 调大效果 | 调小效果 | 位置 |
|---|---:|---|---|---|---|
| BST_fCarAngle_P | 170.0 | 角度误差比例增益 | 回正更快，易过冲 | 更稳但回正慢 | User/upstandingcar.c |
| BST_fCarAngle_D | 0.24 | 角速度阻尼增益 | 阻尼增强，抑制摆动 | 阻尼变弱，易来回摆 | User/upstandingcar.c |
| CAR_ZERO_ANGLE | 0 | 机械零位偏置角 | 向一侧偏置补偿增大 | 向另一侧偏置补偿增大 | User/upstandingcar.h |
| CAR_ANGLE_REF_MAX/MIN | +6 / -6 | 外环生成角度参考的限幅 | 更强纠偏能力，但冲击大 | 响应温和，但易“拉不住” | User/upstandingcar.h |
| MOTOR_OUT_MAX/MIN | +1000 / -1000 | 电机输出饱和限幅 | 更强驱动力 | 更保守输出 | User/upstandingcar.h |

### 4.2 启动与任务相关

| 参数 | 默认值 | 含义 | 位置 |
|---|---:|---|---|
| STARTUP_BALANCE_TICKS | 1000 | 起步静稳时长（tick） | User/main.c |
| STARTUP_STRAIGHT_SPEED_CMD | 110.0 | 起步后直行任务目标速度 | User/main.c |
| STARTUP_STRAIGHT_ACCEL_STEP | 0.35 | 起步后任务加速步长 | User/main.c |
| STARTUP_STRAIGHT_DURATION_TICK | 4000 | 起步后任务持续时间（tick） | User/main.c |
| STARTUP_STRAIGHT_OBS_ENABLE | 1 | 起步后直行任务是否避障 | User/main.c |

### 4.3 超声波避障相关

| 参数 | 默认值 | 含义 | 位置 |
|---|---:|---|---|
| ULTRA_OBS_SLOW_CM | 25.0 cm | 进入减速/避障判定区间上界 | User/upstandingcar.h |
| ULTRA_OBS_STOP_CM | 10.0 cm | 进入强制停车判定阈值 | User/upstandingcar.h |
| ULTRA_OBS_VALID_MIN_CM | 2.5 cm | 无效近距抖动过滤下界 | User/upstandingcar.h |
| ULTRA_OBS_CONFIRM_CNT | 12 | 避障触发确认计数 | User/upstandingcar.h |
| ULTRA_OBS_STOP_CONFIRM_CNT | 4 | 停车触发确认计数 | User/upstandingcar.h |
| ULTRA_OBS_RIGHT_AVOID_SPEED_CMD | 90.0 | 右避障时速度指令 | User/upstandingcar.h |
| ULTRA_OBS_RIGHT_AVOID_TURN_CMD | 90.0 | 右避障时转向指令 | User/upstandingcar.h |
| ULTRA_OBS_RIGHT_TURN_SPEED_RATIO | 0.80 | 转向随速度限幅比例 | User/upstandingcar.h |

### 4.4 轮速平衡补偿相关（非角度内环）

| 参数 | 默认值 | 含义 | 位置 |
|---|---:|---|---|
| WHEEL_BALANCE_ENABLE | 1 | 轮速平衡补偿开关 | User/upstandingcar.h |
| WHEEL_BALANCE_KP | 0.20 | 轮速差比例补偿 | User/upstandingcar.h |
| WHEEL_BALANCE_KI | 0.008 | 轮速差积分补偿 | User/upstandingcar.h |
| WHEEL_BALANCE_I_LIMIT | 40.0 | 轮速积分限幅 | User/upstandingcar.h |
| WHEEL_BALANCE_OUT_LIMIT | 80.0 | 轮速补偿输出限幅 | User/upstandingcar.h |
| WHEEL_BALANCE_ACTIVE_SPEED_CMD | 30.0 | 大于该速度才启用轮速补偿 | User/upstandingcar.h |
| WHEEL_BALANCE_ERR_DEADBAND | 2.0 | 轮速差死区 | User/upstandingcar.h |

## 5. 推荐调参流程（可直接执行）

1. 先只做静稳调参：保持固定模式 speed=0, turn=0，不让车进入直行任务。
2. 以当前 P=170, D=0.24 为起点，按 1.3 节的小步策略调整。
3. 每次只改一个参数，记录“释放后 1 秒内最大摆角”和“5 秒是否收敛”。
4. 调到静稳收敛后，再恢复 STARTUP_STRAIGHT_* 参数调起步过程。
5. 如果起步瞬间再出现摆动，优先微增 D，再微调 P。

以上文档内容已按当前代码实现整理，便于你直接对照源码定位与调参。
