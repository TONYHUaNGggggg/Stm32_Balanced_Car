# 平衡车功能日志（当前版本）

更新日期：2026-04-11

## 当前默认运行模式
- 默认模式为：直行 + 自动绕行避障
- 入口调用：`AutoRun_SetStraight(STRAIGHT_DEFAULT_SPEED_CMD)`
- 入口位置：`User/main.c`

## 已实现功能

### 1. 自动运行模式封装
- 直行避障模式：`AutoRun_SetStraight(float speedCmd)`
- 8字轨迹模式：`AutoRun_SetFigureEight(float baseSpeedCmd, float turnAmplCmd, float omegaRad, float startPhaseRad, float speedModRatio)`
- 固定速度转向模式：`AutoRun_SetFixed(float speedCmd, float turnCmd)`

### 2. 直行 + 自动绕行避障（含转弯）
- 使用超声波距离 `juli` 判断障碍
- 避障状态机：
  - 巡航（enObsCruise）
  - 后退（enObsBack）
  - 转出（enObsTurnOut）
  - 绕行通过（enObsPass）
  - 回转（enObsTurnBack）
  - 恢复（enObsRecover）
- 说明：检测到近障后不只是减速或后退，而是执行带转向的绕障流程

### 3. 8字轨迹（参数化）
- 通过 `AutoRun_SetFigureEight(...)` 传入参数控制轨迹大小与频率
- 参数说明：
  - `baseSpeedCmd`：基准前进速度
  - `turnAmplCmd`：转向幅度（影响8字横向大小）
  - `omegaRad`：角速度（影响8字周期）
  - `startPhaseRad`：起始相位（可用于从顶部相位起步）
  - `speedModRatio`：速度调制比例

## 关键参数位置
- 自动模式参数：`User/upstandingcar.h`
  - `STRAIGHT_DEFAULT_SPEED_CMD`
  - `ULTRA_OBS_SLOW_CM`
  - `ULTRA_OBS_STOP_CM`
  - `ULTRA_OBS_TURN_CMD`
  - `ULTRA_OBS_BACK_TICK` 等时序参数
- 自动模式核心逻辑：`User/upstandingcar.c`
  - `StraightAvoidControl()`
  - `FigureEightControl()`
  - `CarStateOut()`

## 当前代码状态摘要
- 控制架构：串级（速度外环 -> 角度参考，角度内环 -> 电机输出）
- 默认行为：直行 + 超声波自动绕行避障
- 8字功能：可通过代码调用切换并可参数化控制

## 快速切换示例
- 直行避障：
  - `AutoRun_SetStraight(220.0f);`
- 8字轨迹：
  - `AutoRun_SetFigureEight(200.0f, 100.0f, 1.20f, 1.5707963f, 0.15f);`
- 固定模式：
  - `AutoRun_SetFixed(180.0f, 0.0f);`
