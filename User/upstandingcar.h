#ifndef __UPSTANDINGCAR_H
#define __UPSTANDINGCAR_H
#include "stm32f10x.h"

#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0) 


									
#define    CAR_ZERO_ANGLE (0)		 


#define CAR_SPEED_SET         0
// 速度命令极性：若正速度命令方向与期望相反，改为 -1.0f。
#define SPEED_CMD_POLARITY    (-1.0f)
#define MOTOR_LEFT_SPEED_POSITIVE  (BST_fLeftMotorOut >0)
#define MOTOR_RIGHT_SPEED_POSITIVE (BST_fRightMotorOut>0)
#define OPTICAL_ENCODE_CONSTANT  13	
#define SPEED_CONTROL_PERIOD	 40	    
#define CAR_SPEED_CONSTANT		(1000.0/(float)SPEED_CONTROL_PERIOD/(float)OPTICAL_ENCODE_CONSTANT)



#define SPEED_ERR_I_MAX	8000
#define SPEED_ERR_I_MIN	(-8000)
// 速度指令接近0时的停稳控制参数（用于抑制刹停大幅摆动）。
#define SPEED_STOP_CMD_DEADBAND         8.0f
#define SPEED_STOP_ACTUAL_DEADBAND      30.0f
#define SPEED_STOP_ANGLE_REF_MAX        1.2f
#define SPEED_I_BLEED_WHEN_STOP         0.90f
#define SPEED_I_UNWIND_GAIN             2.2f
#define ANGLE_REF_SLEW_STEP             0.20f
#define CAR_ANGLE_REF_MAX   6.0f
#define CAR_ANGLE_REF_MIN   (-6.0f)

#define MOTOR_OUT_DEAD_VAL       0	   
#define MOTOR_OUT_MAX           1000	   
#define MOTOR_OUT_MIN         (-1000)   


#define FIXED_SPEED_MODE       1
#define FIXED_SPEED_CMD        0.0f
#define FIXED_TURN_CMD         0.0f

// 8字轨迹单段弧线的速度指令。
#define EIGHT_DEFAULT_BASE_SPEED_CMD   200.0f
// 8字轨迹单段弧线的转向幅度指令。
#define EIGHT_DEFAULT_TURN_AMPL_CMD    100.0f
// 兼容旧接口的参数。
#define EIGHT_DEFAULT_OMEGA_RAD        1.20f
#define EIGHT_DEFAULT_START_PHASE_RAD  1.5707963f
#define EIGHT_CTRL_DT_SEC              0.005f
#define EIGHT_DEFAULT_SPEED_MOD_RATIO  0.15f
// 8字轨迹由两个弧线任务组成（左圆 + 右圆）。
#define EIGHT_ARC_DURATION_TICK        900u
#define EIGHT_ARC_ACCEL_STEP           0.35f
#define EIGHT_ARC_START_RIGHT          1u

// 单任务运动约束（速度/转向 + 时长 + 加速步长）。
#define MOTION_SPEED_CMD_MAX           260.0f
#define MOTION_SPEED_CMD_MIN           (-260.0f)
#define MOTION_TURN_CMD_MAX            260.0f
#define MOTION_TURN_CMD_MIN            (-260.0f)
#define MOTION_TASK_DURATION_MIN_TICK  1u
#define MOTION_TASK_DURATION_MAX_TICK  5000u
#define MOTION_ACCEL_STEP_MIN          0.01f
#define MOTION_ACCEL_STEP_MAX          3.00f
#define MOTION_DEFAULT_ACCEL_STEP      0.35f

// 直行模式目标速度。
#define STRAIGHT_DEFAULT_SPEED_CMD     100.0f
// 超声波避障阈值（单位：cm）。
#define ULTRA_OBS_SLOW_CM              25.0f
#define ULTRA_OBS_STOP_CM              10.0f
#define ULTRA_OBS_VALID_MIN_CM         2.5f
#define ULTRA_OBS_CONFIRM_CNT          12u
#define ULTRA_OBS_STOP_CONFIRM_CNT     4u
// 直行模式检测到障碍时的右避障速度指令。
#define ULTRA_OBS_RIGHT_AVOID_SPEED_CMD 90.0f
// 右避障转向幅度。
#define ULTRA_OBS_RIGHT_AVOID_TURN_CMD  90.0f
#define ULTRA_OBS_RIGHT_TURN_SPEED_RATIO 0.80f

// 关闭轮速动态补偿，仅保留固定基速偏置。
#define WHEEL_BALANCE_ENABLE           0

// 直行基速人工偏置：用于先手工校正左右轮差，再做转向。
#define WHEEL_BASE_BIAS_ENABLE             1
// 左轮基速偏置（>0 左轮更快，<0 左轮更慢）。
#define WHEEL_LEFT_BASE_BIAS               0.0f
// 右轮基速偏置（>0 右轮更快，<0 右轮更慢）。
#define WHEEL_RIGHT_BASE_BIAS              8.0f
// 仅在该速度以上启用基速偏置，避免低速抖动。
#define WHEEL_BASE_BIAS_ACTIVE_SPEED_CMD   30.0f
// 仅在接近直行（方向指令绝对值小于阈值）时启用基速偏置。
#define WHEEL_BASE_BIAS_STRAIGHT_DIR_MAX   25.0f

#define	MOTOR_LEFT_AIN1_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_15))  
#define	MOTOR_LEFT_AIN1_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_15))	  
#define	MOTOR_LEFT_AIN2_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_14))  
#define	MOTOR_LEFT_AIN2_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_14))	  

#define	MOTOR_RIGHT_BIN1_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_12))
#define	MOTOR_RIGHT_BIN1_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_12))		
#define	MOTOR_RIGHT_BIN2_LOW			(GPIO_ResetBits(GPIOB, GPIO_Pin_13))
#define	MOTOR_RIGHT_BIN2_HIGH		(GPIO_SetBits(GPIOB, GPIO_Pin_13))		

extern float BST_fCarAngle;					
extern float BST_fAngleRef;
extern float BST_fBluetoothSpeed;
extern float BST_fBluetoothDirectionR;
extern float BST_fBluetoothDirectionL;
extern u8 BST_u8MainEventCount;
extern u8 BST_u8SpeedControlCount;
extern float BST_fSpeedControlOut,BST_fCarAngle_P;
extern float  BST_fAngleControlOut;
extern float BST_fSpeedControlOutNew;
extern u8 BST_u8SpeedControlPeriod;
extern u8 BST_u8DirectionControlPeriod;
extern u8 BST_u8DirectionControlCount;
extern u8 BST_u8LEDCount; 
extern u8 BST_u8trig;
extern u8 BST_u8turnPeriod;
extern u8 BST_u8turnCount;
extern u8 ucBluetoothValue;
extern float angle;
extern float anglex;
extern float gyx,gy0;
extern float gyrx;
extern float gyry;
extern float accelx,accely,accelz,gyrx,gyry,gyrz;
extern float BST_fLeftMotorOut,BST_fRightMotorOut,BST_fBluetoothDirectionNew;
extern float BST_fCarSpeedOld;
extern s16 BST_s16LeftMotorPulse,BST_s16RightMotorPulse;
extern float juli;
extern 	int x,y1,z1,y2,z2,flagbt;
extern float BST_fCarSpeed_I,BST_fCarSpeed_P,BST_fCarAngle_P,BST_fCarAngle_D;

extern void CarStateOut(void);   	
extern void SendAutoUp(void);

void AutoRun_SetStraight(float speedCmd);
// 运行时动态更新避障速度与转向幅度。
void AutoRun_SetObsRightAvoid(float avoidSpeedCmd, float avoidTurnCmd);
// 用双弧线参数配置8字轨迹：速度 + 转向 + 单弧时长 + 加速步长。
void AutoRun_SetFigureEightArc(float arcSpeedCmd, float arcTurnCmd, u16 arcDurationTick, float accelStep, u8 startRightArc);
void AutoRun_SetFigureEight(float baseSpeedCmd, float turnAmplCmd, float omegaRad, float startPhaseRad, float speedModRatio);
void AutoRun_SetFixed(float speedCmd, float turnCmd);

// 单任务运动接口，便于独立调参与调试。
void MotionTask_SetStraight(float targetSpeedCmd, float accelStep, u16 durationTick);
// 直行单任务（含避障开关）：obsEnable=1开启超声波避障，0关闭。
void MotionTask_SetStraightWithObs(float targetSpeedCmd, float accelStep, u16 durationTick, u8 obsEnable);
// 运行中动态开关直行任务避障：1开启，0关闭。
void MotionTask_SetStraightObsAvoidEnable(u8 obsEnable);
void MotionTask_SetArc(float speedCmd, float turnCmd, u16 durationTick);
void MotionTask_SetTurn(float speedCmd, float turnCmd, u16 durationTick);
void MotionTask_SetSpin(float turnCmd, u16 durationTick);
void MotionTask_Stop(void);


void delay_nms(u16 time);
void CarUpstandInit(void);
void SpeedControl_ResetStopState(void);

void AngleControl(void)	 ;
void MotorOutput(void);
void SpeedControl(void);
void BluetoothControl(void)	;
void GetMotorPulse(void);
void SpeedControlOutput(void);
void DirectionControlOutput(void);
void DirectionControl(void); 
void chaoshengbo(void);
void gfcsbOutput(void);
void csbcontrol(void);
void turn(void);
void turnfliteroutput(void);
void InitMPU6050(void);
void kalmanfilter(float Gyro,float Accel);
void kalmanangle(void);

#endif
