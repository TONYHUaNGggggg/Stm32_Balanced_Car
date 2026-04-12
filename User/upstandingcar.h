#ifndef __UPSTANDINGCAR_H
#define __UPSTANDINGCAR_H
#include "stm32f10x.h"

#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0) 


									
#define    CAR_ZERO_ANGLE (0)		 


#define CAR_POSITION_SET      0
#define CAR_SPEED_SET         0
#define MOTOR_LEFT_SPEED_POSITIVE  (BST_fLeftMotorOut >0)
#define MOTOR_RIGHT_SPEED_POSITIVE (BST_fRightMotorOut>0)
#define OPTICAL_ENCODE_CONSTANT  13	
#define SPEED_CONTROL_PERIOD	 40	    
#define CAR_SPEED_CONSTANT		(1000.0/(float)SPEED_CONTROL_PERIOD/(float)OPTICAL_ENCODE_CONSTANT)



#define CAR_POSITION_MAX	8000       
#define CAR_POSITION_MIN	(-8000)     
#define CAR_ANGLE_REF_MAX   6.0f
#define CAR_ANGLE_REF_MIN   (-6.0f)

#define MOTOR_OUT_DEAD_VAL       0	   
#define MOTOR_OUT_MAX           1000	   
#define MOTOR_OUT_MIN         (-1000)   


#define FIXED_SPEED_MODE       1
#define FIXED_SPEED_CMD        0.0f
#define FIXED_TURN_CMD         0.0f

#define EIGHT_DEFAULT_BASE_SPEED_CMD   200.0f
#define EIGHT_DEFAULT_TURN_AMPL_CMD    100.0f
#define EIGHT_DEFAULT_OMEGA_RAD        1.20f
#define EIGHT_DEFAULT_START_PHASE_RAD  1.5707963f
#define EIGHT_CTRL_DT_SEC              0.005f
#define EIGHT_DEFAULT_SPEED_MOD_RATIO  0.15f

// Straight mode target speed command.
#define STRAIGHT_DEFAULT_SPEED_CMD     100.0f
// Ultrasonic obstacle thresholds (unit: cm).
#define ULTRA_OBS_SLOW_CM              30.0f
#define ULTRA_OBS_STOP_CM              5.0f

#define WHEEL_BALANCE_ENABLE           1
#define WHEEL_BALANCE_KP               0.16f
#define WHEEL_BALANCE_KI               0.003f
#define WHEEL_BALANCE_I_LIMIT          40.0f
#define WHEEL_BALANCE_OUT_LIMIT        80.0f
// Wheel-balance compensation starts from this speed command.
#define WHEEL_BALANCE_ACTIVE_SPEED_CMD 30.0f
#define WHEEL_BALANCE_ERR_DEADBAND     2.0f

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
extern s16 BST_s16LeftMotorPulse,BST_s16RightMotorPulse;
extern float juli;
extern 	int x,y1,z1,y2,z2,flagbt;
extern float BST_fCarSpeed_I,BST_fCarSpeed_P,BST_fCarAngle_P,BST_fCarAngle_D;

extern void CarStateOut(void);   	
extern void SendAutoUp(void);

void AutoRun_SetStraight(float speedCmd);
void AutoRun_SetFigureEight(float baseSpeedCmd, float turnAmplCmd, float omegaRad, float startPhaseRad, float speedModRatio);
void AutoRun_SetFixed(float speedCmd, float turnCmd);


void delay_nms(u16 time);
void CarUpstandInit(void);

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
