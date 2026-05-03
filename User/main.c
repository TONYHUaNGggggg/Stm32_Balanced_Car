#include "mpu6050.h"
#include "i2c_mpu6050.h"
#include "motor.h"
#include "upstandingcar.h"
#include "SysTick.h"
#include "led.h"
#include "adc.h"
#include "usart.h"
#include "i2c.h"

#include "timer.h"
#include "UltrasonicWave.h"

// 起步静止平衡时间，单位tick（1 tick=5ms）。
#define STARTUP_BALANCE_TICKS   50u
// 正方形单边直行参数（建议先调SIDE，再调TURN）。
#define SQUARE_SIDE_SPEED_CMD            100.0f
#define SQUARE_SIDE_ACCEL_STEP           0.15f
#define SQUARE_SIDE_DURATION_TICK        800u
// 转向前刹停参数：先降到0速度再开始转向。
#define SQUARE_BRAKE_SPEED_CMD           0.0f
#define SQUARE_BRAKE_ACCEL_STEP          1.20f
#define SQUARE_BRAKE_DURATION_TICK       120u
// 90度原地转向参数：正负号决定左右转。
#define SQUARE_TURN_CMD                  -500.0f
#define SQUARE_TURN_DURATION_TICK        180u
// 转向后静止回正：满足最小等待且倾角回到阈值内再进入下一边。
#define SQUARE_SETTLE_ANGLE_DEG          1.5f
#define SQUARE_SETTLE_SPEED_ABS_MAX       10.0f
#define SQUARE_SETTLE_MIN_TICK           60u
#define SQUARE_SETTLE_MAX_TICK           250u
#define SQUARE_EDGE_COUNT                4u

typedef enum {
	enSquarePhaseWaitBalance = 0,
	enSquarePhaseStartSide,
	enSquarePhaseWaitSide,
	enSquarePhaseStartBrake,
	enSquarePhaseWaitBrake,
	enSquarePhaseStartTurn,
	enSquarePhaseWaitTurn,
	enSquarePhaseStartSettle,
	enSquarePhaseWaitSettle,
	enSquarePhaseDone
} enSquarePhase;


float gyz;
int acc;
int acc1;




int main(void)
{	
	u32 u32PhaseTicks = 0;
	u8 u8MainEventLast = 0;
	u8 u8SquareEdgeIndex = 0;
	enSquarePhase enSquarePhaseState = enSquarePhaseWaitBalance;

       
	
	SystemInit();                   
	Timerx_Init(5000,7199);				   
	UltrasonicWave_Configuration(); 	   

	USART1_Config();						

	TIM2_PWM_Init();					   
	MOTOR_GPIO_Config();				  
	LED_GPIO_Config();
	Adc_Init();
	
	
	TIM3_Encoder_Init();                       
	TIM4_Encoder_Init();                       
	
	i2cInit();							   
	delay_nms(10);						   
	MPU6050_Init();						   

	SysTick_Init();						  
	CarUpstandInit();					  
	AutoRun_SetFixed(0.0f, 0.0f);
	u8MainEventLast = BST_u8MainEventCount;
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;	 

	while (1)
	{


		MPU6050_Pose();		

		{
			u8 u8MainEventNow = BST_u8MainEventCount;
			u32PhaseTicks += (u8)(u8MainEventNow - u8MainEventLast);
			u8MainEventLast = u8MainEventNow;
		}

		switch(enSquarePhaseState)
		{
			case enSquarePhaseWaitBalance:
				if(u32PhaseTicks >= STARTUP_BALANCE_TICKS)
				{
					u32PhaseTicks = 0;
					u8SquareEdgeIndex = 0;
					enSquarePhaseState = enSquarePhaseStartSide;
				}
				break;

			case enSquarePhaseStartSide:
				MotionTask_SetStraight(
					SQUARE_SIDE_SPEED_CMD,
					SQUARE_SIDE_ACCEL_STEP,
					SQUARE_SIDE_DURATION_TICK
				);
				u32PhaseTicks = 0;
				enSquarePhaseState = enSquarePhaseWaitSide;
				break;

			case enSquarePhaseWaitSide:
				if(u32PhaseTicks >= SQUARE_SIDE_DURATION_TICK)
				{
					u32PhaseTicks = 0;
					u8SquareEdgeIndex++;
					enSquarePhaseState = enSquarePhaseStartBrake;
				}
				break;

			case enSquarePhaseStartBrake:
				MotionTask_SetStraight(
					SQUARE_BRAKE_SPEED_CMD,
					SQUARE_BRAKE_ACCEL_STEP,
					SQUARE_BRAKE_DURATION_TICK
				);
				u32PhaseTicks = 0;
				enSquarePhaseState = enSquarePhaseWaitBrake;
				break;

			case enSquarePhaseWaitBrake:
				if(u32PhaseTicks >= SQUARE_BRAKE_DURATION_TICK)
				{
					u32PhaseTicks = 0;
					if(u8SquareEdgeIndex >= SQUARE_EDGE_COUNT)
					{
						enSquarePhaseState = enSquarePhaseStartSettle;
					}
					else
					{
						enSquarePhaseState = enSquarePhaseStartTurn;
					}
				}
				break;

			case enSquarePhaseStartTurn:
				MotionTask_SetSpin(SQUARE_TURN_CMD, SQUARE_TURN_DURATION_TICK);
				u32PhaseTicks = 0;
				enSquarePhaseState = enSquarePhaseWaitTurn;
				break;

			case enSquarePhaseWaitTurn:
				if(u32PhaseTicks >= SQUARE_TURN_DURATION_TICK)
				{
					u32PhaseTicks = 0;
					enSquarePhaseState = enSquarePhaseStartSettle;
				}
				break;

			case enSquarePhaseStartSettle:
				AutoRun_SetFixed(0.0f, 0.0f);
				u32PhaseTicks = 0;
				enSquarePhaseState = enSquarePhaseWaitSettle;
				break;

			case enSquarePhaseWaitSettle:
				if(
					((u32PhaseTicks >= SQUARE_SETTLE_MIN_TICK)
						&& (BST_fCarAngle > -SQUARE_SETTLE_ANGLE_DEG)
						&& (BST_fCarAngle < SQUARE_SETTLE_ANGLE_DEG)
						&& (BST_fCarSpeedOld > -SQUARE_SETTLE_SPEED_ABS_MAX)
						&& (BST_fCarSpeedOld < SQUARE_SETTLE_SPEED_ABS_MAX))
					|| (u32PhaseTicks >= SQUARE_SETTLE_MAX_TICK)
				)
				{
					u32PhaseTicks = 0;
					if(u8SquareEdgeIndex >= SQUARE_EDGE_COUNT)
					{
						MotionTask_Stop();
						SpeedControl_ResetStopState();
						AutoRun_SetFixed(0.0f, 0.0f);
						enSquarePhaseState = enSquarePhaseDone;
					}
					else
					{
						enSquarePhaseState = enSquarePhaseStartSide;
					}
				}
				break;

			case enSquarePhaseDone:
				SpeedControl_ResetStopState();
				AutoRun_SetFixed(0.0f, 0.0f);
				break;
			default:
				break;
		}

		CarStateOut();		
		SendAutoUp();
		
	 }
 								    
}
