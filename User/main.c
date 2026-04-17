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
#define STARTUP_BALANCE_TICKS   500u
// 直行单任务调参参数：目标速度（越大越快）。
#define STARTUP_STRAIGHT_SPEED_CMD       150.0f
// 直行单任务调参参数：每个控制节拍的加速步长（越大加速越快）。
#define STARTUP_STRAIGHT_ACCEL_STEP      0.35f
// 直行单任务调参参数：任务持续时间，单位tick（duration*5ms）。
#define STARTUP_STRAIGHT_DURATION_TICK   1000u
// 直行任务是否开启超声波避障：1开启，0关闭。
#define STARTUP_STRAIGHT_OBS_ENABLE      1u


float gyz;
int acc;
int acc1;




int main(void)
{	
	u32 u32StartupTicks = 0;
	u8 u8MainEventLast = 0;
	u8 u8StartupDone = 0;

       
	
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
		

		if(u8StartupDone == 0)
		{
			u8 u8MainEventNow = BST_u8MainEventCount;
			u32StartupTicks += (u8)(u8MainEventNow - u8MainEventLast);
			u8MainEventLast = u8MainEventNow;
			if(u32StartupTicks >= STARTUP_BALANCE_TICKS)
			{
				MotionTask_SetStraightWithObs(
					STARTUP_STRAIGHT_SPEED_CMD,
					STARTUP_STRAIGHT_ACCEL_STEP,
					STARTUP_STRAIGHT_DURATION_TICK,
					STARTUP_STRAIGHT_OBS_ENABLE
				);
				u8StartupDone = 1;
			}
		}

		CarStateOut();		
		SendAutoUp();
		
	 }
 								    
}
