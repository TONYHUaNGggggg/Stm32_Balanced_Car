

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

#define STARTUP_BALANCE_TICKS   1000u
// Startup running speed after the initial balance-hold window.
#define STARTUP_RUN_SPEED_CMD   110.0f


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
				AutoRun_SetStraight(STARTUP_RUN_SPEED_CMD);
				u8StartupDone = 1;
			}
		}

		CarStateOut();		
		SendAutoUp();
		
	 }
 								    
}
