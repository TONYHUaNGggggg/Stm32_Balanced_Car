#include "timer.h"
#include "led.h"
 #include "upstandingcar.h"
 #include "outputdata.h"
 #include "mpu6050.h"
 #include "UltrasonicWave.h"
 #include "I2C_MPU6050.h"










void Timerx_Init(u16 arr,u16 psc)
{   TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 

	TIM_TimeBaseStructure.TIM_Period = 5000; 
	TIM_TimeBaseStructure.TIM_Prescaler =(7200-1); 
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 
 
	TIM_ITConfig(  
		TIM1, 
		TIM_IT_Update  |  
		TIM_IT_Trigger,   
		ENABLE  
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);  
							 
}

void TIM1_UP_IRQHandler(void)   
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) 
		{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  

		    
	  }
}

