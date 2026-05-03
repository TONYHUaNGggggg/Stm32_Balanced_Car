

#include "UltrasonicWave.h"
#include "usart.h"
#include "timer.h"
#include "delay.h"
#include "stm32f10x_exti.h"



#define	TRIG_PORT      GPIOB		
#define	ECHO_PORT      GPIOB		
#define	TRIG_PIN       GPIO_Pin_0   
#define	ECHO_PIN       GPIO_Pin_1	
#define ECHO_TIMEOUT_CNT 60
   
int count;


void UltrasonicWave_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);	
    
  GPIO_InitStructure.GPIO_Pin = TRIG_PIN;					 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		     
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         
  GPIO_Init(TRIG_PORT, &GPIO_InitStructure);	                 

  GPIO_InitStructure.GPIO_Pin = ECHO_PIN;				     
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		 
  GPIO_Init(ECHO_PORT,&GPIO_InitStructure);						 
	
	 
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);

 	 EXTI_InitStructure.EXTI_Line=EXTI_Line1;
   EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);		
	

   NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;					
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
   NVIC_Init(&NVIC_InitStructure);  	  
}
											    

void EXTI1_IRQHandler(void)
{
	
     if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
			TIM_SetCounter(TIM1,0);
			TIM_Cmd(TIM1, ENABLE);   
		
			   count=	1;               
            while(count)	                 
			{
          if(TIM_GetCounter(TIM1)>=ECHO_TIMEOUT_CNT) 
					{ 
					    TIM_Cmd(TIM1, DISABLE); 	
						count=0;					
					} 
					else  count=GPIO_ReadInputDataBit(ECHO_PORT,ECHO_PIN); 
						       
			}
			TIM_Cmd(TIM1, DISABLE);			                                 
			
		
	  
	
	EXTI_ClearITPendingBit(EXTI_Line1);  
	
}

}


void UltrasonicWave_StartMeasure(void)
{
  GPIO_SetBits(TRIG_PORT,TRIG_PIN); 		  
  delay_us(12);		                      
  GPIO_ResetBits(TRIG_PORT,TRIG_PIN);	   
	
}


