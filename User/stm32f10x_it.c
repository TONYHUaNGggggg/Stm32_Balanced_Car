 #include "stm32f10x_it.h"
 #include <stdio.h>
 #include "upstandingcar.h"
 #include "outputdata.h"
 #include "mpu6050.h"
 #include "UltrasonicWave.h"
 #include "stm32f10x_exti.h"
 
void NMI_Handler(void)
{
}


void HardFault_Handler(void)
{
  
  while (1)
  {
  }
}


void MemManage_Handler(void)
{
  
  while (1)
  {
  }
}


void BusFault_Handler(void)
{
  
  while (1)
  {
  }
}


void UsageFault_Handler(void)
{
  
  while (1)
  {
  }
}


void SVC_Handler(void)
{
}


void DebugMon_Handler(void)
{
}


void PendSV_Handler(void)
{
}



#if 1
void SysTick_Handler(void)				 
{  
	BST_u8MainEventCount++;				   
	BST_u8trig++;
	BST_u8SpeedControlCount++;			  

	GetMotorPulse();						

	BST_u8SpeedControlPeriod++;

	BST_u8DirectionControlPeriod++;		   
	BST_u8DirectionControlCount++;

	if(BST_u8trig>=2)
	{
		UltrasonicWave_StartMeasure();	
		chaoshengbo();			       
		BST_u8trig=0;
	}
    if(BST_u8SpeedControlCount>=8)       
	{	
		SpeedControl();                     
		BST_u8SpeedControlCount=0;		  
		BST_u8SpeedControlPeriod=0;		  
	}

  AngleControl();					  
  MotorOutput();					  
		


			    
}	   
#endif