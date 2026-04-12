







#ifndef __HAL_MPU6050_H
#define __HAL_MPU6050_H

#ifdef __cplusplus
 extern "C" {
#endif 
  

#include "stm32f10x.h"



#define MPU6050_I2C                  I2C1
#define MPU6050_I2C_RCC_Periph       RCC_APB1Periph_I2C1
#define MPU6050_I2C_Port             GPIOB
#define MPU6050_I2C_SCL_Pin          GPIO_Pin_8
#define MPU6050_I2C_SDA_Pin          GPIO_Pin_9
#define MPU6050_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define MPU6050_I2C_Speed            100000 

  


#endif 


