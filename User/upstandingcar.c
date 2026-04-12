#include "upstandingcar.h"
#include "I2C_MPU6050.h"
#include "MOTOR.h"
#include "led.h"
#include "USART.H"
#include "MPU6050.H"
#include "UltrasonicWave.h"
#include "stm32f10x_gpio.h"
#include "math.h" 
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "adc.h"



enum{
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT
}enCarState;


int g_newcarstate = enSTOP; 
static u32 BST_u32EightTick = 0;
static u8 BST_u8MainEventLast = 0;

typedef enum {
	enAutoRunStraight = 0,
	enAutoRunFigureEight,
	enAutoRunFixed
} enAutoRunMode;

typedef struct {
	float fBaseSpeedCmd;
	float fTurnAmplCmd;
	float fOmegaRad;
	float fStartPhaseRad;
	float fSpeedModRatio;
} stFigureEightParam;

static enAutoRunMode g_enAutoRunMode = enAutoRunStraight;
static stFigureEightParam g_stFigureEightParam = {
	EIGHT_DEFAULT_BASE_SPEED_CMD,
	EIGHT_DEFAULT_TURN_AMPL_CMD,
	EIGHT_DEFAULT_OMEGA_RAD,
	EIGHT_DEFAULT_START_PHASE_RAD,
	EIGHT_DEFAULT_SPEED_MOD_RATIO
};
static float g_fStraightSpeedCmd = STRAIGHT_DEFAULT_SPEED_CMD;
static float g_fFixedSpeedCmd = FIXED_SPEED_CMD;
static float g_fFixedTurnCmd = FIXED_TURN_CMD;
static u8 BST_u8StraightEventLast = 0;
static float g_fWheelBalanceI = 0.0f;
static float g_fWheelBalanceOut = 0.0f;


char manydisplay[80] ={0};
char updata[80] ={0};




u8 BST_u8MainEventCount;						  
u8 BST_u8SpeedControlCount;						  
u8 BST_u8SpeedControlPeriod;
u8 BST_u8DirectionControlPeriod;
u8 BST_u8DirectionControlCount;					  
u8 BST_u8trig;
u8 ucBluetoothValue;                      
float volt = 12.0;




float BST_fSpeedControlOut;						   
float BST_fSpeedControlOutOld;
float BST_fSpeedControlOutNew;
float BST_fAngleRef;
float BST_fAngleControlOut;
float BST_fLeftMotorOut;
float BST_fRightMotorOut;

float BST_fCarAngle;						 
float gyro_z;
float gyrx;
float gy0;


float  BST_fCarAngle_P = 170.0;
float  BST_fCarAngle_D = 0.24;

float  BST_fCarSpeed_P = 4.5;
float  BST_fCarSpeed_I = 0.12;

const double PID_Original[4] ={170.0, 0.24, 4.5, 0.12}; 
char  alldata[80];
char *iap;


s16   BST_s16LeftMotorPulse;					  
s16	  BST_s16RightMotorPulse;					   

s32   BST_s32LeftMotorPulseOld;
s32   BST_s32RightMotorPulseOld;
s32   BST_s32LeftMotorPulseSigma;				  
s32   BST_s32RightMotorPulseSigma;				 

float BST_fCarSpeed;							 
float BST_fCarSpeedOld;

float BST_fCarPosition;						   


int leftstop=0;
int rightstop=0;
int stopflag=0;



float fchaoshengbo = 0;							   
float juli = 0;									 

																	
float BST_fBluetoothSpeed;						
float BST_fBluetoothDirectionNew;			    
float BST_fBluetoothDirectionSL;			    
float BST_fBluetoothDirectionSR;			   
int chaoflag=0;
int x,y1,z1,y2,z2,flagbt;
int g_autoup = 0;
int g_uptimes = 5000;  
char charkp[10],charkd[10],charksp[10],charksi[10];
char lspeed[10],rspeed[10],daccel[10],dgyro[10],csb[10],vi[10];
char kp,kd,ksp,ksi;
float dac = 0,dgy = 0;



float BST_fBluetoothDirectionL;				   
float BST_fBluetoothDirectionR;				   
int driectionxco=800;



		
float  Q_angle=0.001;  
float  Q_gyro=0.003;
float  R_angle=0.5;
float  dt=0.005;	                  
char   C_0 = 1;
float  Q_bias, Angle_err;
float  PCt_0=0, PCt_1=0, E=0;
float  K_0=0, K_1=0, t_0=0, t_1=0;
float  Pdot[4] ={0,0,0,0};
float  PP[2][2] = { { 1, 0 },{ 0, 1 } };


void SendAutoUp(void);
static void FigureEightControl(void);
static void StraightAvoidControl(void);


void delay_nms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;  
      while(i--) ;    
   }
}




void CarUpstandInit(void)
{
	
	
	BST_s16LeftMotorPulse = BST_s16RightMotorPulse = 0;					  
	BST_s32LeftMotorPulseSigma = BST_s32RightMotorPulseSigma = 0;		  

	BST_fCarSpeed = BST_fCarSpeedOld = 0;								   
	BST_fCarPosition = 0;												  
	BST_fCarAngle    = 0;												  
	BST_fAngleRef    = 0;

	BST_fAngleControlOut = BST_fSpeedControlOut = BST_fBluetoothDirectionNew = 0;	
	BST_fLeftMotorOut    = BST_fRightMotorOut   = 0;								
	BST_fBluetoothSpeed  = 0;														
	BST_fBluetoothDirectionL =BST_fBluetoothDirectionR= 0;						    
	BST_fBluetoothDirectionSL =BST_fBluetoothDirectionSR= 0;						
	
    BST_u8MainEventCount=0;															
	BST_u8SpeedControlCount=0;														
    BST_u8SpeedControlPeriod=0;														
	BST_u8MainEventLast=BST_u8MainEventCount;
	BST_u32EightTick=0;
	BST_u8StraightEventLast=BST_u8MainEventCount;
	g_fWheelBalanceI = 0.0f;
	g_fWheelBalanceOut = 0.0f;
	AutoRun_SetStraight(STRAIGHT_DEFAULT_SPEED_CMD);

	fchaoshengbo=0;											
  
	

}

void ResetPID()
{	
	if(BST_fCarAngle_P != PID_Original[0])
	{
		BST_fCarAngle_P = PID_Original[0];
	}
	if(BST_fCarAngle_D != PID_Original[1])
	{
		BST_fCarAngle_D = PID_Original[1];
	}
	if(BST_fCarSpeed_P != PID_Original[2])
	{
		BST_fCarSpeed_P = PID_Original[2];
	}
	if(BST_fCarSpeed_I != PID_Original[3])
	{
		BST_fCarSpeed_I = PID_Original[3];
	}
	g_fWheelBalanceI = 0.0f;
	g_fWheelBalanceOut = 0.0f;

}	


void AngleControl(void)	 
{
	if(flagbt==1)
	{
		BST_fCarAngle_P=0;
		BST_fCarAngle_P=y1*1.71875;
	}
		if(flagbt==2)
	{
		BST_fCarAngle_D=0;
		BST_fCarAngle_D=(z1-64)*0.15625;
	}
	dac=accel[2];
	dgy=gyro[2];
	if(Pitch==0||Pitch<-20||Pitch>20)			  
	{
		  
	  GPIO_ResetBits(GPIOC, GPIO_Pin_13); 		 
	}
	else 
	{GPIO_SetBits(GPIOC, GPIO_Pin_13);}			 
	
	BST_fCarAngle = Roll - CAR_ZERO_ANGLE;													   
	BST_fAngleControlOut = (BST_fCarAngle - BST_fAngleRef) * BST_fCarAngle_P + gyro[0] * BST_fCarAngle_D;
}


void SetMotorVoltageAndDirection(s16 s16LeftVoltage,s16 s16RightVoltage)
{
	  u16 u16LeftMotorValue;
	  u16 u16RightMotorValue;
	
    if(s16LeftVoltage<0)										 
    {	
	    GPIO_SetBits(GPIOB, GPIO_Pin_14 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_15 );
      s16LeftVoltage = (-s16LeftVoltage);
    }
    else 
    {	
      GPIO_SetBits(GPIOB, GPIO_Pin_15 );				    	 
      GPIO_ResetBits(GPIOB, GPIO_Pin_14 ); 
      s16LeftVoltage = s16LeftVoltage;
    }

    if(s16RightVoltage<0)
    {															 
      GPIO_SetBits(GPIOB, GPIO_Pin_13 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_12 );
      s16RightVoltage = (-s16RightVoltage);
    }
    else														
    {
	    GPIO_SetBits(GPIOB, GPIO_Pin_12 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_13 );	
     
      s16RightVoltage = s16RightVoltage;
    }
		
	   u16RightMotorValue= (u16)s16RightVoltage;
	   u16LeftMotorValue = (u16)s16LeftVoltage;


	TIM_SetCompare3(TIM2,u16LeftMotorValue);			  
	TIM_SetCompare4(TIM2,u16RightMotorValue);			  

#if 1	 
		
  if(Pitch>10||Pitch<-10&BST_fBluetoothDirectionSR==0&BST_fBluetoothDirectionSL==0)
	{		
		TIM_SetCompare3(TIM2,0);
		TIM_SetCompare4(TIM2,0);
		stopflag=1;		
	}
	else stopflag=0;
	
	if(BST_fCarAngle > 50 || BST_fCarAngle < (-50))
	{
		TIM_SetCompare3(TIM2,0);
		TIM_SetCompare4(TIM2,0);  
		stopflag=1;	
	}
	else stopflag=0;

#endif
}


void MotorOutput(void)																					  
{	   
	float fDirectionCmd;
			
	fDirectionCmd = BST_fBluetoothDirectionNew;
#if WHEEL_BALANCE_ENABLE
	fDirectionCmd += g_fWheelBalanceOut;
#endif

	BST_fLeftMotorOut  = BST_fAngleControlOut + fDirectionCmd;
    BST_fRightMotorOut = BST_fAngleControlOut - fDirectionCmd;

		
	if((s16)BST_fLeftMotorOut  > MOTOR_OUT_MAX)	BST_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((s16)BST_fLeftMotorOut  < MOTOR_OUT_MIN)	BST_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((s16)BST_fRightMotorOut > MOTOR_OUT_MAX)	BST_fRightMotorOut = MOTOR_OUT_MAX;
	if((s16)BST_fRightMotorOut < MOTOR_OUT_MIN)	BST_fRightMotorOut = MOTOR_OUT_MIN;
	
    SetMotorVoltageAndDirection((s16)BST_fLeftMotorOut,(s16)BST_fRightMotorOut);
    
}

void GetMotorPulse(void)              
{ 
	
	uint16_t u16TempLeft;
	uint16_t u16TempRight;
	
	u16TempLeft = TIM_GetCounter(TIM3);   
 	u16TempRight= TIM_GetCounter(TIM4);	
	leftstop=u16TempLeft;
	rightstop=u16TempRight;
	TIM_SetCounter(TIM3,0);
	TIM_SetCounter(TIM4,0);
	BST_s16LeftMotorPulse=u16TempLeft;
	BST_s16RightMotorPulse=(-u16TempRight);
		
	BST_s32LeftMotorPulseSigma  +=BST_s16LeftMotorPulse;		 
	BST_s32RightMotorPulseSigma +=BST_s16RightMotorPulse; 	 
}


void SpeedControl(void)
{
  float fPulseDiff;
  
 
	BST_fCarSpeed = (BST_s32LeftMotorPulseSigma  + BST_s32RightMotorPulseSigma );
	fPulseDiff = (float)(BST_s32RightMotorPulseSigma - BST_s32LeftMotorPulseSigma);
	if(fPulseDiff > -WHEEL_BALANCE_ERR_DEADBAND && fPulseDiff < WHEEL_BALANCE_ERR_DEADBAND)
	{
		fPulseDiff = 0.0f;
	}
	BST_s32LeftMotorPulseSigma =BST_s32RightMotorPulseSigma = 0;	  
	BST_fCarSpeedOld *= 0.7;
	BST_fCarSpeedOld +=BST_fCarSpeed*0.3;
	
	BST_fCarPosition += BST_fCarSpeedOld; 		 
	BST_fCarPosition += BST_fBluetoothSpeed;   
	BST_fCarPosition +=	fchaoshengbo;		   
	if(stopflag==1)
	{
		BST_fCarPosition=0;
		
	}
	

	
	if((s32)BST_fCarPosition > CAR_POSITION_MAX)    BST_fCarPosition = CAR_POSITION_MAX;
	if((s32)BST_fCarPosition < CAR_POSITION_MIN)    BST_fCarPosition = CAR_POSITION_MIN;
	
		if(flagbt==3)
	{
		BST_fCarSpeed_P=0;
		BST_fCarSpeed_P=(y2-128)*0.46875;
	}
		if(flagbt==4)
	{
		BST_fCarSpeed_I=0;
		BST_fCarSpeed_I=(z2-192)*0.15625;
	}
	

																								  
	BST_fSpeedControlOutNew = (BST_fCarSpeedOld -CAR_SPEED_SET ) * BST_fCarSpeed_P + (BST_fCarPosition - CAR_POSITION_SET ) * BST_fCarSpeed_I; 

	if(BST_fCarAngle_P > 0.001f || BST_fCarAngle_P < -0.001f)
	{
		BST_fAngleRef = -BST_fSpeedControlOutNew / BST_fCarAngle_P;
	}
	else
	{
		BST_fAngleRef = 0;
	}

	if(BST_fAngleRef > CAR_ANGLE_REF_MAX) BST_fAngleRef = CAR_ANGLE_REF_MAX;
	if(BST_fAngleRef < CAR_ANGLE_REF_MIN) BST_fAngleRef = CAR_ANGLE_REF_MIN;

#if WHEEL_BALANCE_ENABLE
	if(stopflag == 0 && fabsf(BST_fBluetoothSpeed) >= WHEEL_BALANCE_ACTIVE_SPEED_CMD)
	{
		if(fPulseDiff == 0.0f)
		{
			g_fWheelBalanceI *= 0.98f;
		}
		else
		{
			g_fWheelBalanceI += fPulseDiff * WHEEL_BALANCE_KI;
		}
		if(g_fWheelBalanceI > WHEEL_BALANCE_I_LIMIT) g_fWheelBalanceI = WHEEL_BALANCE_I_LIMIT;
		if(g_fWheelBalanceI < -WHEEL_BALANCE_I_LIMIT) g_fWheelBalanceI = -WHEEL_BALANCE_I_LIMIT;

		g_fWheelBalanceOut = fPulseDiff * WHEEL_BALANCE_KP + g_fWheelBalanceI;
		if(g_fWheelBalanceOut > WHEEL_BALANCE_OUT_LIMIT) g_fWheelBalanceOut = WHEEL_BALANCE_OUT_LIMIT;
		if(g_fWheelBalanceOut < -WHEEL_BALANCE_OUT_LIMIT) g_fWheelBalanceOut = -WHEEL_BALANCE_OUT_LIMIT;
	}
	else
	{
		g_fWheelBalanceI *= 0.8f;
		g_fWheelBalanceOut *= 0.8f;
	}
#endif

}






void chaoshengbo(void)
{  
	if(chaoflag==0)
	{
	
      	juli=TIM_GetCounter(TIM1)*5*34/200.0;
		if(g_enAutoRunMode == enAutoRunStraight)
		{
			fchaoshengbo = 0;
			return;
		}
		
	    if(juli <= 4.00)								  
		{
	    	fchaoshengbo= (-300);
	    }
		else if(juli >= 5 & juli <= 8)
		{
			fchaoshengbo=500;
		}
	    else 
		{
			fchaoshengbo=0;						 
 	    }
  	}
	
	
	
    
 }


void CalcUpData()
{
	float ls, rs, sLence;
	short s_Acc, s_Gyro;
	u16 ad_value;

	
	if(g_autoup == 1)
	{
		
		ls = BST_fLeftMotorOut;
		rs = BST_fRightMotorOut;
		s_Acc = accel[1];
		s_Gyro = gyro[0];
		sLence = juli;
		
		
		dac=(s_Acc/16384.0f)*9.8f;
		dgy=((s_Gyro-128.1f)/131.0f);
		
		ls=ls/3.91;
		rs=rs/3.91;
	
		memset(manydisplay, 0x00, 80);
		memcpy(manydisplay, "$LV", 4);
	
		memset(lspeed, 0x00, sizeof(lspeed));
		memset(rspeed, 0x00, sizeof(rspeed));
		memset(daccel, 0x00, sizeof(daccel));
		memset(dgyro, 0x00, sizeof(dgyro));
		memset(csb, 0x00, sizeof(csb));
		memset(vi, 0x00, sizeof(vi));
	
		if((ls <= 1000) && (ls >= -1000))
			sprintf(lspeed,"%3.2f",ls);
		else
		{
			
			
			return;
		}
			
		
		if((rs <= 1000) && (rs >= -1000))
			sprintf(rspeed,"%3.2f",rs);
		else
		{
			
			
			return;
		}
		
		if((dac > -20) && (dac < 20))
			sprintf(daccel,"%3.2f",dac);
		else
		{
			
			
			return;
		}
		
		if((dgy > -3000) && (dgy < 3000))
			sprintf(dgyro,"%3.2f",dgy);
		else
		{
			
			
			return;
		}
	
		if((sLence >= 0) && (sLence < 10000))
			sprintf(csb,"%3.2f",sLence);
		else
		{
			
			
			return;
		}
		
		ad_value = Get_Adc(0);
		volt = 13.3f*ad_value/4096.0f-0.2f;
		if((volt >= 0) && (volt < 20))
			sprintf(vi,"%3.2f",volt);
		else
		{
			
			
			return;
		}
	
		strcat(manydisplay,lspeed);
		strcat(manydisplay,",RV");
		strcat(manydisplay,rspeed);
		strcat(manydisplay,",AC");
		strcat(manydisplay,daccel);
		strcat(manydisplay,",GY");
		strcat(manydisplay,dgyro);
		strcat(manydisplay,",CSB");
		strcat(manydisplay,csb);
		strcat(manydisplay,",VT");
		strcat(manydisplay,vi);
		strcat(manydisplay,"#");
		memset(updata, 0x00, 80);
		memcpy(updata, manydisplay, 80);
	}
	
}


void SendAutoUp(void)
{
	g_uptimes --;
	if ((g_autoup == 1) && (g_uptimes == 0))
	{
		CalcUpData();
		UART3_Send_Char(updata); 
	}
	if(g_uptimes == 0)
		 g_uptimes = 5000;

}

void AutoRun_SetStraight(float speedCmd)
{
	g_enAutoRunMode = enAutoRunStraight;
	g_fStraightSpeedCmd = speedCmd;
	BST_u8MainEventLast = BST_u8MainEventCount;
	BST_u32EightTick = 0;
	BST_u8StraightEventLast = BST_u8MainEventCount;
}

void AutoRun_SetFigureEight(float baseSpeedCmd, float turnAmplCmd, float omegaRad, float startPhaseRad, float speedModRatio)
{
	if(omegaRad < 0.10f)
	{
		omegaRad = 0.10f;
	}
	if(speedModRatio < 0.0f)
	{
		speedModRatio = 0.0f;
	}
	if(speedModRatio > 0.45f)
	{
		speedModRatio = 0.45f;
	}

	g_stFigureEightParam.fBaseSpeedCmd = baseSpeedCmd;
	g_stFigureEightParam.fTurnAmplCmd = turnAmplCmd;
	g_stFigureEightParam.fOmegaRad = omegaRad;
	g_stFigureEightParam.fStartPhaseRad = startPhaseRad;
	g_stFigureEightParam.fSpeedModRatio = speedModRatio;
	g_enAutoRunMode = enAutoRunFigureEight;
	BST_u8MainEventLast = BST_u8MainEventCount;
	BST_u32EightTick = 0;
}

void AutoRun_SetFixed(float speedCmd, float turnCmd)
{
	g_enAutoRunMode = enAutoRunFixed;
	g_fFixedSpeedCmd = speedCmd;
	g_fFixedTurnCmd = turnCmd;
	BST_u8MainEventLast = BST_u8MainEventCount;
	BST_u32EightTick = 0;
}

static void StraightAvoidControl(void)
{
	float fSpeedCmd;
	float fObsCm;

	fSpeedCmd = g_fStraightSpeedCmd;
	fObsCm = juli;

	if(fObsCm > 0.01f && fObsCm <= ULTRA_OBS_STOP_CM)
	{
		fSpeedCmd = 0.0f;
	}

	BST_fBluetoothSpeed = fSpeedCmd;
	BST_fBluetoothDirectionNew = 0;
	chaoflag = 0;
	fchaoshengbo = 0;
}

static void FigureEightControl(void)
{
	u8 u8MainEventNow;
	u8 u8DeltaEvent;
	float fPhase;
	float fSpeedScale;

	u8MainEventNow = BST_u8MainEventCount;
	u8DeltaEvent = (u8)(u8MainEventNow - BST_u8MainEventLast);
	BST_u8MainEventLast = u8MainEventNow;
	BST_u32EightTick += u8DeltaEvent;

	fPhase = g_stFigureEightParam.fStartPhaseRad + g_stFigureEightParam.fOmegaRad * ((float)BST_u32EightTick * EIGHT_CTRL_DT_SEC);
	fSpeedScale = 1.0f - g_stFigureEightParam.fSpeedModRatio + g_stFigureEightParam.fSpeedModRatio * (float)cos(2.0f * fPhase);

	BST_fBluetoothSpeed = g_stFigureEightParam.fBaseSpeedCmd * fSpeedScale;
	BST_fBluetoothDirectionNew = g_stFigureEightParam.fTurnAmplCmd * (float)sin(fPhase);
	chaoflag = 1;
	fchaoshengbo = 0;
}

void CarStateOut(void)
{
#if FIXED_SPEED_MODE
	switch (g_enAutoRunMode)
	{
		case enAutoRunFigureEight:
			FigureEightControl();
			break;

		case enAutoRunFixed:
			BST_fBluetoothSpeed = g_fFixedSpeedCmd;
			BST_fBluetoothDirectionNew = g_fFixedTurnCmd;
			chaoflag = 1;
			fchaoshengbo = 0;
			break;

		case enAutoRunStraight:
		default:
			StraightAvoidControl();
			break;
	}
	return;
#endif

	switch (g_newcarstate)
	{
		case enSTOP: 
		{

			BST_fBluetoothSpeed = 0;
			fchaoshengbo=0;
			BST_fBluetoothDirectionNew=0;
			chaoflag=0;

		} break; 					   

		case enRUN: 
		{
			BST_fBluetoothDirectionNew= 0; 	
			
			BST_fBluetoothSpeed =  800 ;
			chaoflag=1;

		}break;	   

		case enLEFT:
		{

			BST_fBluetoothDirectionNew= -300; 
			chaoflag=1;

		}break;   
		
		case enRIGHT: 
		{

			BST_fBluetoothDirectionNew= 300; 
			chaoflag=1;

		}break;	
		
		case enBACK: 
		{
			BST_fBluetoothDirectionNew= 0; 
			
			BST_fBluetoothSpeed = (-800);
			chaoflag=1;  

		}break;
		
		case enTLEFT: 
		{
			BST_fBluetoothDirectionNew = -driectionxco; 
			chaoflag=1; 

		}break;
		case enTRIGHT: 
		{
			BST_fBluetoothDirectionNew = driectionxco; 
			chaoflag=1;
		}break;
		
		default: BST_fBluetoothSpeed = 0; break; 					   
	}
}

 
 
 
 
 
 
 
 
 
