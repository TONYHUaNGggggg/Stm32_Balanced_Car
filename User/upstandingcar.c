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
	enAutoRunFixed,
	enAutoRunTask
} enAutoRunMode;

typedef struct {
	float fBaseSpeedCmd;
	float fTurnAmplCmd;
	float fOmegaRad;
	float fStartPhaseRad;
	float fSpeedModRatio;
} stFigureEightParam;

typedef enum {
	enMotionTaskIdle = 0,
	enMotionTaskStraight,
	enMotionTaskArc,
	enMotionTaskTurn,
	enMotionTaskSpin
} enMotionTaskType;

typedef struct {
	enMotionTaskType enType;
	float fTargetSpeedCmd;
	float fTargetTurnCmd;
	float fCurrentSpeedCmd;
	float fAccelStepPerTick;
	u16 u16DurationTick;
	u16 u16ElapsedTick;
	u8 u8Active;
} stMotionTask;

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
static float g_fObsRightAvoidSpeedCmd = ULTRA_OBS_RIGHT_AVOID_SPEED_CMD;
static float g_fObsRightAvoidTurnCmd = ULTRA_OBS_RIGHT_AVOID_TURN_CMD;
static stMotionTask g_stMotionTask = {
	enMotionTaskIdle,
	0.0f,
	0.0f,
	0.0f,
	MOTION_DEFAULT_ACCEL_STEP,
	MOTION_TASK_DURATION_MIN_TICK,
	0,
	0
};
static u16 g_u16EightArcDurationTick = EIGHT_ARC_DURATION_TICK;
static float g_fEightArcAccelStep = EIGHT_ARC_ACCEL_STEP;
static u8 g_u8EightArcStartRight = EIGHT_ARC_START_RIGHT;
static u8 g_u8EightArcIndex = 0;
static u8 g_u8TaskStraightObsEnable = 0;
static u8 g_u8ObsConfirmCnt = 0;
static u8 g_u8ObsStopConfirmCnt = 0;
static u8 BST_u8StraightEventLast = 0;
static float g_fSpeedErrIntegral = 0.0f;
static float g_fAngleRefFiltered = 0.0f;


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


float  BST_fCarAngle_P = 150.0;
float  BST_fCarAngle_D = 0.28;

float  BST_fCarSpeed_P = 4.5;
float  BST_fCarSpeed_I = 0.08;

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
static void MotionTaskControl(void);
static float MotionClampFloat(float fValue, float fMin, float fMax);
static void MotionTaskStartInternal(enMotionTaskType enType, float fSpeedCmd, float fTurnCmd, float fAccelStep, u16 u16DurationTick, u8 u8EnterTaskMode);
static u8 MotionTaskRunStep(u8 u8DeltaTick, float *pfSpeedCmd, float *pfTurnCmd);
static void MotionApplyObstacleAvoid(float *pfSpeedCmd, float *pfDirectionCmd);


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
	g_u16EightArcDurationTick = EIGHT_ARC_DURATION_TICK;
	g_fEightArcAccelStep = EIGHT_ARC_ACCEL_STEP;
	g_u8EightArcStartRight = EIGHT_ARC_START_RIGHT;
	g_u8EightArcIndex = 0;
	g_u8TaskStraightObsEnable = 0;
	g_fSpeedErrIntegral = 0.0f;
	g_fAngleRefFiltered = 0.0f;
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
	g_fSpeedErrIntegral = 0.0f;
	g_fAngleRefFiltered = 0.0f;

}	

void SpeedControl_ResetStopState(void)
{
	g_fSpeedErrIntegral = 0.0f;
	g_fAngleRefFiltered = 0.0f;
	BST_fSpeedControlOutNew = 0.0f;
	BST_fAngleRef = 0.0f;
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
	float fLeftBaseBias;
	float fRightBaseBias;
			
	fDirectionCmd = BST_fBluetoothDirectionNew;

	fLeftBaseBias = 0.0f;
	fRightBaseBias = 0.0f;
#if WHEEL_BASE_BIAS_ENABLE
	if(stopflag == 0
		&& fabsf(BST_fBluetoothSpeed) >= WHEEL_BASE_BIAS_ACTIVE_SPEED_CMD
		&& fabsf(BST_fBluetoothDirectionNew) <= WHEEL_BASE_BIAS_STRAIGHT_DIR_MAX)
	{
		fLeftBaseBias = WHEEL_LEFT_BASE_BIAS;
		fRightBaseBias = WHEEL_RIGHT_BASE_BIAS;
	}
#endif

	BST_fLeftMotorOut  = BST_fAngleControlOut + fDirectionCmd + fLeftBaseBias;
    BST_fRightMotorOut = BST_fAngleControlOut - fDirectionCmd + fRightBaseBias;

		
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
	float fSpeedCmdCtrl;
	float fSpeedErr;
	float fIntegralStep;
	float fAbsCmd;
	float fAbsSpeed;
	float fAngleRefTarget;
	float fAngleRefDelta;
  
 
	BST_fCarSpeed = (BST_s32LeftMotorPulseSigma  + BST_s32RightMotorPulseSigma );
	BST_s32LeftMotorPulseSigma =BST_s32RightMotorPulseSigma = 0;	  
	BST_fCarSpeedOld *= 0.7;
	BST_fCarSpeedOld +=BST_fCarSpeed*0.3;

	fSpeedCmdCtrl = BST_fBluetoothSpeed * SPEED_CMD_POLARITY;
	fSpeedErr = fSpeedCmdCtrl - BST_fCarSpeedOld;
	fAbsCmd = fabsf(fSpeedCmdCtrl);
	fAbsSpeed = fabsf(BST_fCarSpeedOld);

	fIntegralStep = fSpeedErr;
	if(g_fSpeedErrIntegral * fSpeedErr < 0.0f)
	{
		fIntegralStep *= SPEED_I_UNWIND_GAIN;
	}
	g_fSpeedErrIntegral += fIntegralStep;

	if(fAbsCmd <= SPEED_STOP_CMD_DEADBAND)
	{
		g_fSpeedErrIntegral *= SPEED_I_BLEED_WHEN_STOP;
		if(fAbsSpeed <= SPEED_STOP_ACTUAL_DEADBAND)
		{
			g_fSpeedErrIntegral *= SPEED_I_BLEED_WHEN_STOP;
		}
	}

	if(g_fSpeedErrIntegral > SPEED_ERR_I_MAX) g_fSpeedErrIntegral = SPEED_ERR_I_MAX;
	if(g_fSpeedErrIntegral < SPEED_ERR_I_MIN) g_fSpeedErrIntegral = SPEED_ERR_I_MIN;
	
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
	

																								  
	BST_fSpeedControlOutNew = -(fSpeedErr * BST_fCarSpeed_P + g_fSpeedErrIntegral * BST_fCarSpeed_I);

	if(BST_fCarAngle_P > 0.001f || BST_fCarAngle_P < -0.001f)
	{
		fAngleRefTarget = -BST_fSpeedControlOutNew / BST_fCarAngle_P;
		if(fAbsCmd <= SPEED_STOP_CMD_DEADBAND)
		{
			if(fAngleRefTarget > SPEED_STOP_ANGLE_REF_MAX) fAngleRefTarget = SPEED_STOP_ANGLE_REF_MAX;
			if(fAngleRefTarget < -SPEED_STOP_ANGLE_REF_MAX) fAngleRefTarget = -SPEED_STOP_ANGLE_REF_MAX;
		}

		if(fAngleRefTarget > CAR_ANGLE_REF_MAX) fAngleRefTarget = CAR_ANGLE_REF_MAX;
		if(fAngleRefTarget < CAR_ANGLE_REF_MIN) fAngleRefTarget = CAR_ANGLE_REF_MIN;

		fAngleRefDelta = fAngleRefTarget - g_fAngleRefFiltered;
		if(fAngleRefDelta > ANGLE_REF_SLEW_STEP) fAngleRefDelta = ANGLE_REF_SLEW_STEP;
		if(fAngleRefDelta < -ANGLE_REF_SLEW_STEP) fAngleRefDelta = -ANGLE_REF_SLEW_STEP;
		g_fAngleRefFiltered += fAngleRefDelta;

		if(fAbsCmd <= SPEED_STOP_CMD_DEADBAND && fAbsSpeed <= SPEED_STOP_ACTUAL_DEADBAND)
		{
			g_fAngleRefFiltered *= 0.85f;
			if(g_fAngleRefFiltered > -0.05f && g_fAngleRefFiltered < 0.05f)
			{
				g_fAngleRefFiltered = 0.0f;
			}
		}

		BST_fAngleRef = g_fAngleRefFiltered;
	}
	else
	{
		BST_fAngleRef = 0;
		g_fAngleRefFiltered = 0.0f;
	}

}






void chaoshengbo(void)
{  
	if(chaoflag==0)
	{
	
      	juli=TIM_GetCounter(TIM1)*5*34/200.0;
		printf("juli=%f\n",juli);
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

static float MotionClampFloat(float fValue, float fMin, float fMax)
{
	if(fValue > fMax)
	{
		return fMax;
	}
	if(fValue < fMin)
	{
		return fMin;
	}
	return fValue;
}

static void MotionTaskStartInternal(enMotionTaskType enType, float fSpeedCmd, float fTurnCmd, float fAccelStep, u16 u16DurationTick, u8 u8EnterTaskMode)
{
	fSpeedCmd = MotionClampFloat(fSpeedCmd, MOTION_SPEED_CMD_MIN, MOTION_SPEED_CMD_MAX);
	fTurnCmd = MotionClampFloat(fTurnCmd, MOTION_TURN_CMD_MIN, MOTION_TURN_CMD_MAX);

	if(fAccelStep < 0.0f)
	{
		fAccelStep = -fAccelStep;
	}
	fAccelStep = MotionClampFloat(fAccelStep, MOTION_ACCEL_STEP_MIN, MOTION_ACCEL_STEP_MAX);

	if(u16DurationTick < MOTION_TASK_DURATION_MIN_TICK)
	{
		u16DurationTick = MOTION_TASK_DURATION_MIN_TICK;
	}
	if(u16DurationTick > MOTION_TASK_DURATION_MAX_TICK)
	{
		u16DurationTick = MOTION_TASK_DURATION_MAX_TICK;
	}

	if(enType == enMotionTaskStraight)
	{
		fTurnCmd = 0.0f;
	}
	if(enType == enMotionTaskSpin)
	{
		fSpeedCmd = 0.0f;
	}

	g_stMotionTask.enType = enType;
	g_stMotionTask.fTargetSpeedCmd = fSpeedCmd;
	g_stMotionTask.fTargetTurnCmd = fTurnCmd;
	g_stMotionTask.fCurrentSpeedCmd = BST_fBluetoothSpeed;
	g_stMotionTask.fAccelStepPerTick = fAccelStep;
	g_stMotionTask.u16DurationTick = u16DurationTick;
	g_stMotionTask.u16ElapsedTick = 0;
	g_stMotionTask.u8Active = 1;

	if(u8EnterTaskMode != 0)
	{
		g_enAutoRunMode = enAutoRunTask;
		BST_u8MainEventLast = BST_u8MainEventCount;
	}
}

static u8 MotionTaskRunStep(u8 u8DeltaTick, float *pfSpeedCmd, float *pfTurnCmd)
{
	u8 u8Step;
	float fDeltaSpeed;

	if(g_stMotionTask.u8Active == 0)
	{
		*pfSpeedCmd = 0.0f;
		*pfTurnCmd = 0.0f;
		return 0;
	}

	for(u8Step = 0; u8Step < u8DeltaTick; u8Step++)
	{
		fDeltaSpeed = g_stMotionTask.fTargetSpeedCmd - g_stMotionTask.fCurrentSpeedCmd;
		if(fabsf(fDeltaSpeed) <= g_stMotionTask.fAccelStepPerTick)
		{
			g_stMotionTask.fCurrentSpeedCmd = g_stMotionTask.fTargetSpeedCmd;
		}
		else if(fDeltaSpeed > 0.0f)
		{
			g_stMotionTask.fCurrentSpeedCmd += g_stMotionTask.fAccelStepPerTick;
		}
		else
		{
			g_stMotionTask.fCurrentSpeedCmd -= g_stMotionTask.fAccelStepPerTick;
		}

		if(g_stMotionTask.u16ElapsedTick < g_stMotionTask.u16DurationTick)
		{
			g_stMotionTask.u16ElapsedTick++;
		}
		if(g_stMotionTask.u16ElapsedTick >= g_stMotionTask.u16DurationTick)
		{
			g_stMotionTask.u8Active = 0;
			break;
		}
	}

	*pfSpeedCmd = g_stMotionTask.fCurrentSpeedCmd;
	*pfTurnCmd = g_stMotionTask.fTargetTurnCmd;

	if(g_stMotionTask.u8Active == 0)
	{
		return 2;
	}
	return 1;
}

void MotionTask_SetStraight(float targetSpeedCmd, float accelStep, u16 durationTick)
{
	g_u8TaskStraightObsEnable = 0;
	MotionTaskStartInternal(enMotionTaskStraight, targetSpeedCmd, 0.0f, accelStep, durationTick, 1);
}

void MotionTask_SetStraightWithObs(float targetSpeedCmd, float accelStep, u16 durationTick, u8 obsEnable)
{
	g_u8TaskStraightObsEnable = (obsEnable == 0u) ? 0u : 1u;
	g_u8ObsConfirmCnt = 0;
	g_u8ObsStopConfirmCnt = 0;
	MotionTaskStartInternal(enMotionTaskStraight, targetSpeedCmd, 0.0f, accelStep, durationTick, 1);
}

void MotionTask_SetStraightObsAvoidEnable(u8 obsEnable)
{
	g_u8TaskStraightObsEnable = (obsEnable == 0u) ? 0u : 1u;
	g_u8ObsConfirmCnt = 0;
	g_u8ObsStopConfirmCnt = 0;
}

void MotionTask_SetArc(float speedCmd, float turnCmd, u16 durationTick)
{
	MotionTaskStartInternal(enMotionTaskArc, speedCmd, turnCmd, MOTION_DEFAULT_ACCEL_STEP, durationTick, 1);
}

void MotionTask_SetTurn(float speedCmd, float turnCmd, u16 durationTick)
{
	MotionTaskStartInternal(enMotionTaskTurn, speedCmd, turnCmd, MOTION_DEFAULT_ACCEL_STEP, durationTick, 1);
}

void MotionTask_SetSpin(float turnCmd, u16 durationTick)
{
	MotionTaskStartInternal(enMotionTaskSpin, 0.0f, turnCmd, MOTION_DEFAULT_ACCEL_STEP, durationTick, 1);
}

void MotionTask_Stop(void)
{
	g_stMotionTask.u8Active = 0;
	g_stMotionTask.enType = enMotionTaskIdle;
	g_stMotionTask.fTargetSpeedCmd = 0.0f;
	g_stMotionTask.fTargetTurnCmd = 0.0f;
	g_stMotionTask.fCurrentSpeedCmd = BST_fBluetoothSpeed;
	g_stMotionTask.u16ElapsedTick = 0;
}

void AutoRun_SetStraight(float speedCmd)
{
	g_enAutoRunMode = enAutoRunStraight;
	g_fStraightSpeedCmd = speedCmd;
	MotionTask_Stop();
	g_u8ObsConfirmCnt = 0;
	g_u8ObsStopConfirmCnt = 0;
	BST_u8MainEventLast = BST_u8MainEventCount;
	BST_u32EightTick = 0;
	BST_u8StraightEventLast = BST_u8MainEventCount;
}

void AutoRun_SetObsRightAvoid(float avoidSpeedCmd, float avoidTurnCmd)
{
	if(avoidSpeedCmd < 0.0f)
	{
		avoidSpeedCmd = -avoidSpeedCmd;
	}
	if(avoidTurnCmd < 0.0f)
	{
		avoidTurnCmd = -avoidTurnCmd;
	}
	g_fObsRightAvoidSpeedCmd = avoidSpeedCmd;
	g_fObsRightAvoidTurnCmd = avoidTurnCmd;
}

void AutoRun_SetFigureEightArc(float arcSpeedCmd, float arcTurnCmd, u16 arcDurationTick, float accelStep, u8 startRightArc)
{
	arcSpeedCmd = fabsf(arcSpeedCmd);
	arcTurnCmd = fabsf(arcTurnCmd);

	g_stFigureEightParam.fBaseSpeedCmd = MotionClampFloat(arcSpeedCmd, 0.0f, MOTION_SPEED_CMD_MAX);
	g_stFigureEightParam.fTurnAmplCmd = MotionClampFloat(arcTurnCmd, 0.0f, MOTION_TURN_CMD_MAX);
	g_u16EightArcDurationTick = arcDurationTick;
	if(g_u16EightArcDurationTick < MOTION_TASK_DURATION_MIN_TICK)
	{
		g_u16EightArcDurationTick = MOTION_TASK_DURATION_MIN_TICK;
	}
	if(g_u16EightArcDurationTick > MOTION_TASK_DURATION_MAX_TICK)
	{
		g_u16EightArcDurationTick = MOTION_TASK_DURATION_MAX_TICK;
	}
	g_fEightArcAccelStep = MotionClampFloat(fabsf(accelStep), MOTION_ACCEL_STEP_MIN, MOTION_ACCEL_STEP_MAX);
	g_u8EightArcStartRight = (startRightArc == 0u) ? 0u : 1u;
	g_u8EightArcIndex = 0;

	g_stFigureEightParam.fOmegaRad = 6.2831853f / ((float)g_u16EightArcDurationTick * EIGHT_CTRL_DT_SEC);
	g_stFigureEightParam.fStartPhaseRad = (g_u8EightArcStartRight != 0u) ? 1.5707963f : -1.5707963f;
	g_stFigureEightParam.fSpeedModRatio = 0.0f;

	MotionTask_Stop();
	g_enAutoRunMode = enAutoRunFigureEight;
	BST_u8MainEventLast = BST_u8MainEventCount;
	BST_u32EightTick = 0;
}

void AutoRun_SetFigureEight(float baseSpeedCmd, float turnAmplCmd, float omegaRad, float startPhaseRad, float speedModRatio)
{
	float fArcTickFloat;
	u16 u16ArcTick;
	float fAccelStep;
	u8 u8StartRight;

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

	fArcTickFloat = (6.2831853f / omegaRad) / EIGHT_CTRL_DT_SEC;
	if(fArcTickFloat < (float)MOTION_TASK_DURATION_MIN_TICK)
	{
		fArcTickFloat = (float)MOTION_TASK_DURATION_MIN_TICK;
	}
	if(fArcTickFloat > (float)MOTION_TASK_DURATION_MAX_TICK)
	{
		fArcTickFloat = (float)MOTION_TASK_DURATION_MAX_TICK;
	}
	u16ArcTick = (u16)(fArcTickFloat + 0.5f);
	fAccelStep = MOTION_DEFAULT_ACCEL_STEP + speedModRatio * 1.2f;
	u8StartRight = ((float)sin(startPhaseRad) >= 0.0f) ? 1u : 0u;

	AutoRun_SetFigureEightArc(baseSpeedCmd, turnAmplCmd, u16ArcTick, fAccelStep, u8StartRight);
}

void AutoRun_SetFixed(float speedCmd, float turnCmd)
{
	g_enAutoRunMode = enAutoRunFixed;
	MotionTask_Stop();
	g_fFixedSpeedCmd = speedCmd;
	g_fFixedTurnCmd = turnCmd;
	BST_u8MainEventLast = BST_u8MainEventCount;
	BST_u32EightTick = 0;
}

static void StraightAvoidControl(void)
{
	float fSpeedCmd;
	float fDirectionCmd;

	fSpeedCmd = g_fStraightSpeedCmd;
	fDirectionCmd = 0.0f;
	MotionApplyObstacleAvoid(&fSpeedCmd, &fDirectionCmd);

	BST_fBluetoothSpeed = fSpeedCmd;
	BST_fBluetoothDirectionNew = fDirectionCmd;
	chaoflag = 0;
	fchaoshengbo = 0;
}

static void MotionApplyObstacleAvoid(float *pfSpeedCmd, float *pfDirectionCmd)
{
	float fDirectionLimit;
	float fObsSlowCm;
	float fObsCm;
	float fSpeedCmd;
	float fDirectionCmd;

	fSpeedCmd = *pfSpeedCmd;
	fDirectionCmd = *pfDirectionCmd;
	fObsSlowCm = ULTRA_OBS_SLOW_CM;
	if(fObsSlowCm <= (ULTRA_OBS_STOP_CM + 1.0f))
	{
		fObsSlowCm = ULTRA_OBS_STOP_CM + 8.0f;
	}
	fObsCm = juli;

	if(fObsCm >= ULTRA_OBS_VALID_MIN_CM && fObsCm <= ULTRA_OBS_STOP_CM)
	{
		g_u8ObsConfirmCnt = 0;
		if(g_u8ObsStopConfirmCnt < 255u)
		{
			g_u8ObsStopConfirmCnt++;
		}

		if(g_u8ObsStopConfirmCnt >= ULTRA_OBS_STOP_CONFIRM_CNT)
		{
			fSpeedCmd = 0.0f;
			fDirectionCmd = 0.0f;
		}
	}
	else if(fObsCm > ULTRA_OBS_STOP_CM && fObsCm <= fObsSlowCm)
	{
		g_u8ObsStopConfirmCnt = 0;
		if(g_u8ObsConfirmCnt < 255u)
		{
			g_u8ObsConfirmCnt++;
		}

		if(g_u8ObsConfirmCnt >= ULTRA_OBS_CONFIRM_CNT)
		{
			fSpeedCmd = g_fObsRightAvoidSpeedCmd;
			// 右避障转向幅度在此处生效。
			fDirectionCmd = g_fObsRightAvoidTurnCmd;
			fDirectionLimit = fabsf(fSpeedCmd) * ULTRA_OBS_RIGHT_TURN_SPEED_RATIO;
			if(fDirectionCmd > fDirectionLimit)
			{
				fDirectionCmd = fDirectionLimit;
			}
		}
	}
	else
	{
		g_u8ObsConfirmCnt = 0;
		g_u8ObsStopConfirmCnt = 0;
	}

	*pfSpeedCmd = fSpeedCmd;
	*pfDirectionCmd = fDirectionCmd;
}

static void FigureEightControl(void)
{
	u8 u8MainEventNow;
	u8 u8DeltaEvent;
	float fArcTurnCmd;
	float fSpeedCmd;
	float fTurnCmd;
	u8 u8RunState;

  	u8MainEventNow = BST_u8MainEventCount;
	u8DeltaEvent = (u8)(u8MainEventNow - BST_u8MainEventLast);
	BST_u8MainEventLast = u8MainEventNow;

	if(g_stMotionTask.u8Active == 0u)
	{
		if((g_u8EightArcIndex & 0x01u) == 0u)
		{
			fArcTurnCmd = (g_u8EightArcStartRight != 0u) ? g_stFigureEightParam.fTurnAmplCmd : (-g_stFigureEightParam.fTurnAmplCmd);
		}
		else
		{
			fArcTurnCmd = (g_u8EightArcStartRight != 0u) ? (-g_stFigureEightParam.fTurnAmplCmd) : g_stFigureEightParam.fTurnAmplCmd;
		}

		MotionTaskStartInternal(
			enMotionTaskArc,
			g_stFigureEightParam.fBaseSpeedCmd,
			fArcTurnCmd,
			g_fEightArcAccelStep,
			g_u16EightArcDurationTick,
			0
		);
	}

	u8RunState = MotionTaskRunStep(u8DeltaEvent, &fSpeedCmd, &fTurnCmd);
	if(u8RunState == 2u)
	{
		g_u8EightArcIndex ^= 1u;
	}

	BST_fBluetoothSpeed = fSpeedCmd;
	BST_fBluetoothDirectionNew = fTurnCmd;
	chaoflag = 1;
	fchaoshengbo = 0;
}

static void MotionTaskControl(void)
{
	u8 u8MainEventNow;
	u8 u8DeltaEvent;
	float fSpeedCmd;
	float fTurnCmd;
	u8 u8RunState;

	u8MainEventNow = BST_u8MainEventCount;
	u8DeltaEvent = (u8)(u8MainEventNow - BST_u8MainEventLast);
	BST_u8MainEventLast = u8MainEventNow;

	u8RunState = MotionTaskRunStep(u8DeltaEvent, &fSpeedCmd, &fTurnCmd);
	if(u8RunState == 0u)
	{
		fSpeedCmd = 0.0f;
		fTurnCmd = 0.0f;
	}

	if(g_stMotionTask.enType == enMotionTaskStraight && g_u8TaskStraightObsEnable != 0u)
	{
		MotionApplyObstacleAvoid(&fSpeedCmd, &fTurnCmd);
	}
	else
	{
		g_u8ObsConfirmCnt = 0;
		g_u8ObsStopConfirmCnt = 0;
	}

	BST_fBluetoothSpeed = fSpeedCmd;
	BST_fBluetoothDirectionNew = fTurnCmd;
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

		case enAutoRunTask:
			MotionTaskControl();
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

 
 
 
 
 
 
 
 
 
