/**
  ******************************************************************************
  * @file    mc_position.h
  * @author  Ken An
  * @brief   This file provides firmware functions for position control
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pmsm_motor_parameters.h"
#include "pid_regulator.h"
#include "abs_encoder_pos_fdbk.h"

/* Define positon control structure */
typedef struct
{	
	/* HW Settings */
  TIM_TypeDef * TIMx;   /*!< Timer used for ENCODER sensor management.*/
	
	volatile int32_t hMeasuredAngle; /* Real rotor Angle */
	volatile int32_t hTargetAngle;   /* Target angle */
	volatile int32_t hError_Angle;   /* Error angle */
	
	int16_t Control_Speed;           /* Speed control value */
	int16_t Real_Speed;	             /* Real average speed */
	uint16_t Duration_ms;            /* Speed ramp, used for speed accelarate */
	int16_t Set_Speed;               /* Position control set speed */

	uint16_t Position_Gain;          /* Position Kp gain */
	uint16_t Postiion_Div;           /* Position Kp divider */
	
	uint16_t Speed_ACC_Gain;         /* Speed accelarate Kp gain */
	uint16_t Speed_ACC_Div;          /* Speed accelarate Kp divider */
	uint16_t Speed_ACC_Count;        /* Speed accelarate Kp period time */
	
	int16_t Encoder_Update_Count;    /* Encoder timer update count */ /* 没用到 */
	bool Encoder_Direction;          /* Encoder runing direction */
	bool Encoder_Pre_Direction;      /* Encoder pre runing direction*/
	
	bool Mode_Flag;                  /* Mode_Flag = 1, under torque mode, Mode_Flag = 0,under speed mode */
	bool Torque_First_Flag;          /* First torque mode flag */
} Position_Handle_t;

/* Define mode */
#define P_TORQUE_MODE 1
#define P_SPEED_MODE  0

/* Do ACC duration calculate rate: (Speed loop time * SPEED_ACC_RATE) */
#define SPEED_ACC_RATE 20

#define _IQ24(A) (long) ((A) * 16777216.0L)
#define IQ24_PI 52707178 // 3.14159265358979323846 * 16777216

/* PI_MUL = 2*PI*10000 */
#define PI_MUL 62832

/* Define low delta angle and high delta angle, used for control mode change */
#define CHANGE_LIMIT_LOW  628320
#define CHANGE_LIMIT_HIGH 628320*2//62832*2

int32_t Position_GetErrorAngle(Abs_Encoder_Handle_t * pHandle, int32_t angle_ref);
int16_t Position_CalcSpeedReferrence(void);
int16_t Position_CalcTorqueReferrence(void);

uint32_t Abs_Value(int32_t dat);

extern PID_Handle_t PIDAngleHandle_M1;
extern Position_Handle_t Position_M1;

/******END OF FILE****/
