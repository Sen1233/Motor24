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

/* D定义位置控制结构体 */
typedef struct
{	
	/* HW Settings */
  TIM_TypeDef * TIMx;   /*!< 用于编码器传感器管理的定时器 */
	
	volatile int32_t hMeasuredAngle; /* 实际转子角度 */
	volatile int32_t hTargetAngle;   /* 目标角度 */
	volatile int32_t hError_Angle;   /* 误差角度 */
	
	int16_t Control_Speed;           /* 速度控制值 */
	int16_t Real_Speed;	             /* 实际平均速度 */
	uint16_t Duration_ms;            /* 持续时间, 速度斜坡, 用于速度加速 */
	int16_t Set_Speed;               /* 位置控制设置速度 */

	uint16_t Position_Gain;          /* 位置Kp增益 */
	uint16_t Postiion_Div;           /* 位置Kp除法器 */
	
	uint16_t Speed_ACC_Gain;         /* 速度加速Kp增益 */
	uint16_t Speed_ACC_Div;          /* 速度加速Kp分配器 */
	uint16_t Speed_ACC_Count;        /* 速度加速Kp周期时间 */
	
	int16_t Encoder_Update_Count;    /* 编码器定时器更新计数 */ /* 没用到 */
	bool Encoder_Direction;          /* 编码器运行方向 */
	bool Encoder_Pre_Direction;      /* 编码器预运行方向 */
	
	bool Mode_Flag;                  /* 转矩模式下Mode_Flag = 1，速度模式下Mode_Flag = 0 */
	bool Torque_First_Flag;          /* 第一个扭矩模式标志 */
} Position_Handle_t;

/* Define mode */
#define P_TORQUE_MODE 1
#define P_SPEED_MODE  0

/* 执行ACC持续时间计算速率:(速度循环时间* SPEED_ACC_RATE) */
#define SPEED_ACC_RATE 20

#define _IQ24(A) (long) ((A) * 16777216.0L)
#define IQ24_PI 52707178 // 3.14159265358979323846 * 16777216

/* PI_MUL = 2*PI*10000 */
#define PI_MUL 62832

/* 定义低 dt 角和高 dt 角,用于控制模式的改变 */
#define CHANGE_LIMIT_LOW  628320
#define CHANGE_LIMIT_HIGH 628320*2//62832*2

int32_t Position_GetErrorAngle(Abs_Encoder_Handle_t * pHandle, int32_t angle_ref);
int16_t Position_CalcSpeedReferrence(void);
int16_t Position_CalcTorqueReferrence(void);

uint32_t Abs_Value(int32_t dat);

extern PID_Handle_t PIDAngleHandle_M1;
extern Position_Handle_t Position_M1;

/******END OF FILE****/
