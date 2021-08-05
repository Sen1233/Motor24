/**
  ******************************************************************************
  * @file    abs_encoder_pos_fdbk.h
  * @author  Ken An
  * @brief   This file provides firmware functions that implement the features of
  *          absolute encoder Speed & Position Feedback component of the MC SDK.
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ABS_ENCODER_POS_FDBK_H
#define __ABS_ENCODER_POS_FDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"


/* Exported types ------------------------------------------------------------*/
#define ZEROD_TH 10000
#define UP_MEC_TH    (int16_t)30000
#define DOWN_MEC_TH  (int16_t)(-30000)


/**
  * @brief Abs encoder component parameters definition
  */

typedef struct
{
  SpeednPosFdbk_Handle_t _Super;
	
	int16_t Encoder_EIAngle; /* Encoder final electrical angle */
	int16_t Encoder_MecAngle; /* Encoder final mechanical angle */

	int16_t Encoder_AngleD_Pre; /* Encoder previous digital angle */
	int16_t Encoder_AngleD_Now; /* Encoder present/now digital angle */
	int16_t Encoder_Speed_RPM; /* Speed uint in RPM */
	int16_t Encoder_Average_Speed_RPM; /* Average speed unit in RPM*/
  
  bool SensorIsReliable;
  uint8_t mode;
	
	int32_t Circle_Counter; /* Count the circle for motro run */
	bool Middle_Flag; /* Middle flag, 1--arrived pass middle*/

} Abs_Encoder_Handle_t;

extern Abs_Encoder_Handle_t Abs_Encoder_M1;
/**
  * @}
  */


void Abs_Encoder_Init( Abs_Encoder_Handle_t * pHandle );
void Abs_Encoder_Clear( Abs_Encoder_Handle_t * pHandle );
int16_t Abs_Encoder_GetElAngle( Abs_Encoder_Handle_t * pHandle );
int16_t Abs_Encoder_GetMecAngle( Abs_Encoder_Handle_t * pHandle );
int16_t Abs_Encoder_GetAvrgMecSpeed01Hz( Abs_Encoder_Handle_t * pHandle );
bool Abs_Encoder_CalcAvrgMecSpeed01Hz( Abs_Encoder_Handle_t * pHandle, int16_t * hMecSpeed01Hz );
void Circle_Calculate(Abs_Encoder_Handle_t * pHandle);
/**
  * @}
  */

/**
  * @}
  */

/** @} */

#endif /*__ABS_ENCODER_POS_FDBK_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
