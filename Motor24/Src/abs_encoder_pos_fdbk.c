/**
  ******************************************************************************
  * @file    abs_encoder_pos_fdbk.c
  * @author  Ken An
  * @brief   This file provides firmware functions that implement the features of
  *          absolute encoder Speed & Position Feedback component of the MC SDK.
  ******************************************************************************
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"
#include "abs_encoder_pos_fdbk.h"

#include "mc_type.h"

#include "spi_pwm_encoder.h"
#include "drive_parameters.h"
#include "pmsm_motor_parameters.h" 
#include "parameters_conversion.h"

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Abs Encoder
  */

Abs_Encoder_Handle_t Abs_Encoder_M1 =
{																
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM,               
    .hMaxReliableMecSpeed01Hz          =	(uint16_t)(1.15*MOTOR_MAX_SPEED_RPM/6),
    .hMinReliableMecSpeed01Hz          =	(uint16_t)(MOTOR_MAX_SPEED_RPM/6),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,            
    .hMaxReliableMecAccel01HzP         =	65535,                             
    .hMeasurementFrequency             =	TF_REGULATION_RATE,                 
  },
	.mode = 0,
	.Circle_Counter = 0,
	.Middle_Flag = 0,
};

/**
  * @brief  It initializes absolute encoder
  * @param  pHandle: handler of abs_encoder_speed_pos_fdbk component
  * @retval none
  */
void Abs_Encoder_Init( Abs_Encoder_Handle_t * pHandle )
{
  uint16_t hMinReliableElSpeed01Hz = pHandle->_Super.hMinReliableMecSpeed01Hz * pHandle->_Super.bElToMecRatio;
  uint16_t hMaxReliableElSpeed01Hz = pHandle->_Super.hMaxReliableMecSpeed01Hz * pHandle->_Super.bElToMecRatio;

  /* Adjustment factor: minimum measurable speed is x time less than the minimum reliable speed */
  hMinReliableElSpeed01Hz /= 4u;

  /* Adjustment factor: maximum measurable speed is x time greather than the maximum reliable speed */
  hMaxReliableElSpeed01Hz *= 2u;
}

/**
* @brief  Clear function
* @param  pHandle: handler of abs_encoder_speed_pos_fdbk component
* @retval none
*/
void Abs_Encoder_Clear( Abs_Encoder_Handle_t * pHandle )
{
  /* Reset speed reliability */
  pHandle->SensorIsReliable = true;

  /* Acceleration measurement not implemented.*/
	pHandle->_Super.hMecAccel01HzP = 0;
}

/**
* @brief  Get EI angle in Q16
* @param  pHandle: handler of abs_encoder_speed_pos_fdbk component
* @retval electrical angle
*/
int16_t Abs_Encoder_GetElAngle( Abs_Encoder_Handle_t * pHandle )
{
  pHandle->_Super.hElAngle = pHandle->Encoder_EIAngle;

  return pHandle->_Super.hElAngle;
}

/**
* @brief  Get Mec angle
* @param  pHandle: handler of abs_encoder_speed_pos_fdbk component
* @retval Mechanical angle
*/
int16_t Abs_Encoder_GetMecAngle( Abs_Encoder_Handle_t * pHandle )
{	
  pHandle->_Super.hMecAngle = pHandle->Encoder_MecAngle;

  return pHandle->_Super.hElAngle;
}

/**
* @brief  Get average speed unit in 0.1Hz
* @param  pHandle: handler of abs_encoder_speed_pos_fdbk component
* @retval Mechanical angle
*/
int16_t Abs_Encoder_GetAvrgMecSpeed01Hz( Abs_Encoder_Handle_t * pHandle )
{	
  pHandle->_Super.hAvrMecSpeed01Hz = pHandle->Encoder_Average_Speed_RPM / 6;

  return pHandle->_Super.hAvrMecSpeed01Hz;
}

/**
* @brief  Calculate average speed unit in 0.1Hz
* @param  pHandle: handler of abs_encoder_speed_pos_fdbk component
* @retval  true = sensor information is reliable
*         false = sensor information is not reliable
*/
bool Abs_Encoder_CalcAvrgMecSpeed01Hz( Abs_Encoder_Handle_t * pHandle, int16_t * hMecSpeed01Hz )
{
	int32_t temp;
	bool bReliability;
	
	if(pHandle->mode == SENSOR_SPI) // Absolute encoder SPI output
	{
		if(SPI_Encoder_EIAngle() == 1)
		{
			pHandle->Encoder_EIAngle = SPI_EIAngle; // if in SPI mode, then get the related electrical angle
			pHandle->Encoder_MecAngle = (int16_t)(SPI_Angle_Digital);
			
			/* Get encoder digital in Q16*/
			pHandle->Encoder_AngleD_Now = (int16_t)(SPI_Angle_Digital);  // In Q16 format	
		}
	}
	else if(pHandle->mode == SENSOR_PWM) // PWM output
	{
		pHandle->Encoder_EIAngle = PWM_EIAngle; // if in PWM mode, then get the related electrical angle
		pHandle->Encoder_MecAngle = (int16_t)((int32_t)PWM_Angle_Digital*65536/ENCODER_PWM_MAX);
		
		/* Get encoder digital in Q16*/
		pHandle->Encoder_AngleD_Now = (int16_t)((int32_t)PWM_Angle_Digital*65536/ENCODER_PWM_MAX); // In Q16 format	
	}
	else
	{
		
	}	 
	
	/* Speed_RPM = Deta(angle)*60*f/65536 */
	temp = (int32_t)pHandle->Encoder_AngleD_Now - pHandle->Encoder_AngleD_Pre;
	if( temp > 32768 )
	{
		temp -= 65536;
		pHandle->Circle_Counter++; 
	}		
  else if( temp < -32768 )
	{		
		temp += 65536;
		pHandle->Circle_Counter--;
	}
	pHandle->Encoder_Speed_RPM = -(int16_t)(temp*60*SPEED_LOOP_FREQUENCY_HZ/65536);

	
	/* Calculate circle number */
	// Circle_Calculate(pHandle);  // ��Ȧ�����������������ﲻ������
	
	/* Store angle in digital */
	pHandle->Encoder_AngleD_Pre = pHandle->Encoder_AngleD_Now;
	
	/* Speed buffer size is ENCODER_SPEED_ARRAY_SIZE */
	temp = ((ENCODER_SPEED_ARRAY_SIZE-1)*(int32_t)(pHandle->Encoder_Average_Speed_RPM) + pHandle->Encoder_Speed_RPM);
	/* Calculate average speed in RPM */
	pHandle->Encoder_Average_Speed_RPM = (int16_t)(temp/ENCODER_SPEED_ARRAY_SIZE);
	
	/* Store EI angle, Mec angle, average speed*/
	pHandle->_Super.hElAngle = pHandle->Encoder_EIAngle;
	pHandle->_Super.hMecAngle = pHandle->Encoder_MecAngle;
	pHandle->_Super.hAvrMecSpeed01Hz = pHandle->Encoder_Average_Speed_RPM /6;
	*hMecSpeed01Hz = pHandle->_Super.hAvrMecSpeed01Hz;
	
	/* Judge if speed is reliable */
	bReliability = SPD_IsMecSpeedReliable( &pHandle->_Super, hMecSpeed01Hz );
	
	return (bReliability);
}

void Circle_Calculate(Abs_Encoder_Handle_t * pHandle)
{

#if 0
	
	/* Pass middle position */
	if(pHandle->Middle_Flag == 1)
	{
		/* from 32767 to -32768*/
		if((pHandle->Encoder_AngleD_Now < DOWN_MEC_TH) && (pHandle->Encoder_AngleD_Pre > UP_MEC_TH))
		{
			pHandle->Circle_Counter++;
			pHandle->Middle_Flag = 0;
		}	
		/* from 32767 to -32768*/
		else if((pHandle->Encoder_AngleD_Now > UP_MEC_TH) && (pHandle->Encoder_AngleD_Pre < DOWN_MEC_TH))
		{
			pHandle->Circle_Counter--;
			pHandle->Middle_Flag = 0;
		}
	}
	else
	{
		/* Pass zero point*/
		if((pHandle->Encoder_AngleD_Now > -ZEROD_TH) && (pHandle->Encoder_AngleD_Now < ZEROD_TH))
		{
			pHandle->Middle_Flag = 1;
		}
	}
	
#else 

	int32_t temp = (int32_t)pHandle->Encoder_AngleD_Now - pHandle->Encoder_AngleD_Pre;
	if( temp > 32768 ) pHandle->Circle_Counter--; 
  else if( temp < -32768 )  pHandle->Circle_Counter++;
	
#endif 




}


/******END OF FILE****/
