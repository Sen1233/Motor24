#include "spi_pwm_encoder.h"
#include "tle5012b.h"

volatile uint32_t foccnt = 0;
uint16_t offset_ia;
uint16_t offset_ib;

uint16_t SPI_Angle_Digital;  /* Angle read from encoder chip AS5048A by SPI */
int16_t  SPI_EIAngle;        /* Electrical angle in SPI mode*/


uint32_t PWM_Angle_Digital; /* Angle read from encoder chip AS5048A by PWM */
int16_t  PWM_EIAngle;       /* Electrical angle in SPI mode*/



/**
  * @brief Read data from AS5048A
  * @param Command
  * @retval Data
  */
uint16_t SPI_Read5048Data(uint16_t TxData) 
{ 
	uint16_t data;
	/* Cs low start SPI communication */
	HAL_GPIO_WritePin(CS_PORT, GPIO_PIN_10,GPIO_PIN_RESET);
	/* Write TxData, then return read data */
	data = SPI_ReadWriteData(TxData);
	/* Cs high to end SPI communication */
	HAL_GPIO_WritePin(CS_PORT, GPIO_PIN_10,GPIO_PIN_SET);
	return data; 
} 

/**
  * @brief Get digital angle value, get electrical angle
  * @param None
  * @retval Angle read and calculate right-1,wrong-0
  */
uint16_t SPI_Encoder_EIAngle(void)
{
	
#if 0
	
	/* First send angle read command-0xFFFF*/
	SPI_Read5048Data(CMD_ANGLE);
	/* Second send dummy 0x0000 to get the digital angle */
	SPI_Angle_Digital = SPI_Read5048Data(0x0000);
	/* Judge if the data is right */
	if (SPI_Angle_Digital & 0x4000)
	{
		/* If wrong then clear adn nop*/
		ClearAndNop(); 
		return 0;
	}
	else
	{
		/* 14-bit digital angle data 0~0x3FFF*/
		SPI_Angle_Digital &= 0x3FFF;
		/* 
		 * Calculate electrical angle based on motor, data in Q16 format 
		 * 16384 stand for 360degree(65536)*motor poles
		*/
		SPI_EIAngle = (int16_t)((65536*SPI_Angle_Digital*POLES)/ENCODER_SPI_MAX)%65536;
		
		return 1;
	} 	

#else

	SPI_Angle_Digital = ReadAngle();
	SPI_EIAngle = SPI_Angle_Digital*POLES; 
	
	return 1;

#endif
	
	
}

/**
  * @brief PWM mode: Get digital angle value, get electrical angle
  * @param None
  * @retval none
  */
void PWM_Encoder_EIangle(uint32_t data)
{
	PWM_Angle_Digital = data;
	if(PWM_Angle_Digital > ENCODER_PWM_MAX)
		PWM_Angle_Digital = ENCODER_PWM_MAX;
	PWM_EIAngle = (int16_t)((65536*PWM_Angle_Digital*POLES)/ENCODER_PWM_MAX)%65536;
}

/**
  * @brief AS5048A clear and nop
  * @param None
  * @retval None
  */
void ClearAndNop(void) 
{
	
	/* First send clear then nop command */
	HAL_GPIO_WritePin(CS_PORT, GPIO_PIN_10,GPIO_PIN_RESET);
	SPI_Read5048Data(CMD_CLAER); 
	HAL_GPIO_WritePin(CS_PORT, GPIO_PIN_10,GPIO_PIN_SET);
	/* Delay 10ms */
	HAL_Delay(10); 
	HAL_GPIO_WritePin(CS_PORT, GPIO_PIN_10,GPIO_PIN_RESET);
	SPI_Read5048Data(CMD_NOP);
	HAL_GPIO_WritePin(CS_PORT, GPIO_PIN_10,GPIO_PIN_SET); 
	
} 

