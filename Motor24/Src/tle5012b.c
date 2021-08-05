#include "main.h"
#include "tle5012b.h"

uint16_t offset = 0;

void SPI5012B_Init(void)
{
	
// 可以把MOSI变成 开漏输出模式试试
//  这些代码放在 HAL_SPI_MspInit 中了	
//	GPIO_InitStruct.Pin = GPIO_PIN_7;
//	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	SPI1->CR1 |= SPI_CR1_SPE;
	
}


void CalibTle5012b( void )
{
	SPI5012B_Init();
	HAL_GPIO_WritePin(VCC_ON_GPIO_Port,VCC_ON_Pin,GPIO_PIN_SET);  // 开驱动电源
	
	
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 100;
	TIM1->CCR3 = 0;
	TIM1->CCER = 0x0555;
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CR1  |= TIM_CR1_CEN;
	
	HAL_Delay(300);
	
	SPI_CS_GPIO_Port->ODR &= ~SPI_CS_Pin;
	SPI_ReadWriteData(READ_ANGLE_VALUE);
	offset = SPI_ReadWriteData(0xffff)<<1;
	SPI_CS_GPIO_Port->ODR |= SPI_CS_Pin;
	
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	
//	TIM1->BDTR &= ~TIM_BDTR_MOE;
//	TIM1->CR1  &= ~TIM_CR1_CEN;

}




uint16_t SPI_ReadWriteData(uint16_t byte)
{
	uint16_t retry = 0;
	
	while( (SPI1->SR&1<<1) == 0 ) // 发送缓冲区非空
	{
		if( ++retry > 200 )
		return 0;                   // 延迟一段时间后返回
	}
	
	SPI1->DR = byte;              // 发送数据
	retry = 0;
	while( (SPI1->SR&1<<0) == 0 ) // 接收缓冲区为空
	{
		if( ++retry > 200 )
		return 0;										// 延迟一段时间后返回
	}
	return SPI1->DR;              // 读一下缓冲区，清标志
	
}


// 得到 0~359 度
uint16_t ReadAngle(void)
{
	// return ( ReadValue(READ_ANGLE_VALUE) * 360 / 0x10000 );
	return ReadValue(READ_ANGLE_VALUE);
}
 
// 得到角速度
uint16_t ReadSpeed(void)
{
	return ReadValue(READ_SPEED_VALUE);
}
 
 

uint16_t ReadValue(uint16_t u16RegValue)
{
	uint16_t u16Data;
	
//	uint16_t txbuf[2];
//	uint16_t rxbuf[2];
	
//	txbuf[0] = u16RegValue;
//	txbuf[1] = 0xffff;
 
	SPI_CS_GPIO_Port->ODR &= ~SPI_CS_Pin;

	// HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)&txbuf,(uint8_t *)&rxbuf,2,2);
	// u16Data = rxbuf[1] << 1;   // 变成16位结果
	
	SPI_ReadWriteData(u16RegValue);
	u16Data = SPI_ReadWriteData(0xffff)<<1;
	
	SPI_CS_GPIO_Port->ODR |= SPI_CS_Pin;
	
	u16Data = (65536 + u16Data - offset);
	
	u16Data = 65536 - u16Data;

	return u16Data;
}

