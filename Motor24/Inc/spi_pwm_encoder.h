#include "stm32f1xx_hal.h"

#define ABS_ENCODER_MODE // When use absolute encoder, uncomment this
#define POSITION_CONTROL // When work under position mode, uncomment this

/* 
 * Should use one of the mode, SPI mode or PWM mode 
 * Comment one, uncomment the other
*/
#define ENCODER_SPI_MODE
//#define ENCODER_PWM_MODE

#define SENSOR_SPI 0
#define SENSOR_PWM 1

// Command for AS5048A
#define CMD_ANGLE 0xFFFF 
#define CMD_AGC   0x7FFD 
#define CMD_MAG   0x7FFE 
#define CMD_CLAER 0x4001 
#define CMD_NOP   0xC000 

#define ENCODER_SPI_MAX 65536
#define POLES 21   

/* Note: PWM from 0.907KHz~1.102KHz,  normal value is 1KHz,the data maybe calculate in code again*/
#define ENCODER_PWM_MAX  67039  //72000 // 72MHz/1KHz = 72000

/* Parameter for speed calculate*/
#define ENCODER_SPEED_ARRAY_SIZE 16


#define CS_PORT GPIOA
#define CS_PIN  GPIO_PIN_4

extern volatile uint32_t foccnt;
extern uint16_t offset_ia;
extern uint16_t offset_ib;

extern uint16_t SPI_Angle_Digital; 
extern int16_t SPI_EIAngle;
extern uint32_t PWM_Angle_Digital;
extern int16_t PWM_EIAngle;


uint16_t SPI_Encoder_EIAngle(void);


//uint16_t SPI_Read5048Data(uint16_t TxData);
//void PWM_Encoder_EIangle(uint32_t data);
//void ClearAndNop(void) ;



