/*/*
By sending text from terminal causes USART2 (RX) interrupt and software
prints (echoes) transmitted text back to terminal.
*/

/* Includes */
#include "stm32l1xx.h"
#define HSI_VALUE    ((uint32_t)16000000)
#include "nucleo152start.h"
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

/* Private typedef */
/* Private define  */
#define READ_LENGTH 7
#define BAUD_RATE 9600
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */
void delay_Ms(int delay);
void delay_Us(int delay);
void USART_write(char data);
char USART_read();
void USART2_Init(void);
void read_7_bytes_from_usartx(unsigned char *received_frame);
unsigned short int CRC16 (unsigned char *nData,unsigned short int wLength);
int read_sensor(int input_reg);
void respond_frame(int sensor_value);
/* Global variables */
char mFlag=0;

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

int main(void)
{
	__disable_irq();			//global disable IRQs, M3_Generic_User_Guide p135.
	USART2_Init();

	/* Configure the system clock to 32 MHz and update SystemCoreClock */
	SetSysClock();
	SystemCoreClockUpdate();

	/* TODO - Add your application code here */

	USART2->CR1 |= 0x0020;			//enable RX interrupt
	NVIC_EnableIRQ(USART2_IRQn); 	//enable interrupt in NVIC
	__enable_irq();					//global enable IRQs, M3_Generic_User_Guide p135

	RCC->AHBENR|=1; 				//GPIOA ABH bus clock ON. p154
	GPIOA->MODER&=~0x00000C00;		//clear (input reset state for PA5). p184
	GPIOA->MODER|=0x400; 			//GPIOA pin 5 to output. p184
	GPIOA->ODR^=0x20;				//0010 0000 xor bit 5. p186
	delay_Ms(1000);
	GPIOA->ODR^=0x20;				//0010 0000 xor bit 5. p186
	delay_Ms(1000);
	GPIOA->MODER&=~0x400;	//PA5 to input
	GPIOA->ODR&=~0x20; //0000 0000 clear bit 5. p186

	unsigned char request_frame[8]={0x04};	// initialize the frame with the slave address
	int crc=0;
	int sensor_value = 0;

	/* Infinite loop */
	while (1)
	{

		if (mFlag == 1)
		{
			/* Read 7 bytes from sensor if the slave's address(0x04) is matched */
			read_7_bytes_from_usartx(request_frame);

			/* CRC calculated */
			crc = CRC16(request_frame[0],6);

			/*Read the sensor values if the checksum in the frame is correct*/
			if(crc == 0x605F){
				sensor_value = read_sensor(request_frame[3]);
				//respond_frame(sensor_value); //not finished
				delay_Ms(20);
			}

			/* Disable receiver */
			USART2->CR1 &= ~0x04;

			/* mFlag = 0, ready for a new request */
			mFlag = 0;
			/* enable USARTx interrupt */
			USART2->CR1 |= 0x0020;
			/* Enable receiver */
			USART2->CR1 |= 0x04;
		}
		else if (mFlag == 2)
		{
			/* Disable receiver*/
			USART2->CR1 &= ~0x04;
			/* Delay for 7 bytes */
			delay_Ms(10);
			/* mFlag = 0 */
			mFlag = 0;
			/* enable USARTx interrupt */
			USART2->CR1 |= 0x0020;
			/* Enable receiver */
			USART2->CR1 |= 0x04;
		}
	}
	return 0;
}

void delay_Us(int delay)
{
	for(int i=0;i<(delay*2);i++) //accurate range 10us-100us
	{
		asm("mov r0,r0");
	}
}

void delay_Ms(int delay)
{
	int i=0;
	for(; delay>0;delay--)
		for(i=0;i<2460;i++); 	//measured with oscilloscope
}

void USART2_Init(void)
{
	RCC->APB1ENR|=0x00020000; 	//set bit 17 (USART2 EN)
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[0]=0x00000700;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->AFR[0]|=0x00007000;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->MODER|=0x00000020; 	//MODER2=PA2(TX) to mode 10=alternate function mode. p184
	GPIOA->MODER|=0x00000080; 	//MODER2=PA3(RX) to mode 10=alternate function mode. p184

	USART2->BRR = 0x00000D05;	//9600 BAUD and crystal 32MHz. p710, D05
	USART2->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART2->CR1 |= 0x00000004;	//RE bit. p739-740. Enable receiver
	USART2->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}

void USART2_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART2->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART2->DR=(data);			//p739
}

char USART2_read()
{
	char input;

	while(!(USART2->SR&0x0020)){} 	//Bit 5 RXNE: Read data register not empty
		input = USART2->DR;			//p739
		return input;
}

void USART2_IRQHandler(void)
{
	int c = 0;

	//This bit is set by hardware when the content of the
	//RDR shift register has been transferred to the USART_DR register.
	if(USART2->SR & 0x0020) 		//if data available in DR register. p739
	{
		c = USART2->DR;
		if(c == 0x04)
		{
			mFlag = 1;						// if the slave address is correct the flag is 1
			USART2->CR1 &= ~0x0020;			//disable RX interrupt
		}
		else
		{
			mFlag = 2;
			USART2->CR1 &= ~0x0020;			//disable RX interrupt
		}
	}
}

/* Read input from Terminal */
void read_7_bytes_from_usartx(unsigned char *received_frame)
{
	int i=0;

	for(i = 1; i <= READ_LENGTH; i++)
	{
		*(received_frame+i) = USART2_read();
	}
}

/*Take the values from the sensor*/
int read_sensor(int input_register){
	int result=0;
	double lux=0;
	char buf[] = "";
	int lux_degree = 0;
	int lux_decimals = 0;


	RCC->AHBENR |= 1; //enable GPIOA clock
	RCC->AHBENR |= 4; //enable GPIOA clock
	GPIOA->MODER |= 0x3; //PA0 analog (A0)
	//setup ADC1. p272
	RCC->APB2ENR |= 0x00000200; //enable ADC1 clock
	ADC1->CR2 = 0; //bit 1=0: Single conversion mode
	ADC1->SQR3 = 10;
	ADC1->SQR1 = 0;
	ADC1->CR2 |= 1;


	//read values from sensor
	while (1) {
		ADC1->CR2 |= 0x40000000;
		while (!(ADC1->SR & 2)) {
		} //wait for conversion complete
		result = ADC1->DR;

		//EQUATION
		lux=-0.5825*(double)result+2537.7;

		// Print Decimal Value
		lux = lux * 100; //remove decimals and 34.54 = 3454
		lux_degree = (int) lux / 100;
		lux_decimals = abs((int) lux % 100);
		// sprintf(buf, "%d.%d lux ", lux_degree, lux_decimals);

		// int len = 0;
		// while (buf[len] != '\0')
		// 	len++;

		// for (int i = 0; i < len; i++) {
		// 	USART2_write(buf[i]);
		// }

		// USART2_write('\n');
		// USART2_write('\r');

		// for (int i = 0; i < len; i++) {
		// 	buf[i]=0;
		// }
		if (input_register == 0x01){
			return lux;
		}
		
		delay_Ms(500);
}

/*Respond Frame to Master*/

void respond_frame(int sensor_value){

}
unsigned short int CRC16 (unsigned char *nData,unsigned short int wLength){
	//parameter wLenght = how my bytes in your frame?
	//*nData = your first element in frame array

	static const unsigned short int wCRCTable[] = {
	0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
	0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
	0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
	0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
	0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
	0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
	0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
	0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
	0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
	0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
	0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
	0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
	0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
	0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
	0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
	0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
	0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
	0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
	0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
	0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
	0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
	0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
	0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
	0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
	0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
	0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
	0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
	0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
	0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
	0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
	0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
	0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

	unsigned char nTemp;
	unsigned short int wCRCWord = 0xFFFF;

	   while (wLength--)
	   {
	      nTemp = *nData++ ^ wCRCWord;
	      wCRCWord >>= 8;
	      wCRCWord ^= wCRCTable[nTemp];
	   }
	   return wCRCWord;

}