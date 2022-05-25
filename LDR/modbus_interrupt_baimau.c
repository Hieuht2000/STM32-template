/*
By sending text from terminal causes USART2 (RX) interrupt and software
prints (echoes) transmitted text back to terminal.
*/

/* Includes */
#include "stm32l1xx.h"
#define HSI_VALUE    ((uint32_t)16000000)
#include "nucleo152start.h"
#include <stdio.h>

/* Private typedef */
/* Private define  */
#define READ_LENGTH 7
#define BAUD_RATE 9600
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */
void delay_Ms(int delay);
void USART_write(char data);
char USART_read();
void USART2_Init(void);
void read_7_bytes_from_usartx(unsigned char *received_frame);

/* Global variables */
char mFlag=0;
int delay_byte = READ_LENGTH * (1/BAUD_RATE);

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

	unsigned char input[READ_LENGTH+1] = {'K'};

	/* Infinite loop */
	while (1)
	{
		if (mFlag == 1)
		{
			int i = 0;

			/*Read 7 bytes from sensor*/
			read_7_bytes_from_usartx(input);
			/* Disable receiver */
			USART2->CR1 &= ~0x04;

			for(i = 0; i < READ_LENGTH; i++)
			{
				USART_write(input[i]);
			}

			/* mFlag = 0 */
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
			delay_Ms(1);
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

void USART_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART2->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART2->DR=(data);			//p739
}

char USART_read()
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
		if(c == 0x08)
		{
			mFlag = 1;
			GPIOA->ODR|=0x20; //0010 0000 set bit 5. p186
			delay_Ms(1500);
			GPIOA->ODR&=~0x20; //0000 0000 clear bit 5. p186
			delay_Ms(1500);
			//USART2->CR1 &= ~0x0020;			//disable RX interrupt
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
	int i;

	for(i = 0; i < READ_LENGTH; i++)
	{
		*(received_frame+i) = USART_read();
	}
}
