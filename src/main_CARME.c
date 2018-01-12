/**
 *****************************************************************************
 * @defgroup	Project
 * @brief		Project related code
 * @{
 *
 * @file		main.c
 * @version		1.0
 * @date		2013-11-22
 * @author		rct1
 *
 * @brief		main.c template
 *
 *****************************************************************************
 * @copyright
 * @{
 * Copyright &copy; 2013, Bern University of Applied Sciences.
 * All rights reserved.
 *
 * ##### GNU GENERAL PUBLIC LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 * @}
 *****************************************************************************
 */

/*----- Header-Files -------------------------------------------------------*/
#include <stm32f4xx.h>				/* Processor STM32F407IG				*/
#include "carme.h"					/* CARME Module							*/
//#include "uart.h"					/* Carme-UART Module					*/
#include "lcd.h"					/* Carme-LCD Module						*/
#include "stm32f4xx_usart.h"		/* UART STM32F4 Module					*/
#include "i2c.h"					/* Carme-I2C Module						*/
#include "stdio.h"					/* Standard Input/Output				*/
#include "stdlib.h"					/* Standard Library						*/
#include "string.h"					/* String Library						*/

/*----- Macros -------------------------------------------------------------*/
#define INT_PER_SEC			1000U	/**< SysTick interrupts per second		*/
#define NMEA_stringlength	120

// Accelero I2C-Address
#define I2C_AXL_ADDR		0x53 << 1
// Accelero Controlregisters
#define BW_RATE				0x2C
#define POWER_CTL			0x2D
#define DATA_FORMAT			0x31
// Accelero Dataregisters
#define DATAX0 				0x32 //X
#define DATAX1 				0x33
#define DATAY0 				0x34 //Y
#define DATAY1 				0x35
#define DATAZ0 				0x36 //Z
#define DATAZ1 				0x37
// Accelero Scale
#define ACC_Scale			0.0039f

// Gyro I2C-Address
#define I2C_GYR_ADDR 		0x69 << 1
// Gyro Controlregisters
#define CTRL_REG1 			0x20
#define CTRL_REG2 			0x21
#define CTRL_REG3 			0x22
#define CTRL_REG4 			0x23
#define CTRL_REG5 			0x24
#define REFERENCE 			0x25
#define STATUS_REG 			0x27
// Gyro Dataregisters
#define OUT_X_L				0x28
#define OUT_X_H				0x29
#define OUT_Y_L				0x2A
#define OUT_Y_H				0x2B
#define OUT_Z_L				0x2C
#define OUT_Z_H				0x2D
// Gyro Scale
#define GYR_Scale_250		0.00875f
#define GYR_Scale_500		0.0175f
#define GYR_Scale_2000		0.07f
// Gyro Resolution
#define GYR_Res_250			0x00
#define GYR_Res_500			0x10
#define GYR_Res_2000		0x20


/*----- Data types ---------------------------------------------------------*/
ERROR_CODES err_code __attribute__((unused));

/*----- Function prototypes ------------------------------------------------*/
void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);
void Display_NMEA(char NMEA_string[NMEA_stringlength]);

/*----- Data ---------------------------------------------------------------*/
static __IO uint32_t TimingDelay;
//NMEA-Data
char NMEA_string[NMEA_stringlength];
static struct NMEA
{
	char GGA[NMEA_stringlength];
	char GLL[NMEA_stringlength];
	char GSA[NMEA_stringlength];
	char GSV[NMEA_stringlength];
	char RMC[NMEA_stringlength];
	char VTG[NMEA_stringlength];
	char ZDA[NMEA_stringlength];
}NMEA;
uint8_t NMEAStringReadyFlag = 0;
//Gyro-Data
int16_t X_RawError, Y_RawError, Z_RawError;

/*----- Implementation -----------------------------------------------------*/
/**
 * @brief		This function waits for nTime / INT_PER_SEC seconds.
 *
 * @param[in]	nTime	Time to wait
 */
void Delay(__IO uint32_t nTime) {
	TimingDelay = nTime;
	while (TimingDelay != 0U) {
	}
}

/**
 * @brief		Decrement the TimingDelay. This function is called in the
 *				SysTick_Handler.
 */
void TimingDelay_Decrement(void) {
	if (TimingDelay > 0U) {
		TimingDelay--;
	}
}

void Init_USART1(void)
{
	/* Init GPIO Ports [PA9=TX, PA10=RX] for UART1 */
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	// Set GPIOA & USART1 Peripherial Clock Ressource
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	// Init USART1 (to change settings disable USART1)
	USART_DeInit(USART1);
	// Init USART1 Clock
	USART_ClockInitTypeDef USART1_ClockInit;
	USART_ClockStructInit(&USART1_ClockInit);
	USART_ClockInit(USART1, &USART1_ClockInit);
	USART_Cmd(USART1, DISABLE);
	USART_InitTypeDef USART1_Init;
	USART_StructInit(&USART1_Init);
	USART_Init(USART1, &USART1_Init);
	// Enable UART1 Interrupts
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;	// set priority of the USART1 interrupt to highest (lowest=15)
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 10;			// set the subpriority inside the group to highest (lowest=15)
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 		// enable the USART1 interrupts globally
	NVIC_Init(&NVIC_InitStructure);							 		// NVIC_Init function takes care of the low level stuff
	USART_Cmd(USART1, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_RXNE);
	uint8_t i = 0xFF;
	// Wait until hardware is running
	while (i--);
}

/**
 * @brief		interrupt request handler (IRQ) for ALL USART1 interrupts
 */
void USART1_IRQHandler(void)
{
	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) )
	{
		static uint8_t cnt = 0; // this counter is used to determine the current string length
		char t = USART1->DR;  // the character from the USART1 data register is saved in t
		static uint8_t NMEA_start = 0;
		if (cnt < NMEA_stringlength)
		{
			switch (t)
			{
				case '$':
					NMEA_start = 1;
					break;
				case '\r':
					NMEA_start = 0;
					break;
				case '\n':
					NMEA_start = 0;
					break;
				default:
					break;
			}
			if(NMEA_start)
			{
				NMEA_string[cnt++] = t;
			}
		}
		else
		{
			NMEAStringReadyFlag = 1;
			NMEA_start=0;
			cnt=0;
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

void Init_I2C2(void)
{
	/* Configure the GPIO [PH4=SCL, PH5=SDA] for I2C2*/
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOH, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOH, GPIO_PinSource4, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOH, GPIO_PinSource5, GPIO_AF_I2C2);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	I2C_InitTypeDef I2C_InitStruct;
	I2C_StructInit(&I2C_InitStruct);
	I2C_InitStruct.I2C_ClockSpeed = 100000;
	I2C_InitStruct.I2C_OwnAddress1 = 0xA0;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_Cmd(I2C2, DISABLE);
	I2C_DeInit(I2C2);
	I2C_Init(I2C2, &I2C_InitStruct);
	I2C_Cmd(I2C2, ENABLE);
	uint8_t i = 0xFF;
	// Wait until hardware is running
	while (i--);
}

void AcceleroInit(void)
{
	uint8_t value = 0x0F;
	err_code = CARME_I2C_Write(I2C2,I2C_AXL_ADDR,BW_RATE,0,&value,1); //Set Data rate

	value = 0x08;
	err_code = CARME_I2C_Write(I2C2,I2C_AXL_ADDR,POWER_CTL,0,&value,1); //Choose power-mode

	value = 0x08;
	err_code = CARME_I2C_Write(I2C2,I2C_AXL_ADDR,DATA_FORMAT,0,&value,1); //Data format
}

void ReadAccelero(int16_t *gx, int16_t *gy, int16_t *gz)
{
	uint8_t gravitation[6];
	err_code = CARME_I2C_Read(I2C2, I2C_AXL_ADDR, DATAX0, 0, gravitation, sizeof(gravitation));

	*gx = ((int16_t)(gravitation[1] << 8) | gravitation[0]);
	*gy = ((int16_t)(gravitation[3] << 8) | gravitation[2]);
	*gz = ((int16_t)(gravitation[5] << 8) | gravitation[4]);
}

/*
 * @brief 		displays measured acceleration in all 3 axis
 */
void Display_Accelero(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t MaxLength = LCD_HOR_RESOLUTION / LCD_GetFont()->width;
	char acc_disp[MaxLength];

	if (*x != 0)
	{
		snprintf(acc_disp, MaxLength, "Acc_X:%d.%03d", (int)(*x*ACC_Scale),abs((int)(*x*ACC_Scale*1000)%1000));
		LCD_DisplayStringLine(8, acc_disp);
	}
	if (*y != 0)
	{
		snprintf(acc_disp, MaxLength, "Acc_Y:%d.%03d",(int)(*y*ACC_Scale),abs((int)(*y*ACC_Scale*1000)%1000));
		LCD_DisplayStringLine(9, acc_disp);
	}
	if (*z != 0)
	{
		snprintf(acc_disp, MaxLength, "Acc_Z:%d.%03d",(int)(*z*ACC_Scale),abs((int)(*z*ACC_Scale*1000)%1000));
		LCD_DisplayStringLine(10, acc_disp);
	}
}

void GyroInit(void)
{
	uint8_t value = 0x00;
	err_code = CARME_I2C_Write(I2C2,I2C_GYR_ADDR,CTRL_REG2,0,&value,1); // disable HPF
	value = 0x00;
	err_code = CARME_I2C_Write(I2C2,I2C_GYR_ADDR,CTRL_REG3,0,&value,1); // disable interrupts
	value = GYR_Res_250;
	err_code = CARME_I2C_Write(I2C2,I2C_GYR_ADDR,CTRL_REG4,0,&value,1); // Set Resolution
	value = 0x0F;
	err_code = CARME_I2C_Write(I2C2,I2C_GYR_ADDR,CTRL_REG1,0,&value,1); //Wakeup Gyro, enable x,y,z
	// Quick Calibration
	float X_BiasError, Y_BiasError, Z_BiasError = 0.0;
	uint16_t BiasErrorSplNbr = 500;
	uint16_t i = 0;

	for (i = 0; i < BiasErrorSplNbr; i++)
	{
		uint8_t angularrate[6];
			err_code = CARME_I2C_Read(I2C2, I2C_GYR_ADDR, OUT_X_L | (1 << 7), 0, angularrate, sizeof(angularrate));

	    X_BiasError += ((int16_t)(angularrate[1] << 8) | angularrate[0]);
	    Y_BiasError += ((int16_t)(angularrate[3] << 8) | angularrate[2]);
	    Z_BiasError += ((int16_t)(angularrate[5] << 8) | angularrate[4]);
	}
	// Set raw errors
	X_RawError = X_BiasError / BiasErrorSplNbr;
	Y_RawError = Y_BiasError / BiasErrorSplNbr;
	Z_RawError = Z_BiasError / BiasErrorSplNbr;
}

void ReadGyro(int16_t *dx, int16_t *dy, int16_t *dz)
{
	uint8_t value = 0;
	err_code = CARME_I2C_Read(I2C2,I2C_GYR_ADDR,STATUS_REG,0,&value,1); //Wakeup Gyro
	if ( value&0x08 )
	{
		uint8_t angularrate[6];
		err_code = CARME_I2C_Read(I2C2, I2C_GYR_ADDR, OUT_X_L | (1 << 7), 0, angularrate, sizeof(angularrate));

		*dx = ((int16_t)(angularrate[1] << 8) | angularrate[0]) + (-1*X_RawError);
		*dy = ((int16_t)(angularrate[3] << 8) | angularrate[2]) + (-1*Y_RawError);
		*dz = ((int16_t)(angularrate[5] << 8) | angularrate[4]) + (-1*Z_RawError);
	}

}

void Display_Gyro(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t MaxLength = LCD_HOR_RESOLUTION / LCD_GetFont()->width;
	char gyr_disp[MaxLength];

	if (*x != 0)
	{
		snprintf(gyr_disp, MaxLength, "Gyr_X:%d.%03d",(int)(*x*GYR_Scale_250),abs((int)(*x*GYR_Scale_250*1000)%1000));
		LCD_DisplayStringLine(12, gyr_disp);
	}
	if (*y != 0)
	{
		snprintf(gyr_disp, MaxLength, "Gyr_Y:%d.%03d",(int)(*y*GYR_Scale_250),abs((int)(*y*GYR_Scale_250*1000)%1000));
		LCD_DisplayStringLine(13, gyr_disp);
	}
	if (*z != 0)
	{
		snprintf(gyr_disp, MaxLength, "Gyr_Z:%d.%03d",(int)(*z*GYR_Scale_250),abs((int)(*z*GYR_Scale_250*1000)%1000));
		LCD_DisplayStringLine(14, gyr_disp);
	}
}

uint8_t GetSegment_Length(char *str)
{
	uint8_t MaxLength = min((LCD_HOR_RESOLUTION / LCD_GetFont()->width),strlen(str));
	for (int segment_index = 0; segment_index<MaxLength; segment_index++)
	{
		switch (str[segment_index])
		{
			case '$':
				return 0;
			case '*':
				return segment_index+3;
			default:
				break;
		}
	}
	return 0;
}

void Display_NMEA(char *NMEA_string)
{
	// Disable UART1 Interrupts
	NVIC_DisableIRQ(USART1_IRQn);
	uint8_t SegmentLength = 1;
	for ( int search=0 ; search<NMEA_stringlength-6 ; search+=SegmentLength )
	{
		if ((NMEA_string[search] == '$' && NMEA_string[search+1] == 'G') && NMEA_string[search+2] == 'P')
		{
			switch (NMEA_string[search+3])
			{
				case 'G':
					switch (NMEA_string[search+4]) {
						case 'G':
							if (NMEA_string[search+5] == 'A')
							{
								SegmentLength = GetSegment_Length(&NMEA_string[search+6]);
								// TODO: only copy if checksum correct: EXOR(NMEA_string[search+1]-to-NMEA_string[search+SegmentLength+2]
								strncpy(&NMEA.GGA[4], &NMEA_string[search+6], SegmentLength);
								if (SegmentLength == 0) SegmentLength = 1;
								else SegmentLength+=6;
							}
							break;
						case 'S':
							if (NMEA_string[search+5] == 'A')
							{
								SegmentLength = GetSegment_Length(&NMEA_string[search+6]);
								strncpy(&NMEA.GSA[4], &NMEA_string[search+6], SegmentLength);
								if (SegmentLength == 0) SegmentLength = 1;
								else SegmentLength+=6;
							}
							else if (NMEA_string[search+5] == 'V')
							{
								SegmentLength = GetSegment_Length(&NMEA_string[search+6]);
								strncpy(&NMEA.GSV[4], &NMEA_string[search+6], SegmentLength);
								if (SegmentLength == 0) SegmentLength = 1;
								else SegmentLength+=6;
							}
							break;
						case 'L':
							if (NMEA_string[search+5] == 'L')
							{
								SegmentLength = GetSegment_Length(&NMEA_string[search+6]);
								strncpy(&NMEA.GLL[4], &NMEA_string[search+6], SegmentLength);
								if (SegmentLength == 0) SegmentLength = 1;
								else SegmentLength+=6;
							}
							break;
						default:
							break;
					}

					break;
				case 'R':
					if (NMEA_string[search+4] == 'M' && NMEA_string[search+5] == 'C')
					{
						SegmentLength = GetSegment_Length(&NMEA_string[search+6]);
						strncpy(&NMEA.RMC[4], &NMEA_string[search+6], SegmentLength);
						if (SegmentLength == 0) SegmentLength = 1;
						else SegmentLength+=6;
					}
					break;
				case 'V':
					if (NMEA_string[search+4] == 'T' && NMEA_string[search+5] == 'G')
					{
						SegmentLength = GetSegment_Length(&NMEA_string[search+6]);
						strncpy(&NMEA.VTG[4], &NMEA_string[search+6], SegmentLength);
						if (SegmentLength == 0) SegmentLength = 1;
						else SegmentLength+=6;
					}
					break;
				case 'Z':
					if (NMEA_string[search+4] == 'D' && NMEA_string[search+5] == 'A')
					{
						SegmentLength = GetSegment_Length(&NMEA_string[search+6]);
						strncpy(&NMEA.ZDA[4], &NMEA_string[search+6], SegmentLength);
						if (SegmentLength == 0) SegmentLength = 1;
						else SegmentLength+=6;
					}
					break;
				default:
					break;
			}
		}
		else SegmentLength = 1;
	}

	// Empty NMEA string
	for (int i = 0; i<NMEA_stringlength; i++)
		NMEA_string[i] = '\0';
	// Reenable UART1 interrupts
	NVIC_EnableIRQ(USART1_IRQn);
	if (NMEA.GGA[4] != 0)
	{
		LCD_DisplayStringLine(0, NMEA.GGA);
		for (int i = 4; i<NMEA_stringlength; i++)
			if (NMEA.GGA[i] != 0) NMEA.GGA[i] = 0;
			else break;
	}
	if (NMEA.GLL[4] != 0)
	{
		LCD_DisplayStringLine(1, NMEA.GLL);
		for (int i = 4; i<NMEA_stringlength; i++)
			if (NMEA.GLL[i] != 0) NMEA.GLL[i] = 0;
			else break;
	}
	if (NMEA.GSA[4] != 0)
	{
		LCD_DisplayStringLine(2, NMEA.GSA);
		for (int i = 4; i<NMEA_stringlength; i++)
			if (NMEA.GSA[i] != 0) NMEA.GSA[i] = 0;
			else break;
	}
	if (NMEA.GSV[4] != 0)
	{
		LCD_DisplayStringLine(3, NMEA.GSV);
		for (int i = 4; i<NMEA_stringlength; i++)
			if (NMEA.GSV[i] != 0) NMEA.GSV[i] = 0;
			else break;
	}
	if (NMEA.RMC[4] != 0)
	{
		LCD_DisplayStringLine(4, NMEA.RMC);
		for (int i = 4; i<NMEA_stringlength; i++)
			if (NMEA.RMC[i] != 0) NMEA.RMC[i] = 0;
			else break;
	}
	if (NMEA.VTG[4] != 0)
	{
		LCD_DisplayStringLine(5, NMEA.VTG);
		for (int i = 4; i<NMEA_stringlength; i++)
			if (NMEA.VTG[i] != 0) NMEA.VTG[i] = 0;
			else break;
	}
	if (NMEA.ZDA[4] != 0)
	{
		LCD_DisplayStringLine(6, NMEA.ZDA);
		for (int i = 4; i<NMEA_stringlength; i++)
			if (NMEA.ZDA[i] != 0) NMEA.ZDA[i] = 0;
			else break;
	}

}

/**
 * @brief		main
 * @return		0 if success
 */
int main(void) {

	/* LCD Init */
 	LCD_Init(); 						// CARME LCD initialisieren
	LCD_SetTextColor(GUI_COLOR_WHITE);	// Schfiftfarbe setzen
	LCD_SetFont(&font_5x7);

	/* USART1 Init */
	Init_USART1();

	/* I2C Init */
	Init_I2C2();

	// Init Accelerometer
	AcceleroInit();

	// Init Gyro
	GyroInit();

	// Init NMEA struct
	strncpy(NMEA.GGA, "GGA:", 4);
	strncpy(NMEA.GSA, "GSA:", 4);
	strncpy(NMEA.GLL, "GLL:", 4);
	strncpy(NMEA.GSV, "GSV:", 4);
	strncpy(NMEA.RMC, "RMC:", 4);
	strncpy(NMEA.VTG, "VTG:", 4);
	strncpy(NMEA.ZDA, "ZDA:", 4);

	while (1)
	{
		if (NMEAStringReadyFlag)
		{
			/* Display received NMEA String*/
			Display_NMEA(NMEA_string);
			//ClearFlag
			NMEAStringReadyFlag = 0;
		}

		/* receive Accelero Data */
		int16_t ax = 0;
		int16_t ay = 0;
		int16_t az = 0;
		ReadAccelero(&ax,&ay,&az);
		Display_Accelero(&ax,&ay,&az);

		/* receive Gyro Data */
		int16_t dx = 0;
		int16_t dy = 0;
		int16_t dz = 0;
		ReadGyro(&dx,&dy,&dz);
		Display_Gyro(&dx,&dy,&dz);
	}
	return 0U;
}

/**
 * @}
 */
