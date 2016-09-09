/*****************************************************************************
* CODE OWNERSHIP AND DISCLAIMER OF LIABILITY
*
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in
* all derivatives hereto.  You may use this code, and any derivatives created
* by any person or entity by or on your behalf, exclusively with Microchip’s
* proprietary products.  Your acceptance and/or use of this code constitutes
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT
* LIMITED TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND
* FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH
* MICROCHIP’S PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY
* APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
* WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
* STATUTORY DUTY), STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
* FOR ANY INDIRECT, SPECIAL, PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
* CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE
* BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS
* CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY
* TO HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
* Author                Date        Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* jgh                  12/19/11    Specific hardware for the PIC18F46K22 Dev board
******************************************************************************/

#ifndef __PCAP_HARDWARE_PIC18F46K22_DEV_H
#define __PCAP_HARDWARE_PIC18F46K22_DEV_H

#include <p18f46k22.h>
#include <delays.h>

//
// controller specifics, type version number
//
// CONFIGIDHIGH is now set in the hardware .h files
//#define CONFIGIDHIGH 0x83			// 0x80 - development code
									// 0	- PIC16F707 based
									// 1 	- PIC18F46K22 based
									// 2	- PCAPPIC24F64GB106 based
									// 3	- PIC32MX
									// 4 	- PIC24FJ64GB004
									// 5	- PIC32MX120F032D
#define CONFIGIDHIGH 0x81

//
// Maximum sensor dimensions
//
#ifdef DEVKIT_HARDWARE
	#define MAXRX 19
	#define MAXTX 14
#else // DEVKIT_HARDWARE
	#define MAXRX NUMBEROFRXCHANNELS
	#define MAXTX NUMBEROFTXCHANNELS
#endif // DEVKIT_HARDWARE
//
// headers
//
void InterruptHandlerHigh();
void hardwarePreInit(void);
void hardwarePostInit(void);
void delay1ms(unsigned short delay);
void initPCHardware(void);
void groundAll(void);
void preInitSelf(void);
void selfCTMUScan(unsigned char y);
void selfCVDScan(unsigned char y);
void preInitMutual(void);
void mutualCTMUScan(unsigned char y);
void mutualCVDScan(unsigned char y);
void initI2C(void);
void readI2C(void);
void writeI2C(void);
void initUART(void);
void writeByteToEEPROM( unsigned int address,unsigned char data);
unsigned char readByteFromEEPROM( unsigned int address);
void writeRAMToEEPROM(void);
void readEEPROMToRAM(void);
void eraseEEPROM(void);
void checkIO(void);


void peripheralShutdown(void);
void peripheralWakeup(void);
void goToSleep(void);


void initLCDSPI(void);
void sendSPIchar(unsigned char c);
void waitOnSPITX(void);
void sendSPIstring(const unsigned char *s, int size);

void initGlobalTimer(void);

//
// define the look up tables for TRIS, and LAT, and
// analog select, note these are in the same order as
// RX and TX PINMAP's
//
#pragma romdata overlay LUT
static near rom unsigned char TRIS_LUT[] =
{
	0b00000001,	//	0	RA0, AN0
	0b00000010,	//	1	RA1, AN1
	0b00000100,	//	2	RA2, AN2
	0b00001000,	//	3	RA3, AN3
	0b00100000,	//	4	RA5, AN4
	0b00000001,	//	5	RE0, AN5
	0b00000010,	//	6	RE1, AN6
	0b00000100,	//	7	RE2, AN7
	0b00000100,	//	8	RB2, AN8
	0b00001000,	//	9	RB3, AN9
	0b00000010,	//	10	RB1, AN10
	0b00010000,	//	11	RB4, AN11
	0b00000001,	//	12	RB0, AN12
	0b00100000,	//	13	RB5, AN13
	0b00000100,	//	14	RC2, AN14
	0b00001000,	//	15	RC3, AN15
	0b00010000,	//	16	RC4, AN16
	0b00100000,	//	17	RC5, AN17
	0b01000000,	//	18	RC6, AN18
	0b10000000,	//	19	RC7, AN19
	0b00000001,	//	20	RD0, AN20
	0b00000010,	//	21	RD1, AN21
	0b00000100,	//	22	RD2, AN22
	0b00001000,	//	23	RD3, AN23
	0b00010000,	//	24	RD4, AN24
	0b00100000,	//	25	RD5, AN25
	0b01000000,	//	26	RD6, AN26
	0b10000000	//	27	RD7, AN27
};

//#pragma romdata overlay TRISPORTLUT =0x800
static near rom unsigned int TRISPORT_LUT[] =
{
	0x0F92,		//	0:A
	0x0F92,		//	1:A
	0x0F92,		//	2:A
	0x0F92,		//	3:A
	0x0F92,		//	4:A
	0x0F96,		//	5:E
	0x0F96,		//	6:E
	0x0F96,		//	7:E
	0x0F93,		//	8:B
	0x0F93,		//	9:B
	0x0F93,		//	10:B
	0x0F93,		//	11:B
	0x0F93,		//	12:B
	0x0F93,		//	13:B
	0x0F94,		//	14:C
	0x0F94,		//	15:C
	0x0F94,		//	16:C
	0x0F94,		//	17:C
	0x0F94,		//	18:C
	0x0F94,		//	19:C
	0x0F95,		//	20:D
	0x0F95,		//	21:D
	0x0F95,		//	22:D
	0x0F95,		//	23:D
	0x0F95,		//	24:D
	0x0F95,		//	25:D
	0x0F95,		//	26:D
	0x0F95		//	27:D
};
//#pragma code
static near rom unsigned int ANSELPORT_LUT[] =
{
	0x0F38,		//	0:A
	0x0F38,		//	1:A
	0x0F38,		//	2:A
	0x0F38,		//	3:A
	0x0F38,		//	4:A
	0x0F3C,		//	5:E
	0x0F3C,		//	6:E
	0x0F3C,		//	7:E
	0x0F39,		//	8:B
	0x0F39,		//	9:B
	0x0F39,		//	10:B
	0x0F39,		//	11:B
	0x0F39,		//	12:B
	0x0F39,		//	13:B
	0x0F3A,		//	14:C
	0x0F3A,		//	15:C
	0x0F3A,		//	16:C
	0x0F3A,		//	17:C
	0x0F3A,		//	18:C
	0x0F3A,		//	19:C
	0x0F3B,		//	20:D
	0x0F3B,		//	21:D
	0x0F3B,		//	22:D
	0x0F3B,		//	23:D
	0x0F3B,		//	24:D
	0x0F3B,		//	25:D
	0x0F3B,		//	26:D
	0x0F3B		//	27:D
};

static near rom unsigned char LAT_LUT[] =
{
	0b00000001,	//	0	RA0
	0b00000010,	//	1	RA1
	0b00000100,	//	2	RA2
	0b00001000,	//	3	RA3
	0b00010000,	//	4	RA4
	0b00100000,	//	5	RA5
	0b01000000,	//	6	RA6
	0b10000000,	//	7	RA7
	0b00000001,	//	8	RB0
	0b00000010,	//	9	RB1
	0b00000100,	//	10	RB2
	0b00001000,	//	11	RB3
	0b00010000,	//	12	RB4
	0b00100000,	//	13	RB5
	0b01000000,	//	14	RB6
	0b10000000,	//	15	RB7
	0b00000001,	//	16	RC0
	0b00000010,	//	17	RC1
	0b00000100,	//	18	RC2
	0b00001000,	//	19	RC3
	0b00010000,	//	20	RC4
	0b00100000,	//	21	RC5
	0b01000000,	//	22	RC6
	0b10000000,	//	23	RC7
	0b00000001,	//	24	RD0
	0b00000010,	//	25	RD1
	0b00000100,	//	26	RD2
	0b00001000,	//	27	RD3
	0b00010000,	//	28	RD4
	0b00100000,	//	29	RD5
	0b01000000,	//	30	RD6
	0b10000000,	//	31	RD7
	0b00000001,	//	32	RE0
	0b00000010,	//	33	RE1
	0b00000100,	//	34	RE2
	0b00001000	//	35	RE3
};

static near rom unsigned int LATPORT_LUT[] =
{
	0x0F89,		//	0:A
	0x0F89,		//	1:A
	0x0F89,		//	2:A
	0x0F89,		//	3:A
	0x0F89,		//	4:A
	0x0F89,		//	5:A
	0x0F89,		//	6:A
	0x0F89,		//	7:A
	0x0F8A,		//	8:B
	0x0F8A,		//	9:B
	0x0F8A,		//	10:B
	0x0F8A,		//	11:B
	0x0F8A,		//	12:B
	0x0F8A,		//	13:B
	0x0F8A,		//	14:B
	0x0F8A,		//	15:B
	0x0F8B,		//	16:C
	0x0F8B,		//	17:C
	0x0F8B,		//	18:C
	0x0F8B,		//	19:C
	0x0F8B,		//	20:C
	0x0F8B,		//	21:C
	0x0F8B,		//	22:C
	0x0F8B,		//	23:C
	0x0F8C,		//	24:D
	0x0F8C,		//	25:D
	0x0F8C,		//	26:D
	0x0F8C,		//	27:D
	0x0F8C,		//	28:D
	0x0F8C,		//	29:D
	0x0F8C,		//	30:D
	0x0F8C,		//	31:D
	0x0F8D,		//	32:E
	0x0F8D,		//	33:E
	0x0F8D,		//	34:E
	0x0F8D		//	35:E
};
#pragma code
//
// PIC18F46K22 RXPINMAP Constants, note all
// analog ports are defined here
//
// RX Analog ports
#define RA0RX 0		//	AN0
#define RA1RX 1		//	AN1
#define RA2RX 2		//	AN2
#define RA3RX 3		//	AN3
#define RA5RX 4		//	AN4
#define RE0RX 5		//	AN5
#define RE1RX 6		//	AN6
#define RE2RX 7		//	AN7
#define RB2RX 8		//	AN8
#define RB3RX 9		//	AN9
#define RB1RX 10	//	AN10
#define RB4RX 11	//	AN11
#define RB0RX 12	//	AN12
#define RB5RX 13	//	AN13
#define RC2RX 14	//	AN14
#define RC3RX 15	//	AN15
#define RC4RX 16	//	AN16
#define RC5RX 17	//	AN17
#define RC6RX 18	//	AN18
#define RC7RX 19	//	AN19
#define RD0RX 20	//	AN20
#define RD1RX 21	//	AN21
#define RD2RX 22	//	AN22
#define RD3RX 23	//	AN23
#define RD4RX 24	//	AN24
#define RD5RX 25	//	AN25
#define RD6RX 26	//	AN26
#define RD7RX 27	//	AN27
//
// PIC18F46K22 TXPINMAP Constants, note any
// porty can be used as TX
//
#define RA0TX 0
#define RA1TX 1
#define RA2TX 2
#define RA3TX 3
#define RA4TX 4
#define RA5TX 5
#define RA6TX 6
#define RA7TX 7
#define RB0TX 8
#define RB1TX 9
#define RB2TX 10
#define RB3TX 11
#define RB4TX 12
#define RB5TX 13
#define RB6TX 14
#define RB7TX 15
#define RC0TX 16
#define RC1TX 17
#define RC2TX 18
#define RC3TX 19
#define RC4TX 20
#define RC5TX 21
#define RC6TX 22
#define RC7TX 23
#define RD0TX 24
#define RD1TX 25
#define RD2TX 26
#define RD3TX 27
#define RD4TX 28
#define RD5TX 29
#define RD6TX 30
#define RD7TX 31
#define RE0TX 32
#define RE1TX 33
#define RE2TX 34
#define RE3TX 35

//
// define the PORT, LAT, and TRIS maps for this device
//
// GTEMPLAT is used to "AND" the LAT port to clear the LAT
// for pins used for PCAP
//
/*#ifndef PCAPPIC18FDEVBMPK22V5
#define GTEMPLATA 0b11110000
#define GTEMPLATB 0b11111111
#define GTEMPLATC 0b00000011
#define GTEMPLATD 0b00000000
#define GTEMPLATE 0b00000000
#endif // ifndef PCAPPIC18FDEVBMPK22V5

#ifdef PCAPPIC18FDEVBMPK22V5
#define GTEMPLATA 0b11000111
#define GTEMPLATB 0b00111111
#define GTEMPLATC 0b00110011	// note RC4 abnd RC5 reamin high for I2C
#define GTEMPLATD 0b10000000
#define GTEMPLATE 0b00000000
#endif // ifdef PCAPPIC18FDEVBMPK22V5*/
//
// TEMPTRIS are the TRIS bits that are "AND" with the TRIS port
// to set the PCAP port to output
//
/*#ifndef PCAPPIC18FDEVBMPK22V5
#define TEMPTRISA 0b00000000
#define TEMPTRISB 0b11000000
#define TEMPTRISC 0b00000000
#define TEMPTRISD 0b11000000
#define TEMPTRISE 0b11111000
#endif // ifndef PCAPPIC18FDEVBMPK22V5

#ifdef PCAPPIC18FDEVBMPK22V5
#define TEMPTRISA 0b00000000
#define TEMPTRISB 0b11000000
#define TEMPTRISC 0b00011000	// note TRISC3 and C4 are input for I2C
#define TEMPTRISD 0b00000000
#define TEMPTRISE 0b11111000
#endif // ifdef PCAPPIC18FDEVBMPK22V5*/

//
// varible definitions
//
#define PCAPULONG unsigned long	// 32 bits
//
// port mappings
//
//#define I2C_INT LATGbits.LATG7
//#define I2C_INT_TRIS TRISGbits.TRISG7
#define I2C_SCL	TRISCbits.TRISC3
#define I2C_SDA	TRISCbits.TRISC4
#define I2C_INT LATAbits.LATA0
#define I2C_TRIS TRISAbits.TRISA0
#ifdef PCAPPIC18FDEVBMPK22V5
	#undef I2C_INT
	#undef I2C_TRIS
	#define I2C_INT LATDbits.LATD7
	#define I2C_TRIS TRISDbits.TRISD7
#endif // ifdef PCAPPIC18FDEVBMPK22V5
#define _TX TRISDbits.TRISD6
#define _RX TRISDbits.TRISD7
#ifdef DISPLAY
	#define LCD_RST LATDbits.LATD2
	#define LCD_RST_TRIS TRISDbits.TRISD2
	#define LCD_A0 LATDbits.LATD3
	#define LCD_A0_TRIS TRISDbits.TRISD3
	#define SPI_SCK_TRIS TRISCbits.TRISC3
	#define SPI_SDO_TRIS TRISCbits.TRISC5
	#define SPI_CS LATDbits.LATD1
	#define SPI_CS_TRIS TRISDbits.TRISD1
#endif

//
// Constants
//
#ifndef LOW_POWER
	#define T1CONSET500NS 0b00110000;		// FOSC/4 - 1:8 pre-scale 500nS per count @ 64MHZ
	#define T2CONSET250NS 0b00000001; 		// 8 bit 250ns resolution @ 48MHz, 1:4 Prescaler
	#define T6CONSET250NS 0b00000001; 		// 8 bit FOSC/4 = 16 prescale of 4 post scale of 0 = 0.25us res
	#define BAUDLO 0x22
#endif
#ifdef LOW_POWER
	#define T1CONSET500NS 0b01110000;		// FOSC - 1:8 prescaler 500ns per count @ 16MHz
	#define T2CONSET250NS 0b00000000;		// 8 bit 250ns resolution @ 16MHz, 1:1 Prescaler
	#define T6CONSET250NS 0b00000000;		// 8 bit 250ns resolution @ 16MHz, 1:1 prescaler
	#define BAUDLO 0x08
#endif
#define BAUDHI 0x00

//
//CVD macros
//
#define LAT_OFFSET 				0x09

//
//Noise macros
//
#define NOISE_INC_VALUE		1

//CVD macros
#define ADC_CHARGE_TRIS		TRISAbits.TRISA0
#define ADC_CHARGE_LAT		LATAbits.LATA0
#define ADC_CHARGE_CHANNEL	0

//
// macros
//
#define PC_DELAY_PR 		PR2

#define UART_TXREG 			TXREG2

#define UART_TRMT 			TXSTA2bits.TRMT

#define UART_RX_REG 		RCREG2

#define UART_RXIF 			PIR3bits.RC2IF

#define ADC_TAD 			ADCON2bits.ADCS

#define CTMU_IRNG 			CTMUICONbits.IRNG

#define CTMU_EDGE2STAT 		CTMUCONLbits.EDG2STAT

#define CTMU_EDGE1STAT 		CTMUCONLbits.EDG1STAT

#define DELAY_TIMER_IF 		PIR1bits.TMR2IF

#define ADC_CHS 			ADCON0bits.CHS

#ifdef TOUCHPADDEMONSTRATOR
	#define SELECT_PIN			PORTBbits.RB5
	#define SELECT_PIN_TRIS		TRISBbits.TRISB5
	#define SELECT_PIN_LAT		LATBbits.LATB5
#endif

#define setup_delay_tmr() \
	T2CONbits.TMR2ON = 0;			/* turn off TMR2 */ \
	T2CON = T2CONSET250NS;			/* 250nS */

#define start_delay_timer() \
	T2CONbits.TMR2ON = 1;			/* turn on TMR1 */

#define stop_delay_tmr() \
	T2CONbits.TMR2ON = 0;			/* turn off TMR2 */

#define set_ansel_port(x, y)  (x = (unsigned char *)(TRISPORT_LUT[y]-0x5A)) /*(x = (unsigned int *)(ANSELPORT_LUT[y]))*/

#define set_tris_port() \
	varRam.gtempTRISPORT = (unsigned char *)TRISPORT_LUT[newTemp];

#define set_lat_port() \
	varRam.gtempLATPORT[1] = (unsigned char *)LATPORT_LUT[tempVar[1]]; \
	varRam.gtempLATPORT[0] = (unsigned char *)LATPORT_LUT[tempVar[0]];

//#define set_ansel_bit(x, y) (x = y)
#define set_ansel_bit() \
	varRam.gANSELbit = TRIS_LUT[newTemp];

#define setup_self_lat() \
	varRam.gtempLATA = GTEMPLATA; \
	varRam.gtempLATB = GTEMPLATB; \
	varRam.gtempLATC = GTEMPLATC; \
	varRam.gtempLATD = GTEMPLATD; \
	varRam.gtempLATE = GTEMPLATE;

#define setup_self_clear_lat() \
	varRam.gtempLATA = 0; \
	varRam.gtempLATB = 0; \
	varRam.gtempLATC = 0; \
	varRam.gtempLATD = 0; \
	varRam.gtempLATE = 0;

#define setup_self_clear_lat_stutter() \
	varRam.gtempLATAgrp[wave] = 0; \
	varRam.gtempLATBgrp[wave] = 0; \
	varRam.gtempLATCgrp[wave] = 0; \
	varRam.gtempLATDgrp[wave] = 0; \
	varRam.gtempLATEgrp[wave] = 0;

#define setup_self_set_lat_bit() \
	if (LATPORT_LUT[tempVar] == 0x0F89) /* LATA */ \
	{ \
		varRam.gtempLATA |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x0F8A) /* LATB */ \
	{ \
		varRam.gtempLATB |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x0F8B) /* LATC */ \
	{ \
		varRam.gtempLATC |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x0F8C) /* LATD */ \
	{ \
		varRam.gtempLATD |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x0F8D) /* LATE */ \
	{ \
		varRam.gtempLATE |= LAT_LUT[tempVar]; \
	}


#define setup_self_set_lat_bit_stutter() \
	if (LATPORT_LUT[tempVar] == 0x0F89) /* LATA */ \
	{ \
		varRam.gtempLATAgrp[wave] |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x0F8A) /* LATB */ \
	{ \
		varRam.gtempLATBgrp[wave] |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x0F8B) /* LATC */ \
	{ \
		varRam.gtempLATCgrp[wave] |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x0F8C) /* LATD */ \
	{ \
		varRam.gtempLATDgrp[wave] |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x0F8D) /* LATE */ \
	{ \
		varRam.gtempLATEgrp[wave] |= LAT_LUT[tempVar]; \
	} 

#define comm_rcv_timeout_setup() \
	globalCounter = 50;


#define comm_rcv_timeout() \
	if (globalCounter == 0) \
	{ \
		goto UARTreceiveComplete; \
	}

#define comm_rcv_timeout_off() 



#endif //ifndef __PCAP_HARDWARE_PIC18F46K22_DEV_H
