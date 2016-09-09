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
* jgh                  12/19/11    Specific hardware for the PIC32MX250F128D Dev board
******************************************************************************/

#ifndef __PCAP_HARDWARE_PIC32MX120F032D_DEV_H
#define __PCAP_HARDWARE_PIC32MX120F032D_DEV_H

#include <p32xxxx.h>
#include "plib.h"

//
// configuration bits
//
// SYSCLK = 38 MHz (8MHz FRC/ FPLLIDIV * FPLLMUL / FPLLODIV)
// SYSCLK = 8MHZ/2 * 19 / 2 = 38MHZ
// SYSCLK = 8MHZ/2 * 16 / 2 = 32MHZ
// PBCLK(TPB) = 32MHZ
// PBCLK = 32 MHz
//#define set_config_bits()

//#define BOOTLOADER_ADDRESS 0x9FC00000

//#define I2CDEBUGIRQ  // For the "blue wire" debug fix


//
// Maximum sensor dimensions
//
#if defined(DEVKIT_HARDWARE) || defined(MTC6301)
	#define MAXRX 13
	#define MAXTX 18
#else // DEVKIT_HARDWARE
	#define MAXRX 			NUMBEROFRXCHANNELS
	#define MAXTX 			NUMBEROFTXCHANNELS
#endif // DEVKIT_HARDWARE

//
// headers
//
void hardwarePreInit(void);
void hardwarePostInit(void);
unsigned char readEEPROMToRAM(unsigned int addr);
unsigned char writeRAMToEEPROM(unsigned int addr);
unsigned char EE_Write(unsigned int *data, unsigned char size, unsigned int** addr);
unsigned char EE_Read(unsigned int *data, unsigned char size, unsigned int** addr);
unsigned char eraseEEPROM(unsigned int addr);
void delay100usTMR(unsigned short delay);
void delay1ms(unsigned short delay);
void initPCHardware(void);
void groundAll(void);
void preInitSelf(void);
void selfCVDScan(unsigned char y);
void preInitMutual(void);
void mutualCVDScan(unsigned char y);
void initI2C(void);
void readI2C(void);
void writeI2C(void);
void initUART(void);
unsigned char checkIO(unsigned char withComm);
void peripheralShutdown(void);
void peripheralWakeup(void);
void goToSleep(void);

void initLCDSPI(void);
void inline sendSPIchar(unsigned char c);
void inline waitOnSPITX(void);
void inline sendSPIstring(const BYTE *c, int size);

void updateHardwareCfg(void);

//
// PIC32MX250F128D RXPINMAP Constants, note all
// analog ports are defined here
//
//
// RX Analog ports
//
#define RA0RX  0	//	AN0
#define RA1RX  1	//	AN1
#define RB0RX  2	//	AN2
#define RB1RX  3	//	AN3
#define RB2RX  4	//	AN4
#define RB3RX  5	//	AN5
#define RC0RX  6	//	AN6
#define RC1RX  7	//	AN7
#define RC2RX  8	//	AN8
#define RB15RX 9	//	AN9
#define RB14RX 10	//	AN10
#define RB13RX 11	//	AN11
#define RB12RX 12	//	AN12
//
// TX digital Ports
//
#define RA0TX  0
#define RA1TX  1
#define RA2TX  2
#define RA3TX  3
#define RA4TX  4
#define RA7TX  5
#define RA8TX  6
#define RA9TX  7
#define RA10TX 8
#define RB0TX  9
#define RB1TX  10
#define RB2TX  11
#define RB3TX  12
#define RB4TX  13
#define RB5TX  14
#define RB6TX  15
#define RB7TX  16
#define RB8TX  17
#define RB9TX  18
#define RB10TX 19
#define RB11TX 20
#define RB12TX 21
#define RB13TX 22
#define RB14TX 23
#define RB15TX 24
#define RC0TX  25
#define RC1TX  26
#define RC2TX  27
#define RC3TX  28
#define RC4TX  29
#define RC5TX  30
#define RC6TX  31
#define RC7TX  32
#define RC8TX  33
#define RC9TX  34

//
// varible definitions
//
#define PCAPULONG unsigned int		// 32 bits
//
// port mappings
//
#ifdef I2CDEBUGIRQ
	#define I2C_IRQ LATCbits.LATC6
	#define I2C_IRQ_TRIS TRISCbits.TRISC6
#else
	#define I2C_IRQ LATBbits.LATB10
	#define I2C_IRQ_TRIS TRISBbits.TRISB10
#endif // I2CDEBUGIRQ

#define LCD_RST LATBbits.LATB6
#define LCD_RST_TRIS TRISBbits.TRISB6
#define LCD_A0 LATBbits.LATB7
#define LCD_A0_TRIS TRISBbits.TRISB7
#define SPI_SCK_TRIS TRISBbits.TRISB14
#define SPI_SDO_TRIS TRISBbits.TRISB5
#define SPI_CS LATCbits.LATC5
#define SPI_CS_TRIS TRISCbits.TRISC5

//
//CVD charge pin configurations
//
#define CVD_CHARGE_CHS1  	8
#define CVD_CHARGE_TRIS1  	TRISCbits.TRISC2
#define CVD_CHARGE_LAT1  	LATCbits.LATC2

#define CVD_CHARGE_CHS2  	12
#define CVD_CHARGE_TRIS2  	TRISBbits.TRISB12
#define CVD_CHARGE_LAT2  	LATBbits.LATB12

//
//CVD macros
//
#define LAT_OFFSET	0x8

#define _ERROR 1
#define _SUCCESS 0

//
//Noise macros
//
#define NOISE_INC_VALUE 1

#define SELECT_PIN			PORTCbits.RC3
#define SELECT_PIN_TRIS		TRISCbits.TRISC3
#define SELECT_PIN_LAT		LATCbits.LATC3

//#define DIAG_A_SET	TRISCCLR = 0x0040; LATCSET = 0x0040;
//#define DIAG_A_CLR  TRISCCLR = 0x0040; LATCCLR = 0x0040;
#define DIAG_B_SET  TRISACLR = 0x0400; LATASET = 0x0400;
#define DIAG_B_CLR  TRISACLR = 0x0400; LATACLR = 0x0400;
//#define DIAG_A_TOGGLE	TRISCCLR = 0x0040; LATCbits.LATC6^=1;
//
// macros
//
#define PC_DELAY_PR 		PR2

#define UART_TXREG 			U1TXREG

#define UART_TRMT 			U1STAbits.TRMT

#define UART_RX_REG 		U1RXREG

#define UART_RXIF 			IFS0bits.U1RXIF

#define ADC_TAD 			AD1CON3bits.ADCS

#define setup_delay_tmr() \
	T2CONCLR = 0x00008000;			/* turn off TMR2 */ \
	T2CON = 0x00000010;				/* 1:2 prescale 62.5ns @ 32MHZ */

#define start_delay_timer() \
	T2CONSET = 0x00008000;			/* turn on TMR2 */

#define stop_delay_tmr() \
	T2CONCLR = 0x00008000;			/* turn off TMR2 */

#define DELAY_TIMER_IF IFS0bits.T2IF

#define set_ansel_port(x, y) (x = (unsigned int *)(TRISPORT_LUT[y]-0x10))

#define set_tris_port() \
	varRam.gtempTRISPORT = (unsigned int *)TRISPORT_LUT[newTemp];

#define set_lat_port() \
			varRam.gtempLATPORT[1] = (unsigned int *)LATPORT_LUT[tempVar[1]]; \
	varRam.gtempLATPORT[0] = (unsigned int *)LATPORT_LUT[tempVar[0]];

//#define set_ansel_bit(x, y) (x = y)
#define set_ansel_bit() \
	varRam.gANSELbit = TRIS_LUT[newTemp];

#define ADC_CHS AD1CHSbits.CH0SA

#define setup_self_lat() \
	varRam.gtempLATA = GTEMPLATA; \
	varRam.gtempLATB = GTEMPLATB; \
	varRam.gtempLATC = GTEMPLATC;

#define setup_self_clear_lat() \
	varRam.gtempLATA = 0; \
	varRam.gtempLATB = 0; \
	varRam.gtempLATC = 0;

#define setup_self_clear_lat_stutter() \
	varRam.gtempLATAgrp[wave] = 0; \
	varRam.gtempLATBgrp[wave] = 0; \
	varRam.gtempLATCgrp[wave] = 0;

#define setup_self_set_lat_bit() \
	if (LATPORT_LUT[tempVar] == 0xbf886030) /* LATA */ \
	{ \
		varRam.gtempLATA |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0xbf886130) /* LATB */ \
	{ \
		varRam.gtempLATB |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0xbf886230) /* LATC */ \
	{ \
		varRam.gtempLATC |= LAT_LUT[tempVar]; \
	}

#define setup_self_set_lat_bit_stutter() \
	if (LATPORT_LUT[tempVar] == 0xbf886030) /* LATA */ \
	{ \
		varRam.gtempLATAgrp[wave] |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0xbf886130) /* LATB */ \
	{ \
		varRam.gtempLATBgrp[wave] |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0xbf886230) /* LATC */ \
	{ \
		varRam.gtempLATCgrp[wave] |= LAT_LUT[tempVar]; \
	}

#define comm_rcv_timeout_setup() \
	globalCounter = 50;

#define comm_rcv_timeout() \
	if (globalCounter==0) \
	{ \
		timeout = 1; \
		goto receiveComplete; \
	}

#define comm_rcv_timeout_off() \
	//T1CONbits.TON = 0;		/* turn off TMR1 */


#endif //ifndef __PCAP_HARDWARE_PIC32MX120F032D_DEV_H
