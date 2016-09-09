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
* jgh                  12/19/11    Specific hardware for the PIC24FJ64GB106 Dev board
******************************************************************************/

#ifndef __PCAP_HARDWARE_PIC24FJ64GB004_DEV_H
#define __PCAP_HARDWARE_PIC24FJ64GB004_DEV_H

#include "p24fxxxx.h"
#include <libpic30.h>
#include "i2c.h"

#ifdef USB
	//
	// USB includes
	//
	#include "usb\GenericTypeDefs.h"
	#include "usb\Compiler.h"
	#include "usb\USB.h"
	#include "usb\usb_config.h"
	#include "usb\HardwareProfile.h"
	#include "usb\usb_common.h"
	#include "usb\usb_device.h"
#endif
#include "eeprom\DEE Emulation 16-bit.h"

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
#define CONFIGIDHIGH 0x84

//
// Maximum sensor dimensions
//
#ifdef DEVKIT_HARDWARE
	#define MAXRX 13
	#define MAXTX 18
#else // DEVKIT_HARDWARE
	#define MAXRX NUMBEROFRXCHANNELS
	#define MAXTX NUMBEROFTXCHANNELS
#endif // DEVKIT_HARDWARE
//
// headers
//
void hardwarePreInit(void);
void hardwarePostInit(void);
void readEEPROMToRAM(void);
void writeRAMToEEPROM(void);
void eraseEEPROM(void);
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
void checkIO(void);
void peripheralShutdown(void);
void peripheralWakeup(void);
void goToSleep(void);


void initLCDSPI(void);
void inline sendSPIchar(unsigned char c);
void inline waitOnSPITX(void);
void inline sendSPIstring(const BYTE *c, int size);


//
// PIC24FJ64GB004 RXPINMAP Constants, note all
// analog ports are defined here
//
// RX Analog ports
#define RA0RX  0		//	AN0
#define RA1RX  1		//	AN1
#define RB0RX  2		//	AN2
#define RB1RX  3		//	AN3
#define RB2RX  4		//	AN4
#define RB3RX  5		//	AN5
#define RC0RX  6		//	AN6
#define RC1RX  7		//	AN7
#define RC2RX  8		//	AN8
#define RB15RX 9		//	AN9
#define RB14RX 10		//	AN10
#define RB13RX 11		//	AN11
#define RC3RX  12		//	AN12
//
// PIC24FJ64GB004 TXPINMAP Constants, note any
// port can be used as TX
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
#define RB7TX  15
#define RB8TX  16
#define RB9TX  17
#define RB10TX 18
#define RB11TX 19
#define RB13TX 20
#define RB14TX 21
#define RB15TX 22
#define RC0TX  23
#define RC1TX  24
#define RC2TX  25
#define RC3TX  26
#define RC4TX  27
#define RC5TX  28
#define RC6TX  29
#define RC7TX  30
#define RC8TX  31
#define RC9TX  32



//
// define the PORT, LAT, and TRIS maps for this device
//
// GTEMPLAT is used to "AND" the LAT port to clear the LAT
// for pins used for PCAP
//
#define GTEMPLATA 0b1111100001100011
#define GTEMPLATB 0b1111000001001111
#define GTEMPLATC 0b1111110000001111

//
// TEMPTRIS are the TRIS bits that are "AND" with the TRIS port
// to set the PCAP port to output
//
#define TEMPTRISA 0b1111100001100000
#define TEMPTRISB 0b0001000001000000
#define TEMPTRISC 0b1111110000000000

//
// varible definitions
//
#define PCAPULONG unsigned long		// 32 bits
//
// port mappings
//
//#define I2C_INT LATGbits.LATG7
//#define I2C_INT_TRIS TRISGbits.TRISG7

//
//CVD macros
//
#define LAT_OFFSET 0x1

//
//Noise macros
//
#define NOISE_INC_VALUE 1

//CVD Macros
#define ADC_CHARGE_TRIS		TRISCbits.TRISC3
#define ADC_CHARGE_LAT		LATCbits.LATC3
#define ADC_CHARGE_CHANNEL	12
#define ADC_CHARGE_ANSEL	AD1PCFGLbits.PCFG12
#define NO_CHANNEL			16

//
// macros
//
#define PC_DELAY_PR 		PR2

#define UART_TXREG 			U1TXREG

#define UART_TRMT 			U1STAbits.TRMT

#define UART_RX_REG 		U1RXREG

#define UART_RXIF 			IFS0bits.U1RXIF

#define ADC_TAD 			AD1CON3bits.ADCS

#define CTMU_IRNG 			CTMUICONbits.IRNG

#define CTMU_EDGE2STAT 		CTMUCONbits.EDG2STAT

#define CTMU_EDGE1STAT 		CTMUCONbits.EDG1STAT

#ifdef TOUCHPADDEMONSTRATOR
	#define SELECT_PIN			PORTBbits.RB5
	#define SELECT_PIN_TRIS		TRISBbits.TRISB5
	#define SELECT_PIN_LAT		LATBbits.LATB5
#endif

#define setup_delay_tmr() \
	T2CONbits.TON = 0;				/* turn off TMR2 */ \
	T2CON = 0b0000000000000000;		/* 1:1 - prescale = 0.125uS @ 32MHZ */

#define start_delay_timer() \
	T2CONbits.TON = 1;				/* turn on TMR2 */

#define stop_delay_tmr() \
	T2CONbits.TON = 0;				/* turn off TMR1 */

#define DELAY_TIMER_IF IFS0bits.T2IF

#define set_ansel_port(x, y) (x = &AD1PCFG)

#define set_tris_port() \
	varRam.gtempTRISPORT = (unsigned int *)TRISPORT_LUT[newTemp];

#define set_lat_port() \
			varRam.gtempLATPORT[1] = (unsigned int *)LATPORT_LUT[tempVar[1]]; \
	varRam.gtempLATPORT[0] = (unsigned int *)LATPORT_LUT[tempVar[0]];

//#define set_ansel_bit(x, y) (x = ~(y))
#define set_ansel_bit() \
	varRam.gANSELbit = 0xfffe; \
	while(newTemp>0) \
	{ \
		varRam.gANSELbit <<= 1; \
		newTemp --; \
	}


#define ADC_CHS AD1CHSbits.CH0SA

#define setup_self_lat() \
	varRam.gtempLATA = ~(GTEMPLATA); \
	varRam.gtempLATB = ~(GTEMPLATB); \
	varRam.gtempLATC = ~(GTEMPLATC);

#define setup_self_clear_lat() \
	varRam.gtempLATA = 0; \
	varRam.gtempLATB = 0; \
	varRam.gtempLATC = 0;

#define setup_self_clear_lat_stutter() \
	varRam.gtempLATAgrp[wave] = 0; \
	varRam.gtempLATBgrp[wave] = 0; \
	varRam.gtempLATCgrp[wave] = 0;

#define setup_self_set_lat_bit() \
	if (LATPORT_LUT[tempVar] == 0x02c4) /* LATA */ \
	{ \
		varRam.gtempLATA |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x02cc) /* LATB */ \
	{ \
		varRam.gtempLATB |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x02d4) /* LATC */ \
	{ \
		varRam.gtempLATC |= LAT_LUT[tempVar]; \
	} \

#define setup_self_set_lat_bit_stutter() \
	if (LATPORT_LUT[tempVar] == 0x02c4) /* LATA */ \
	{ \
		varRam.gtempLATAgrp[wave] |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x02cc) /* LATB */ \
	{ \
		varRam.gtempLATBgrp[wave] |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x02d4) /* LATC */ \
	{ \
		varRam.gtempLATCgrp[wave] |= LAT_LUT[tempVar]; \
	} \

#define comm_rcv_timeout_setup() \
	globalCounter = 50;

#define comm_rcv_timeout() \
	if(globalCounter == 0) \
	{ \
		timeout = 1; \
		goto UARTreceiveComplete; \
	} 

#define comm_rcv_timeout_off() \

#endif //ifndef __PCAP_HARDWARE_PIC24FJ64GB004_DEV_H
