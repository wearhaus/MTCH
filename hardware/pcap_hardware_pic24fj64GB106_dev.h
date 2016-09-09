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

#ifndef __PCAP_HARDWARE_PIC24FJ64GB106_DEV_H
#define __PCAP_HARDWARE_PIC24FJ64GB106_DEV_H

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
#define CONFIGIDHIGH 0x82

//
// Maximum sensor dimensions
//
#ifdef DEVKIT_HARDWARE
	#define MAXRX 16
	#define MAXTX 25
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
void delay100usTMR(unsigned short delay);
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



//
// PIC24FJ64GB106 RXPINMAP Constants, note all
// analog ports are defined here
//
// RX Analog ports
#define RB0RX  0	//	AN0
#define RB1RX  1	//	AN1
#define RB2RX  2	//	AN2
#define RB3RX  3	//	AN3
#define RB4RX  4	//	AN4
#define RB5RX  5	//	AN5
#define RB6RX  6	//	AN6
#define RB7RX  7	//	AN7
#define RB8RX  8	//	AN8
#define RB9RX  9	//	AN9
#define RB10RX 10	//	AN10
#define RB11RX 11	//	AN11
#define RB12RX 12	//	AN12
#define RB13RX 13	//	AN13
#define RB14RX 14	//	AN14
#define RB15RX 15	//	AN15
//
// PIC24FJ64GB106 TXPINMAP Constants, note any
// porty can be used as TX
//
#define RB0TX  0
#define RB1TX  1
#define RB2TX  2
#define RB3TX  3
#define RB4TX  4
#define RB5TX  5
#define RB6TX  6
#define RB7TX  7
#define RB8TX  8
#define RB9TX  9
#define RB10TX 10
#define RB11TX 11
#define RB12TX 12
#define RB13TX 13
#define RB14TX 14
#define RB15TX 15
#define RC13TX 16
#define RC14TX 17
#define RD0TX  18
#define RD1TX  19
#define RD2TX  20
#define RD3TX  21
#define RD4TX  22
#define RD5TX  23
#define RD6TX  24
#define RD7TX  25
#define RD8TX  26
#define RD11TX 27
#define RE0TX  28
#define RE1TX  29
#define RE2TX  30
#define RE3TX  31
#define RE4TX  32
#define RE5TX  33
#define RE6TX  34
#define RE7TX  35
#define RF0TX  36
#define RF1TX  37
#define RF3TX  38
#define RF4TX  39
#define RF5TX  40

//
// define the PORT, LAT, and TRIS maps for this device
//
// GTEMPLAT is used to "AND" the LAT port to clear the LAT
// for pins used for PCAP
//

#define GTEMPLATB 0b1111111111111111
#define GTEMPLATC 0b1001111111111111
#define GTEMPLATD 0b1111011000000001
#define GTEMPLATE 0b1111111100000000
#define GTEMPLATF 0b1111111111000100

//
// TEMPTRIS are the TRIS bits that are "AND" with the TRIS port
// to set the PCAP port to output
//

#define TEMPTRISB 0b0000000000000000
#define TEMPTRISC 0b1001111111111111
#define TEMPTRISD 0b1111011000000001
#define TEMPTRISE 0b1111111100000000
#define TEMPTRISF 0b1111111111000100

//
// varible definitions
//
#define PCAPULONG unsigned long		// 32 bits
//
// port mappings
//
#define I2C_INT LATGbits.LATG7
#define I2C_INT_TRIS TRISGbits.TRISG7

//
//CVD macros
//
#define LAT_OFFSET		0x1

//
//Noise macros
//
#define NOISE_INC_VALUE		2

//CVD macros
#define ADC_CHARGE_TRIS		TRISBbits.TRISB0
#define ADC_CHARGE_LAT		LATBbits.LATB0
#define ADC_CHARGE_CHANNEL	0

//
// macros
//
#define PC_RISE_DELAY_PR 	PR1

#define PC_FREQ_DELAY_PR 	PR2

#define UART_TXREG 			U1TXREG

#define UART_TRMT 			U1STAbits.TRMT

#define UART_RX_REG 		U1RXREG

#define UART_RXIF 			IFS0bits.U1RXIF

#define ADC_TAD 			AD1CON3bits.ADCS

#define CTMU_IRNG 			CTMUICONbits.IRNG

#define CTMU_EDGE2STAT 		CTMUCONbits.EDG2STAT

#define CTMU_EDGE1STAT 		CTMUCONbits.EDG1STAT

#define setup_samp_risetime_tmr() \
	T1CONbits.TON = 0;				/* turn off TMR1 */ \
	TMR1 = 0; \
	T1CON = 0b0010000000000000;		/* Prescaler=1:1 --- Clk Freq=16MHz(FOSC/2) --- Period=62.5ns */

#define start_samp_risetime_tmr() \
	T1CONbits.TON = 1;				/* turn on TMR1 */

#define setup_samp_freq_tmr() \
	T2CONbits.TON = 0;				/* turn off TMR2 */ \
	TMR2 = 0; \
	T2CON = 0b0000000000000000;		/* Prescaler=1:1 --- Clk Freq=16MHz(FOSC/2) --- Period=62.5ns */

#define start_samp_freq_timer() \
	T2CONbits.TON = 1;				/* turn on TMR2 */

#define stop_samp_risetime_tmr() \
	T1CONbits.TON = 0;				/* turn off TMR1 */

#define stop_samp_freq_tmr() \
	T2CONbits.TON = 0;				/* turn off TMR2 */

#define SAMPLE_TIMER_IF IFS0bits.T2IF

#define set_ansel_port(x, y) (x = &AD1PCFGL)

#define set_tris_port() \
	varRam.gtempTRISPORT = (unsigned int *)TRISPORT_LUT[newTemp];

#define set_lat_port() \
	varRam.gtempLATPORT[1] = (unsigned int *)LATPORT_LUT[tempVar[1]]; \
	varRam.gtempLATPORT[0] = (unsigned int *)LATPORT_LUT[tempVar[0]];

//#define set_ansel_bit(x, y) (x = ~(y))
#define set_ansel_bit() \
	varRam.gANSELbit = ~(TRIS_LUT[newTemp]);

#define ADC_CHS AD1CHSbits.CH0SA

#define setup_self_lat() \
	varRam.gtempLATC = ~(GTEMPLATC); \
	varRam.gtempLATD = ~(GTEMPLATD); \
	varRam.gtempLATE = ~(GTEMPLATE); \
	varRam.gtempLATF = ~(GTEMPLATF);

#define setup_self_clear_lat() \
	varRam.gtempLATC = 0; \
	varRam.gtempLATD = 0; \
	varRam.gtempLATE = 0; \
	varRam.gtempLATF = 0;

#define setup_self_clear_lat_stutter() \
	varRam.gtempLATCgrp[wave] = 0; \
	varRam.gtempLATDgrp[wave] = 0; \
	varRam.gtempLATEgrp[wave] = 0; \
	varRam.gtempLATFgrp[wave] = 0;

#define setup_self_set_lat_bit() \
	if (LATPORT_LUT[tempVar] == 0x02d4) /* LATC */ \
	{ \
		varRam.gtempLATC |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x02dc) /* LATD */ \
	{ \
		varRam.gtempLATD |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x02e4) /* LATE */ \
	{ \
		varRam.gtempLATE |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x02ec) /* LATF */ \
	{ \
		varRam.gtempLATF |= LAT_LUT[tempVar]; \
	} \

#define setup_self_set_lat_bit_stutter() \
	if (LATPORT_LUT[tempVar] == 0x02d4) /* LATC */ \
	{ \
		varRam.gtempLATCgrp[wave] |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x02dc) /* LATD */ \
	{ \
		varRam.gtempLATDgrp[wave] |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x02e4) /* LATE */ \
	{ \
		varRam.gtempLATEgrp[wave] |= LAT_LUT[tempVar]; \
	} \
	if (LATPORT_LUT[tempVar] == 0x02ec) /* LATF */ \
	{ \
		varRam.gtempLATFgrp[wave] |= LAT_LUT[tempVar]; \
	} \

#define comm_rcv_timeout_setup() \
	T1CON = 0b0010000000110000;	/* Prescaler=1:256 --- Clk Freq=16MHz(FOSC/2) --- Period=16us */ \
	TMR1 = 0; \
	PR1 = 1562;				/* 25mS timeout */ \
	IFS0bits.T1IF = 0; \
	T1CONbits.TON = 1;		/* turn on TMR1 */

#define comm_rcv_timeout() \
	if (IFS0bits.T1IF) \
	{ \
		timeout = 1; \
		goto UARTreceiveComplete; \
	}

#define comm_rcv_timeout_off() \
	T1CONbits.TON = 0;		/* turn on TMR1 */

#endif //ifndef __PCAP_HARDWARE_PIC24FJ64GB106_DEV_H
