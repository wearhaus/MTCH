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

#include "main\main.h"
#ifdef DISPLAY
	#include "display\lcddisplay.h"
#endif
#include "hardware\pcap_hardware_pic18f46k22_dev.h"

//
// Globals
//
extern USERRAM userRam;
extern NOISERAM selfNoise;
extern NOISERAM mutNoise;
extern COMMRAM commRam;
extern RAWRAM rawRam;
extern VARRAM varRam;
extern BASERAM baseRam;
#ifdef LONGADCSCAN
extern unsigned short longADC[512];
#endif

#ifdef DISPLAY
extern LCDBUFFER lcdBuffer;
#endif
extern GESTURE gesture[MAX_TOUCHES];
extern unsigned char sleepFlag;
extern unsigned short baselineCounter;
extern unsigned short sleepCount;
extern volatile unsigned short globalCounter;

extern unsigned char selfScanFineTune[MAXRX];

#pragma config FOSC = INTIO67//INTIO7//INTIO67
#pragma config PLLCFG = OFF		//OFF = software-controlled via PLLEN bit(OSCTUNE<6>)
#pragma config PRICLKEN = ON
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config PWRTEN = OFF
#pragma config BOREN = OFF
#pragma config WDTEN = SWON
#pragma config WDTPS = 64
#pragma config HFOFST = OFF
#pragma config MCLRE = EXTMCLR
#pragma config STVREN = OFF
#pragma config LVP = OFF
#pragma config XINST = ON
#pragma config DEBUG = ON
#pragma config CP0 = OFF
#pragma config CP1 = OFF
#pragma config CP2 = OFF
#pragma config CP3 = OFF
#pragma config CPB = OFF
#pragma config CPD = OFF
#pragma config WRT0 = OFF
#pragma config WRT1 = OFF
#pragma config WRT2 = OFF
#pragma config WRT3 = OFF
#pragma config WRTC = OFF
#pragma config WRTB = OFF
#pragma config WRTD = OFF
#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
#pragma config EBTR3 = OFF
#pragma config EBTRB = OFF

#pragma code InterruptVectorHigh = 0x08
void InterruptVectorHigh (void)
{
  _asm
    goto InterruptHandlerHigh //jump to interrupt routine
  _endasm
}


//
// Interrupts
//
/******************************************************************************
* Function:        	void InterruptHandlerHigh()
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	PIC18F I2C/UART interrupt functions
*
* Note:
*
*****************************************************************************/
#pragma interrupt InterruptHandlerHigh
void InterruptHandlerHigh()
{
	unsigned char x;

	#ifdef UART
	if (PIR3bits.RC2IF && PIE3bits.RC2IE)
	{
		if (commRam.rcvCount > (unsigned char)RCVBUFFERSIZE)
		{
			commRam.rcvCount = 0;
		}
		commRam.rcvBuffer[commRam.rcvCount] = RCREG2;
		commRam.rcvCount ++;
		PIR3bits.RC2IF = 0;
	}
	#endif // ifdef UART

	#ifdef I2C
	if (PIR1bits.SSP1IF && PIE1bits.SSP1IE)
	{
		readI2C();
		PIR1bits.SSP1IF = 0;
	}
	#endif //ifdef I2C

	if (INTCONbits.TMR0IF && INTCONbits.TMR0IE)
	{
		if(globalCounter > 0)
		{
			globalCounter--;
		}

		if(baselineCounter < 65535)
		{
			baselineCounter++;
		}

		for(x=0;x<MAX_TOUCHES;x++)
		{
			if(gesture[x].timer < 65535)
			{
				gesture[x].timer++;
			}
		}

		sleepCount++;
		if(sleepCount > 8000)		//sleep if no touch for 8s
		{
			sleepCount = 0;
			sleepFlag = 1;
		}

		if(selfNoise.timerCount >= 0)
		{
			selfNoise.timerCount++;
			if(selfNoise.timerCount > 1000)
			{
				selfNoise.timerCount = -1;
				selfNoise.noisePresent = 0;
				selfNoise.triedFrequencies = 0;
			}
		}

		if(mutNoise.timerCount >= 0)
		{
			mutNoise.timerCount++;
			if(mutNoise.timerCount > 1000)
			{
				mutNoise.timerCount = -1;
				mutNoise.noisePresent = 0;
				mutNoise.triedFrequencies = 0;
			}
		}
#ifdef DISPLAY
		if (lcdBuffer.guiGlyphTimer > 0)
		{
			lcdBuffer.guiGlyphState = GS_CLEAR;
			lcdBuffer.guiGlyphTimer--;
		}
#endif
		TMR0L = 0x05;
		INTCONbits.TMR0IF = 0;
	}

	//
	// Set the interrupt flag
	//
	varRam.flag |= INTERRUPTOCCURRED;
}	//This return will be a "retfie fast", since this is in a #pragma interrupt section

//
// functions
//
/******************************************************************************
* Function:        	void hardwarePreInit(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Initialize any processor specific functions
*
* Note:
*
*****************************************************************************/
void hardwarePreInit(void)
{
	//
	// OSC
	//
	OSCCON = 0b01110100;

	#ifndef LOW_POWER
		OSCTUNE = 0b11000000;		//PLL on
	#endif
	#ifdef LOW_POWER
		OSCTUNE = 0b10000000;		//PLL off
	#endif

	//
	// Enable interrupts
	//
	//RCONbits.IPEN = 1; // high priority ints
	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;
}

/******************************************************************************
* Function:        	void hardwarePostInit(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Initialize any processor specific functions
*
* Note:
*
*****************************************************************************/
void hardwarePostInit(void)
{
	//
	// Enable interrupts
	//
	//RCONbits.IPEN = 1; // high priority ints
	// INTCONbits.PEIE = 1;
	// INTCONbits.GIE = 1;
	SLRCONbits.SLRA = 0;
	SLRCONbits.SLRB = 0;
	SLRCONbits.SLRD = 0;
	SLRCONbits.SLRE = 0;
}


/******************************************************************************
* Function:        	void delay100usT1(unsigned short delay)
*
* PreCondition:    	None
*
* Input:           	delay 1 = 100us, 2 = 200us..
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	100us delay
*
* Note:
*
*****************************************************************************/
void delay1ms(unsigned short delay)
{
	globalCounter = delay;
	while(globalCounter > 0){}
}

/******************************************************************************
* Function:        	void initPCHardware(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Initializes the projected capacitance
*
* Note:
*
*****************************************************************************/
void initPCHardware(void)
{
	//
	// init the ADC
	//
	ADCON0bits.ADON = 0;			// turn off ADC
	ADCON0 = 0b00000000;
	ADCON1 = 0b10000000;
	ADCON2 = 0b10000000;			//0b10000010;
	#ifndef LOW_POWER
		ADCON2bits.ADCS = 2 ;	// 2=500ns, 6 = 1us
	#endif
	#ifdef LOW_POWER
		ADCON2bits.ADCS = 1;	// 1 = 500ns, 5 = 1us
	#endif
	ADCON0bits.ADON = 1;			// turn on ADC

	//
	// setup the CTMU
	//
	CTMUCONHbits.CTMUEN = 0;		// make sure CTMU disabled
	CTMUCONH = 0b00000000;			//set up CTMU for 100x base current (55 uA)
	CTMUCONL = 0b10010000;			//
	CTMUICON = 0b01111111;			//
	CTMUCONHbits.CTMUEN = 1;		// Enable CTMU

	//
	// set up TMR0 as noise timer
	//
	T0CONbits.TMR0ON = 0;
	T0CON = 0b01000101;			//8-bit, 1:64 prescaler, 4us resolution
	INTCONbits.TMR0IE = 1;
	INTCON2bits.TMR0IP = 1;
	INTCONbits.TMR0IF = 0;
	TMR0L = 0x05;
	T0CONbits.TMR0ON = 1;

	//
	// run a 32ms delay to let everything warm up
	//

	delay1ms(32);
}

/******************************************************************************
* Function:        	void groundAll(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	All pins used in projected measurement are logic grounded
*					in this routine
*
* Note:
*
*****************************************************************************/
void groundAll(void)
{
	ANSELA &= TEMPTRISA;	//
	ANSELB &= TEMPTRISB;	//
	ANSELC &= TEMPTRISC;	// All ANSEL bits for TX or RX pins should be configured as digital output
	ANSELD &= TEMPTRISD;	//
	ANSELE &= TEMPTRISE; 	//

	TRISA &= TEMPTRISA;		//
	TRISB &= TEMPTRISB;		//
	TRISC &= TEMPTRISC; 	// All TRIS bits for TX or RX pins should be configured as outputs
	TRISD &= TEMPTRISD;		//
	TRISE &= TEMPTRISE;		//

	ANSELDbits.ANSD7=0;
	#ifdef UART
	_TX = 0;				// TX bit (RD6) must always be cleared
	_RX = 1;				// RX bit (RD7) must always be set
	#endif
	#ifdef USB
	_TX = 0;				// TX bit (RD6) must always be cleared
	_RX = 1;				// RX bit (RD7) must always be set
	#endif

	#ifdef I2C
	ANSELCbits.ANSC4=0;
	ANSELCbits.ANSC3=0;
	I2C_SCL = 1;			// special case for remote sensor - these bits are tied to TX and RX
	I2C_SDA = 1;
	#endif
#ifdef UARTONI2C // used for specific implementations where the UART is configured on the I2C lines

	ANSELCbits.ANSC4=0;
	ANSELCbits.ANSC3=0;
	I2C_SCL = 1;			// special case for remote sensor - these bits are tied to TX and RX
	I2C_SDA = 1;
#endif

}

/******************************************************************************
* Function:        	void preInitSelf(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Run any special inits for the processor
*
* Note:
*
*****************************************************************************/
void preInitSelf(void)
{

}

/******************************************************************************
* Function:        	void selfCTMUScan(unsigned char y)
*
* PreCondition:    	gtempTRISPORT - the TRIS port address, gtempTRIS - the
*					TRIS mask, gtempTRISClear - the TRIS clear mask
*
* Input:           	y - rawADC Array location
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Run 1 self ADC measuement using one of the PORTD inputs
*
* Note:            	Assume the ADC channel was setup prior to this call
*
*****************************************************************************/
void selfCTMUScan(unsigned char y)
{

	unsigned char i; //iterate through stuttering waves

	rawRam.rawADC[y] = 0; //initialize this since it will be an accumulator


	*(varRam.gANSEL) = varRam.gANSELbit;
	ADCON0bits.CHS = varRam.gADCONCHS;

	for(i=0;i<userRam.stutterMult;i++) //iterate through all stuttering waves
	{
		varRam.gtempLATA = varRam.gtempLATAgrp[i];
		varRam.gtempLATB = varRam.gtempLATBgrp[i];
		varRam.gtempLATC = varRam.gtempLATCgrp[i];
		varRam.gtempLATD = varRam.gtempLATDgrp[i];
		#ifndef PCAPPIC18FDEVBMPK22V5
		varRam.gtempLATE = varRam.gtempLATEgrp[i];
		#endif

		*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;
		Nop();
		Nop();
		Nop();
		CTMUCONHbits.IDISSEN = 1;	// discharge CTMU
		Nop();
		Nop();
		Nop();
		CTMUCONHbits.IDISSEN = 0;	// stop CTMU discharge
		//
		// Drive TX's
		//
		LATA |= varRam.gtempLATA;
		LATB |= varRam.gtempLATB;
		LATC |= varRam.gtempLATC;
		LATD |= varRam.gtempLATD;
		#ifndef PCAPPIC18FDEVBMPK22V5
		LATE |= varRam.gtempLATE;
		#endif

		TMR2 = 0;					// clear TMR2
		Nop();
		PIR1bits.TMR2IF = 0;		// clear TMR2 interrupt
		while (!PIR1bits.TMR2IF){}	// wait appropritae rise time

		ADCON0bits.DONE = 1;
		while(ADCON0bits.DONE){}

		rawRam.rawADC[y]+=ADRES;

		*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;
		LATA &= ~varRam.gtempLATA;
		LATB &= ~varRam.gtempLATB;
		LATC &= ~varRam.gtempLATC;
		LATD &= ~varRam.gtempLATD;
		#ifndef PCAPPIC18FDEVBMPK22V5
		LATE &= ~varRam.gtempLATE;
		#endif
	}
}

/******************************************************************************
* Function:        	void selfCVDScan(unsigned char y)
*
* PreCondition:    	gtempTRISPORT - the TRIS port address, gtempTRIS - the
*					TRIS mask, gtempTRISClear - the TRIS clear mask
*
* Input:           	y - rawADC Array location
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Run 1 self ADC measuement using one of the PORTD inputs
*
* Note:            	Assume the ADC channel was setup prior to this call
*
*****************************************************************************/
void selfCVDScan(unsigned char y)
{
	unsigned char i; //iterate through stuttering waves

	rawRam.rawADC[y] = 0; //initialize this since it will be an accumulator

	for(i=0;i<userRam.stutterMult;i++) //iterate through all stuttering waves
	{
		varRam.gtempLATA = varRam.gtempLATAgrp[i];
		varRam.gtempLATB = varRam.gtempLATBgrp[i];
		varRam.gtempLATC = varRam.gtempLATCgrp[i];
		varRam.gtempLATD = varRam.gtempLATDgrp[i];
		#ifndef PCAPPIC18FDEVBMPK22V5
		varRam.gtempLATE = varRam.gtempLATEgrp[i];
		#endif



		*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;
		*(varRam.gANSEL) = varRam.gANSELbit;
		ANSELAbits.ANSA0 = 1;

		ADC_CHARGE_TRIS = 0;
		ADC_CHARGE_LAT = 1;
		ADCON0bits.CHS = ADC_CHARGE_CHANNEL;

		Nop();

		ADCON0bits.CHS = varRam.gADCONCHS;

		//
		// Drive TX's
		//
		LATA |= varRam.gtempLATA;
		LATB |= varRam.gtempLATB;
		LATC |= varRam.gtempLATC;
		LATD |= varRam.gtempLATD;
		#ifndef PCAPPIC18FDEVBMPK22V5
		LATE |= varRam.gtempLATE;
		#endif

		ADCON0bits.DONE = 1;
		while(ADCON0bits.DONE){}

		rawRam.rawADC[y] += ADRES;

		*(varRam.gtempTRISPORT-LAT_OFFSET) |= varRam.gtempTRIS;		//turn sense channel output driver high
		*(varRam.gtempTRISPORT) &= ~varRam.gtempTRIS;				//turn sense channel to output
		Nop();
		Nop();
		Nop();
		*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;
		ADCON0bits.CHS = varRam.gADCONCHS;
		LATA &= ~varRam.gtempLATA;
		LATB &= ~varRam.gtempLATB;
		LATC &= ~varRam.gtempLATC;
		LATD &= ~varRam.gtempLATD;
		#ifndef PCAPPIC18FDEVBMPK22V5
		LATE &= ~varRam.gtempLATE;
		#endif

		ADCON0bits.DONE = 1;
		while(ADCON0bits.DONE){}

		*(varRam.gtempTRISPORT-LAT_OFFSET) &= ~varRam.gtempTRIS;
		*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;

		ADC_CHARGE_LAT = 0;

		rawRam.rawADC[y] += (1023 - ADRES);
	}
}

/******************************************************************************
* Function:        	void preInitMutual(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Run any special inits for the processor
*
* Note:
*
*****************************************************************************/
void preInitMutual(void)
{

}

/******************************************************************************
* Function:        	void mutualCTMUScan(unsigned char y)
*
* PreCondition:    	gtempTRISPORT - the TRIS port address, gtempTRIS - the
*					TRIS mask, gtempTRISClear - the TRIS clear mask
*
* Input:           	y - rawADC Array location
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Run 1 mutual ADC measuement using one of the PORTD inputs
*
* Note:            	Assume the ADC channel was setup prior to this call
*
*****************************************************************************/
void mutualCTMUScan(unsigned char y)
{
	if ((userRam.customFlag&USEMUTDIFF) != USEMUTDIFF)
	{
		*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;
		*(varRam.gANSEL) = varRam.gANSELbit;
		ADCON0bits.CHS = varRam.gADCONCHS;
		Nop();
		Nop();
		Nop();
		CTMUCONHbits.IDISSEN = 1;	// discharge CTMU
		Nop();
		Nop();
		Nop();
		CTMUCONHbits.IDISSEN = 0;	// stop CTMU discharge

		//
		// Drive TX's
		//
		*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];
		*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];

		TMR2 = 0;					// clear TMR2
		Nop();
		PIR1bits.TMR2IF = 0;		// clear TMR2 interrupt
		while (!PIR1bits.TMR2IF){}	// wait appropritae rise time

		ADCON0bits.DONE = 1;
		while(ADCON0bits.DONE){}

		*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;

		rawRam.rawADC[y] = ADRES;

		*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;
		Nop();
		Nop();
		Nop();
		CTMUCONHbits.IDISSEN = 1;	// discharge CTMU
		Nop();
		Nop();
		Nop();
		CTMUCONHbits.IDISSEN = 0;	// stop CTMU discharge
		//
		// Drive TX's
		//
		*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
		*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];

		TMR2 = 0;					// clear TMR2
		Nop();
		PIR1bits.TMR2IF = 0;		// clear TMR2 interrupt
		while (!PIR1bits.TMR2IF){}	// wait appropritae rise time

		ADCON0bits.DONE = 1;
		while(ADCON0bits.DONE){}

		*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;

		if(ADRES > rawRam.rawADC[y])			//extra processing if 2nd measurement > 1st measurement
		{
			rawRam.rawADC[y] = (ADRES - rawRam.rawADC[y]);
		}
		else
		{
			rawRam.rawADC[y] -= ADRES;			//differential measurement - subtract 1st minus 2nd
		}
	}
	else
	{
		*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;
		*(varRam.gANSEL) = varRam.gANSELbit;
		ADCON0bits.CHS = varRam.gADCONCHS;
		Nop();
		Nop();
		Nop();
		CTMUCONHbits.IDISSEN = 1;	// discharge CTMU
		Nop();
		Nop();
		Nop();
		CTMUCONHbits.IDISSEN = 0;	// stop CTMU discharge
		Nop();
		Nop();
		Nop();
		//
		// Drive TX's
		//
		*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];
		*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];

		TMR2 = 0;					// clear TMR2
		Nop();
		PIR1bits.TMR2IF = 0;		// clear TMR2 interrupt
		while (!PIR1bits.TMR2IF){}	// wait appropritae rise time

		ADCON0bits.DONE = 1;
		while(ADCON0bits.DONE){}

		*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;

		*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
		*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];

		rawRam.rawADC[y] = ADRES;
	}
}

/******************************************************************************
* Function:        	void mutualCVDScan(unsigned char y)
*
* PreCondition:    	gtempTRISPORT - the TRIS port address, gtempTRIS - the
*					TRIS mask, gtempTRISClear - the TRIS clear mask
*
* Input:           	y - rawADC Array location
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Run 1 mutual ADC measuement using one of the PORTD inputs
*
* Note:            	Assume the ADC channel was setup prior to this call
*
*****************************************************************************/
void mutualCVDScan(unsigned char y)
{
	unsigned int temp;

	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;		//switch ADC to analog channel we want to sense
	*(varRam.gANSEL) = varRam.gANSELbit;				//setup sense channel as analog input
	ANSELAbits.ANSA0 = 1;

	ADC_CHARGE_TRIS = 0;
	ADC_CHARGE_LAT = 1;
	ADCON0bits.CHS = ADC_CHARGE_CHANNEL;

	Nop();

	ADCON0bits.CHS = varRam.gADCONCHS;

	*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];

	Nop();

	ADCON0bits.DONE = 1;
	while(ADCON0bits.DONE){}

	*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];

	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;			//restore TRIS registers to previous state

	ADC_CHARGE_LAT = 0;
	temp = ADRES;							//store first ADC result

	*(varRam.gtempTRISPORT-LAT_OFFSET) |= varRam.gtempTRIS;	//drive sense channel high
	*(varRam.gtempTRISPORT) &= ~varRam.gtempTRIS;		//turn sense channel to digital output

	Nop();

	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;		//turn sense channel back to input
	ADCON0bits.CHS = varRam.gADCONCHS;					//reconnect sense channel to ADC

	*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];

	Nop();

	ADCON0bits.DONE = 1;
	while(ADCON0bits.DONE){}

	*(varRam.gtempTRISPORT-LAT_OFFSET) &= ~varRam.gtempTRIS;	//clear LAT bit associated with sense channel
	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;			//restore TRIS registers to previous state

	temp = (1023 - ADRES) - temp;

	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;		//switch ADC to analog channel we want to sense
	*(varRam.gANSEL) = varRam.gANSELbit;				//setup sense channel as analog input
	ANSELAbits.ANSA0 = 1;

	ADC_CHARGE_TRIS = 0;
	ADC_CHARGE_LAT = 1;
	ADCON0bits.CHS = ADC_CHARGE_CHANNEL;

	Nop();

	ADCON0bits.CHS = varRam.gADCONCHS;

	*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];

	Nop();

	ADCON0bits.DONE = 1;
	while(ADCON0bits.DONE){}

	*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];

	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;			//restore TRIS registers to previous state

	ADC_CHARGE_LAT = 0;
	rawRam.rawADC[y] = ADRES;							//store first ADC result

	*(varRam.gtempTRISPORT-LAT_OFFSET) |= varRam.gtempTRIS;	//drive sense channel high
	*(varRam.gtempTRISPORT) &= ~varRam.gtempTRIS;		//turn sense channel to digital output

	Nop();

	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;		//turn sense channel back to input
	ADCON0bits.CHS = varRam.gADCONCHS;					//reconnect sense channel to ADC

	*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];

	Nop();

	ADCON0bits.DONE = 1;
	while(ADCON0bits.DONE){}

	*(varRam.gtempTRISPORT-LAT_OFFSET) &= ~varRam.gtempTRIS;	//clear LAT bit associated with sense channel
	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;			//restore TRIS registers to previous state

	rawRam.rawADC[y] = (1023 - ADRES) - rawRam.rawADC[y];

	rawRam.rawADC[y] -= temp;
}

//
// I2C functions
//
/******************************************************************************
* Function:        	void initI2C(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Initializes the UART
*
* Note:            	None
*
*****************************************************************************/
#ifdef I2C
void initI2C(void)
{
	SSP1CON1 = 0b00010000;			// disable synchronous serial port
	//
	// setup the I2C
	//
	SSP1STAT &= 0x3F;			// power on state
	SSP1CON1 = 0x00;			// power on state
	SSP1CON2 = 0x01;			// power on state. CLK Stretching enabled
	SSP1CON1 |= 0b00010110;//0x1e;//0x0e;//0x06;//0x0e;//0x06;			// Slave 7 bit
	SSP1STAT |= 0x80;			// slew rate off
	SSP1ADD = 0x4A;//0x4A; 			// M << 1, use M as address
	SSP1MSK = 0b11111110;
	SSP1CON3 = 0b00110000;

	ANSELCbits.ANSC3 = 0;
	ANSELCbits.ANSC4 = 0;
	I2C_SCL = 1;
	I2C_SDA = 1;
	SSP1CON1bits.SSPEN = 1;		// enable synchronous serial port
	//
	// setup the interrupt pin
	//
	I2C_INT = 0;
	I2C_TRIS = 0;
	//
	// clear the output buffer count
	//
	commRam.txCount = 0;
	commRam.rcvCount = 0;
	//
	// get the I2C interrupt setup
	//
	IPR1bits.SSP1IP = 0;
	PIR1bits.SSP1IF = 0;
	PIE1bits.SSP1IE = 1;
	//
	// interrupts are enabled in init.c
	//
}
#endif //ifdef I2C
/******************************************************************************
* Function:        	void readI2C(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Reads bytes from I2C
*
* Note:            	For data receieved from the master we expect a single
*					byte at a time
*					Format:
*					<Address><data>
*
*****************************************************************************/
#ifdef I2C
void readI2C(void)
{
	unsigned char x;
	unsigned char tmrState;
	//
	// setup a time out of 5ms
	//

	PIE1bits.SSP1IE = 0;
	Nop();
	PIR1bits.SSP1IF=0;
	T1CONbits.TMR1ON = 0;
	T1CON =0b00110000;
	TMR1H = 0xDE;	// 5ms timeout
	TMR1L = 0x8F;
	PIR1bits.TMR1IF=0;
	tmrState = T1CONbits.TMR1ON;
	T1CONbits.TMR1ON = 1; 			// turn TMR1 on

	while (!SSP1STATbits.BF)
	{
		if(PIR1bits.TMR1IF)
		{
			SSP1CON1bits.SSPEN = 0;
			SSP1CON1 = 0x1e;
			Nop();
			SSP1CON1 = 0b00010110;
			SSP1CON1bits.SSPEN = 1;
			goto endreadI2C;
		}
	}
	while (!PIR1bits.SSP1IF)
	{
		if(PIR1bits.TMR1IF)
		{
			goto endreadI2C;
		}
	}
	x = SSP1BUF; // address
	SSP1CON1bits.CKP = 1;		// Release SCL
	//
	// if we are here we received the address byte
	// so get the data byte
	//
	PIR1bits.SSP1IF = 0;
	while (!SSP1STATbits.BF)	// wait for the buffer to be full
	{
		if (PIR1bits.TMR1IF)
		{
			goto endreadI2C;
		}
	}
	if (commRam.rcvCount > (unsigned char)RCVBUFFERSIZE)
	{
		commRam.rcvCount = 0;
	}
	commRam.rcvBuffer[commRam.rcvCount] = SSP1BUF;
	commRam.rcvCount ++;

endreadI2C:
	TMR1H=0;
	TMR1L=0;
	PIR1bits.TMR1IF = 0;
	T1CONbits.TMR1ON = 0; 			// turn TMR1 off
	T1CONbits.TMR1ON = tmrState;
	SSP1CON1bits.CKP = 1;		// Release SCL
	PIE1bits.SSP1IE = 1;
}
#endif //ifdef I2C
/******************************************************************************
* Function:        	void writeI2C(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Writes n bytes, < 64, to master
*
* Note:            	Send up to 64 data bytes to the master
*					Format:
*					<address><N - number of data bytes to send><data0>..<dataN>
*					The host software reads the first byte to determine how many
*					data bytes to clock out
*
*****************************************************************************/
#ifdef I2C
void writeI2C(void)
{
	unsigned char x;

	if (commRam.txCount == 0)
	{
		return;
	}
	//
	// first disable the interrupt
	//
	PIE1bits.SSP1IE = 0;
	x = SSP1BUF; // clear the buffer
	//
	// setup a time out of 5ms
	//

	globalCounter = 5;

	PIR1bits.SSP1IF = 0;
	//
	// set the interrupt
	//
	I2C_INT = 1;
	//
	// The loop below while (!SSP1STATbits.BF) determines
	// how long we wait for the master to respond to
	// the interrupt, currently set at 5ms
	//
	// get the address byte
	//
	while (!SSP1STATbits.BF)	// wait for the buffer to be full
	{
		if(globalCounter == 0)
		{
			goto endwriteI2C;
		}
	}
	//
	// clear the interrupt pin after the address byte
	//

	while(SSP1CON1bits.CKP) // R/W held low after 9th clovk
	{
		if(globalCounter == 0)
		{
//initI2C();
	//	SSP1CON1bits.SSPEN=0;
	//	Nop();
	//	SSP1CON1bits.SSPEN=1;
			goto endwriteI2C;
		}
	}
	x = SSP1BUF; // address
	I2C_INT = 0;			// clear the interrupt

	//
	// if we are here we received the address byte
	// so send the data bytes
	//
	// The first byte we send out is the number of
	// bytes were going to return
	//
	if (SSP1STAT&0x04)	// make sure the master is ready for transmission
	{
		//
		// The first byte we send out is the number of
		// bytes were going to return
		//
		while (SSP1STATbits.BF)	// wait for the buffer to be empty
		{
			if(globalCounter == 0)
			{
				goto endwriteI2C;
			}
		}
		PIR1bits.SSP1IF = 0;
		SSP1BUF = commRam.txCount;
		SSP1CON1bits.CKP = 1;			// release clock line
		while (SSP1STATbits.BF)	// wait for the buffer to be full
		{
			if(globalCounter == 0)
			{
				goto endwriteI2C;
			}
		}
		while (!PIR1bits.SSP1IF)		// wait until ninth clock pulse received
		{
			if(globalCounter == 0)
			{
				goto endwriteI2C;
			}
		}

		//
		// trasmit the bytes
		//

		globalCounter = 32;
		SSP1STATbits.P = 0;			// clear the stop bit flag, use it to determine we are done
		for (x=0;x<commRam.txCount;x++)
		{
			while(SSP1CON1bits.CKP) // R/W held low after 9th clovk
			{
				if(globalCounter == 0)
				{
					goto endwriteI2C;
				}
			}

			PIR1bits.SSP1IF = 0;
			SSP1BUF = commRam.txBuffer[x];
			SSP1CON1bits.CKP = 1;			// release clock line
			while (SSP1STATbits.BF)	// wait for the buffer to be full
			{
				if(globalCounter == 0)
				{
					goto endwriteI2C;
				}
			}
			while (!PIR1bits.SSP1IF)		// wait until ninth clock pulse received
			{
				if(globalCounter == 0)
				{
					goto endwriteI2C;
				}
			}

		}
		while (!SSP1STATbits.P)			// wait for the stop condition
		{
			if(globalCounter == 0)
			{
				goto endwriteI2C;
			}
		}
	}
	commRam.txCount = 0;
	SSP1CON1bits.CKP = 1;		// release clock line
	I2C_INT = 0;			// clear the interrupt
	PIR1bits.SSP1IF = 0;
	PIE1bits.SSP1IE = 1;		// re-enable the interrupt


	return;

endwriteI2C:
	//
	// reste the module
	//
	SSP1CON1bits.SSPEN=0;
	Nop();
	SSP1CON1bits.SSPEN=1;
	commRam.txCount = 0;
	SSP1CON1bits.CKP = 1;		// release clock line
	I2C_INT = 0;			// clear the interrupt
	PIR1bits.SSP1IF = 0;
	PIE1bits.SSP1IE = 1;		// re-enable the interrupt



}
#endif //ifdef I2C

//
// UART
//
/******************************************************************************
* Function:        	void initUART(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Initializes the UART
*
* Note:            	None
*
*****************************************************************************/
#ifdef UART
void initUART(void)
{
	//
	// Init UART 2
	//
	ANSELDbits.ANSD7 = 0;
	TRISDbits.TRISD7 = 1;	// should be set at boot
	TXSTA2 = 0b00100010;
	RCSTA2 = 0b10010000;
	BAUDCON2 = 0b00001000;
	SPBRG2 = BAUDLO;//19;//34;//68;	// 34 = 115.2K, 68 57.6K, 6 for 576000, 9 for 400000, 19 for 200000
	SPBRGH2 = BAUDHI;//0;
	commRam.rcvCount = 0;
	PIR3bits.RC2IF = 0;
	PIE3bits.RC2IE = 1;	// enable receive interrupt
	IPR3bits.RC2IP = 0; // low priority
}
#endif // ifdef UART

/******************************************************************************
* Function:        	void writeByteToEEPROM( unsigned int address,unsigned char data)
*
* PreCondition:    	None
*
* Input:           	EE address, EE data to write
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	writes a byte to EEPROM
*
* Note:
*
*****************************************************************************/
void writeByteToEEPROM( unsigned int address,unsigned char data)
{
	while(EECON1bits.WR);

	EEADRH = (address >> 8) & 0x03;
	EEADR = (address & 0x0ff);
	EEDATA = data;
  	EECON1bits.EEPGD = 0;
	EECON1bits.CFGS = 0;
	EECON1bits.WREN = 1;
	INTCONbits.GIE = 0;
	EECON2 = 0x55;
	EECON2 = 0xAA;
	EECON1bits.WR = 1;
	INTCONbits.GIE = 1;
	EECON1bits.WREN = 0;
}

/******************************************************************************
* Function:        	unsigned char readByteFromEEPROM( unsigned int address)
*
* PreCondition:    	None
*
* Input:           	EE address
*
* Output:          	data byte from EE
*
* Side Effects:    	None
*
* Overview:        	reads a byte to EEPROM
*
* Note:
*
*****************************************************************************/
unsigned char readByteFromEEPROM( unsigned int address)
{
	EEADRH = (address >> 8) & 0x03;
	EEADR = (address & 0x0ff);
  	EECON1bits.CFGS = 0;
	EECON1bits.EEPGD = 0;
	EECON1bits.RD = 1;
	return ( EEDATA );              // return with read byte
}

/******************************************************************************
* Function:        	void writeRAMToEEPROM(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Writes all of RAM to EEPROM including the 0x4D tag
*
* Note:
*
*****************************************************************************/
void writeRAMToEEPROM(void)
{
	unsigned char x;

	for (x=0;x<sizeof(userRam);x++)
	{
		writeByteToEEPROM((unsigned int)(x+1), ((unsigned char*)&userRam)[x]);
	}
	//
	// indicate we have data by writting an M to word location 0
	//
	writeByteToEEPROM(0, 0x4D);

}

/******************************************************************************
* Function:        	void readEEPROMToRAM(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	If the EEPROM has data will write the data to userRam
*
* Note:
*
*****************************************************************************/
void readEEPROMToRAM(void)
{
	unsigned char x;

	//
	// first check if word location 0 has an M
	//
	if (0x4D == readByteFromEEPROM(0))
	{
		//
		// never retore flag1
		//
		for (x=1;x<sizeof(userRam);x++)
		{
			((unsigned char*)&userRam)[x] = readByteFromEEPROM((unsigned int)(x+1));
		}
	}
}

/******************************************************************************
* Function:        	void eraseEEPROM(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Erases EEPROM
*
* Note:
*
*****************************************************************************/
void eraseEEPROM(void)
{
	unsigned char x;

	for (x=0;x<sizeof(userRam)+1;x++)
	{
		writeByteToEEPROM((unsigned int)x, 0xff);
	}
}

/******************************************************************************
* Function:        	void checkIO(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Checks all pCap configured IO for shorts
*
* Note:
*					If a level shifter is between the TX IO and the sensor the
*					short on the sensor cannot be checked
*
*					Output Format:
*					<0x55><0x08><RXLSB><RX><RX><RXMSB><TXLSB><TX><TX><TXMSB>
*					If the registers are not 0 there is a short
*
*****************************************************************************/
void checkIO(void)
{
	unsigned char x;
	unsigned long rxIO;
	unsigned long txIO;
	unsigned char tempVar;
	unsigned char errorBit;
	unsigned char gtempTRIS;

	rxIO = 0;
	txIO = 0;
	//
	// set all unmasked I/O as outputs and set to 0
	//
	groundAll();
	//
	// check RX lines
	//
	for (x=0;x<userRam.NumberofRXChannels;x++)
	{
		rxIO <<= 1;					// shift the error bit
		errorBit = 0;
		//
		// Get the TRIS settings for the channel
		//
		gtempTRIS = TRIS_LUT[userRam.rxPinMap[x]];


		tempVar = userRam.rxPinMap[x];

		if (tempVar<5)
		{
			LATA |= gtempTRIS;		// set the latch to 1
			Delay100TCYx(160);		// delay 1ms
			tempVar = PORTA;		// read the port
			tempVar &= gtempTRIS;	// mask off the bbit
			if (tempVar == 0)
			{
				errorBit = 1;
			}
			LATA &= ~(gtempTRIS); 	// clear the bit
		}
		else if (tempVar<8)
		{
			LATE |= gtempTRIS;		// set the latch to 1
			Delay100TCYx(160);		// delay 1ms
			tempVar = PORTE;		// read the port
			tempVar &= gtempTRIS;	// mask off the bbit
			if (tempVar == 0)
			{
				errorBit = 1;
			}
			LATE &= ~(gtempTRIS); 	// clear the bit
		}
		else if (tempVar<14)
		{
			LATB |= gtempTRIS;		// set the latch to 1
			Delay100TCYx(160);		// delay 1ms
			tempVar = PORTB;		// read the port
			tempVar &= gtempTRIS;	// mask off the bbit
			if (tempVar == 0)
			{
				errorBit = 1;
			}
			LATB &= ~(gtempTRIS); 	// clear the bit
		}
		else if (tempVar<20)
		{
			LATC |= gtempTRIS;		// set the latch to 1
			Delay100TCYx(160);		// delay 1ms
			tempVar = PORTC;		// read the port
			tempVar &= gtempTRIS;	// mask off the bbit
			if (tempVar == 0)
			{
				errorBit = 1;
			}
			LATC &= ~(gtempTRIS); 	// clear the bit
		}
		else if (tempVar<28)
		{
			LATD |= gtempTRIS;		// set the latch to 1
			Delay100TCYx(160);		// delay 1ms
			tempVar = PORTD;		// read the port
			tempVar &= gtempTRIS;	// mask off the bbit
			if (tempVar == 0)
			{
				errorBit = 1;
			}
			LATD &= ~(gtempTRIS); 	// clear the bit
		}
		rxIO |= errorBit;
	}
	groundAll();
	//
	// check TX lines
	//
	for (x=0;x<NUMBEROFTXCHANNELS;x++)
	{
		txIO <<= 1;					// shift the error bit
		errorBit = 0;
		tempVar = userRam.txPinMap[x];
		gtempTRIS = 1<<(tempVar%8);

		if (tempVar<8)
		{
			LATA |= gtempTRIS;		// set the latch to 1
			Delay100TCYx(160);		// delay 1ms
			tempVar = PORTA;		// read the port
			tempVar &= gtempTRIS;	// mask off the bbit
			if (tempVar == 0)
			{
				errorBit = 1;
			}
			LATA &= ~(gtempTRIS); 	// clear the bit
		}
		else if (tempVar<16)
		{
			LATB |= gtempTRIS;		// set the latch to 1
			Delay100TCYx(160);		// delay 1ms
			tempVar = PORTB;		// read the port
			tempVar &= gtempTRIS;	// mask off the bbit
			if (tempVar == 0)
			{
				errorBit = 1;
			}
			LATB &= ~(gtempTRIS); 	// clear the bit
		}
		else if (tempVar<24)
		{
			LATC |= gtempTRIS;		// set the latch to 1
			Delay100TCYx(160);		// delay 1ms
			tempVar = PORTC;		// read the port
			tempVar &= gtempTRIS;	// mask off the bbit
			if (tempVar == 0)
			{
				errorBit = 1;
			}
			LATC &= ~(gtempTRIS); 	// clear the bit
		}
		else if (tempVar<32)
		{
			LATD |= gtempTRIS;		// set the latch to 1
			Delay100TCYx(160);		// delay 1ms
			tempVar = PORTD;		// read the port
			tempVar &= gtempTRIS;	// mask off the bbit
			if (tempVar == 0)
			{
				errorBit = 1;
			}
			LATD &= ~(gtempTRIS); 	// clear the bit
		}
		else if (tempVar<36)
		{
			LATE |= gtempTRIS;		// set the latch to 1
			Delay100TCYx(160);		// delay 1ms
			tempVar = PORTE;		// read the port
			tempVar &= gtempTRIS;	// mask off the bbit
			if (tempVar == 0)
			{
				errorBit = 1;
			}
			LATE &= ~(gtempTRIS); 	// clear the bit
		}

		txIO |= errorBit;
	}

	groundAll();
	#ifdef UART
	while(!UART_TRMT)
	{}
	UART_TXREG = 0x55;
	while(!UART_TRMT)
	{}
	UART_TXREG = 0x08;
	for (x=0;x<4;x++)
	{
		while(!UART_TRMT)
		{}
		UART_TXREG = (unsigned char)rxIO;
		rxIO >>= 8;
	}
	for (x=0;x<4;x++)
	{
		while(!UART_TRMT)
		{}
		UART_TXREG = (unsigned char)txIO;
		txIO >>= 8;
	}
	#endif // ifdef UART

	#ifdef I2C
	commRam.txBuffer[0] = 0x55;
	commRam.txBuffer[1] = 0x08;
	commRam.txCount = 2;
	for (x=0;x<4;x++)
	{
		commRam.txBuffer[commRam.txCount] = (unsigned char) rxIO;
		commRam.txCount ++;
		rxIO >>= 8;
	}
	for (x=0;x<4;x++)
	{
		commRam.txBuffer[commRam.txCount] = (unsigned char) txIO;
		commRam.txCount ++;
		txIO >>= 8;
	}

	#ifdef TOUCHPADDEMONSTRATOR
		if(!SELECT_PIN)
		{
			writeI2C();
		}
	#else
		writeI2C();
	#endif

	#endif // ifdef I2C
}

#ifdef DISPLAY
//
// SPI functions
//
/******************************************************************************
* Function:        	void initSPI(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Initializes the UART
*
* Note:            	None
*
*****************************************************************************/
void initLCDSPI(void)
{
	LCD_RST = 0;					// 0 for LCD reset, 1 for active
	LCD_RST_TRIS = 0;				// output
	LCD_A0 = 0;						// LCD A0 line
	LCD_A0_TRIS = 0;				// output
	LCD_RST = 1;					// 0 for LCD reset, 1 for active

	//
	// set the peripreal pin select
	//
	SLRCONbits.SLRC = 0;
	SPI_SCK_TRIS = 0;				// RC3 !!shared with I2C SCL
	SPI_SDO_TRIS = 0;				// RC5
	//
	// Configure the SPI module
	//
	SSP1CON1bits.SSPEN = 0;			// disable the module
	SSP1STAT = 0b00000000;
	SSP1CON1 = 0b00010000;			// FOSC/4
	//SSP1CON2 = 0b00000000;		// Not used for SPI leave set from I2C
	//SSP1CON3 = 0b00000000;		// Not used for SPI leave set from I2C
	//SSP1ADD = 0b00000000;			// Not used for SPI leave set from I2C
	SSP1CON1bits.SSPEN = 1;			// enable the module
	//
	// Use SDI as CS
	//
	SPI_CS = 1;
	SPI_CS_TRIS = 0;
}


/*************************************************************************//**
* Function:        	unsigned char sendSPIchar(unsigned char c)
*
* \pre		    	None
*
* \result
*
* \post		    	None
*
* \brief        	Sends a character out over SPI
*
* \note           	None
*
*****************************************************************************/
void sendSPIchar(unsigned char c)
{
/*	while (!SSP1STATbits.BF);
	SSP1STATbits.BF = 0;
	SSP1BUF = c;*/
	PIR1bits.SSP1IF = 0;
	SSP1BUF = c;
	while (!PIR1bits.SSP1IF) {}
}
/*************************************************************************//**
* Function:        	unsigned char waitOnSPITX(void)
*
* \pre		    	None
*
* \result
*
* \post		    	None
*
* \brief        	Wait for all SPI communications to be complete
*
* \note           	None
*
*****************************************************************************/
void waitOnSPITX(void)
{
	// empty
}
/*************************************************************************//**
* Function:        	unsigned char sendSPIstring(unsigned char *c, int size)
*
* \pre		    	None
*
* \result
*
* \post		    	None
*
* \brief        	Sends a character out over SPI
*
* \note           	None
*
*****************************************************************************/
void sendSPIstring(const unsigned char *s, int size)
{
	unsigned char c;
	int i = 0;
	for (i = 0; i < size; i++)
	{
		c = s[i];
		sendSPIchar(c);
	}
}
#endif // DISPLAY


void peripheralShutdown(void)
{
	CTMUCONHbits.CTMUEN = 0;		//disable CTMU
	T0CONbits.TMR0ON = 0;			//disable TMR0
	T1CONbits.TMR1ON = 0;			//disable TMR1
	T2CONbits.TMR2ON = 0;			//disable TMR2
	T3CONbits.TMR3ON = 0;			//disable TMR3
	T4CONbits.TMR4ON = 0;			//disable TMR4
	SSP1CON1bits.SSPEN = 0;			//disable SPI

	LATA = 0x0000;					//
	LATB = 0x0000;					//LAT output drivers cleared
	LATC = 0x0000;					//
	LATD = 0x0004;
	LATE = 0x0000;

	TRISA = 0xffff;					//
	TRISB = 0xffff;					//configure I/O as input
	TRISC = 0xffff;					//
	TRISD = 0xfffb;
	TRISE = 0xffff;
}

/******************************************************************************
* Function:        	void peripheralWakeup(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	none
*
* Side Effects:    	None
*
* Overview:        	re-enables all interrupts and re-initializes peripherals for
*					normal operation
*
* Note:            	None
*
*****************************************************************************/
void peripheralWakeup(void)
{

#ifdef I2C
	initI2C();
#endif
	decodeInit();
	initPCHardware();
	sleepCount = 0;
	initNoise();
	stutterMaskSetup();
#ifdef DISPLAY
	initLCDSPI();
	lcdInit(INIT_FAST);
#endif
}

/******************************************************************************
* Function:        	void goToSleep(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	none
*
* Side Effects:    	None
*
* Overview:        	shuts down peripherals and puts the controller to sleep. The
*					WDT is used to periodically wake up the PIC to scan for a touch.
*					If a touch is present, proceed with wakeup. If not, go back to
*					sleep.
*
* Note:            	None
*
*****************************************************************************/
void goToSleep(void)
{
#ifdef ENABLE_SLEEP
	unsigned char sleepCycles = 0;
#ifdef DISPLAY
	lcdSetOverlay(OVERLAY_SLEEP,250);
	//lcdBufferBlit();
	lcdFrame();
#endif
	peripheralShutdown();
	varRam.flag &= NOTTOUCH;
	while((varRam.flag&TOUCH)!=TOUCH)
	{
		ADCON0bits.ADON = 0;
		INTCONbits.GIE = 0;				//disable all interrupts
		WDTCONbits.SWDTEN = 1;	//turn on WDT
		Sleep();
		//RCONbits.WDTO = 0;		//clear WDT flag
		WDTCONbits.SWDTEN = 0;	//turn off WDT

		INTCONbits.GIE = 1;			//enable all interrupts
		ADCON0bits.ADON = 1;
		groundAll();
		initGestures();
		checkTouch();
#ifdef DISPLAY
		sleepCycles++;
		if (sleepCycles > 250)
		{
			sleepCycles = 0;
			initLCDSPI();
			lcdSetGUIState(UI_LOGO);
			lcdFrame();
			peripheralShutdown();
		}
#endif
	}
	peripheralWakeup();
	baselineCounter = 0;
	sleepCount = 0;
	sleepFlag = 0;
	delay1ms(5);
#ifdef DISPLAY
	lcdSetOverlay(OVERLAY_NONE,0);
	lcdFrame();
#endif
#endif // ENABLE_SLEEP
}
