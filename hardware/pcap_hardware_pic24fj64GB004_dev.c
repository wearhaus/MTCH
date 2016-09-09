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

#include "main/main.h"
#ifdef DISPLAY
	#include "display\lcddisplay.h"
#endif
#include "hardware/pcap_hardware_pic24fj64GB004_dev.h"

//
// Globals
//
extern NOISERAM mutNoise;
extern NOISERAM selfNoise;
extern USERRAM userRam;
extern COMMRAM commRam;
extern RAWRAM rawRam;
extern VARRAM varRam;
extern BASERAM baseRam;
#ifdef LONGADCSCAN
extern unsigned short longADC[512];
#endif

extern DEBUGRAM debugRam;
extern volatile unsigned short globalCounter;
extern unsigned char sleepFlag;
extern unsigned short sleepCount;
extern unsigned short baselineCounter;

extern unsigned char selfScanFineTune[MAXTX];

extern GESTURE gesture[MAX_TOUCHES];
#ifdef DISPLAY
	extern LCDBUFFER lcdBuffer;
#endif //DISPLAY
unsigned short tempIE0;
unsigned short tempIE1;
unsigned short tempIE2;
unsigned short tempIE3;
unsigned short tempIE4;
unsigned short tempIE5;

//_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & FWDTEN_OFF & ICS_PGx1)
_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & FWDTEN_OFF & FWPSA_PR32 & WDTPS_PS64 & ICS_PGx2)
// use following for internal oscillator
_CONFIG2( IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_ON & FNOSC_FRCPLL & POSCMOD_NONE & PLLDIV_DIV2 & I2C1SEL_PRI & IOL1WAY_OFF & PLL96MHZ_ON)
_CONFIG3( WPCFG_WPCFGDIS & WPDIS_WPDIS & SOSCSEL_IO)		//Disable erase/write protect of all memory regions.


//
// define the look up tables for TRIS, and LAT, and
// analog select, note these are in the same order as
// RX and TX PINMAP's
//
const __attribute__((space(auto_psv))) unsigned int TRIS_LUT[] =
{
	0b0000000000000001,	//	0 	RA0, AN0
	0b0000000000000010,	//	1 	RA1, AN1
	0b0000000000000001,	//	2 	RB0, AN2
	0b0000000000000010,	//	3 	RB1, AN3
	0b0000000000000100,	//	4 	RB2, AN4
	0b0000000000001000,	//	5 	RB3, AN5
	0b0000000000000001,	//	6 	RC0, AN6
	0b0000000000000010,	//	7 	RC1, AN7
	0b0000000000000100,	//	8 	RC2, AN8
	0b1000000000000000,	//	9 	RB15, AN9
	0b0100000000000000,	//	10 	RB14, AN10
	0b0010000000000000,	//	11 	RB13, AN11
	0b0000000000001000	//	12 	RC3, AN12
};

//static unsigned int TRISPORT_LUT[] =
const __attribute__((space(auto_psv))) unsigned int TRISPORT_LUT[] =
{
	0x02c0, 			//	0	TRISA address
	0x02c0, 			//	1	TRISA address
	0x02c8, 			//	2	TRISB address
	0x02c8, 			//	3	TRISB address
	0x02c8, 			//	4	TRISB address
	0x02c8, 			//	5	TRISB address
	0x02d0, 			//	6	TRISC address
	0x02d0, 			//	7	TRISC address
	0x02d0, 			//	8	TRISC address
	0x02c8, 			//	9	TRISB address
	0x02c8, 			//	10	TRISB address
	0x02c8, 			//	11	TRISB address
	0x02d0 				//	12	TRISC address
};

//static unsigned int LAT_LUT[] =
const __attribute__((space(auto_psv))) unsigned int LAT_LUT[] =
{
	0b0000000000000001,	//	0 	RA0
	0b0000000000000010,	//	1 	RA1
	0b0000000000000100,	//	2 	RA2
	0b0000000000001000,	//	3 	RA3
	0b0000000000010000,	//	4 	RA4
	0b0000000010000000,	//	5 	RA7
	0b0000000100000000,	//	6 	RA8
	0b0000001000000000,	//	7 	RA9
	0b0000010000000000,	//	8 	RA10
	0b0000000000000001,	//	9 	RB0
	0b0000000000000010,	//	10 	RB1
	0b0000000000000100,	//	11 	RB2
	0b0000000000001000,	//	12 	RB3
	0b0000000000010000,	//	13 	RB4
	0b0000000000100000,	//	14 	RB5
	0b0000000010000000,	//	15 	RB7
	0b0000000100000000,	//	16 	RB8
	0b0000001000000000,	//	17 	RB9
	0b0000010000000000,	//	18 	RB10
	0b0000100000000000,	//	19 	RB11
	0b0010000000000000,	//	20 	RB13
	0b0100000000000000,	//	21 	RB14
	0b1000000000000000,	//	22 	RB15
	0b0000000000000001,	//	23 	RC0
	0b0000000000000010,	//	24 	RC1
	0b0000000000000100,	//	25 	RC2
	0b0000000000001000,	//	26 	RC3
	0b0000000000010000,	//	27 	RC4
	0b0000000000100000,	//	28 	RC5
	0b0000000001000000,	//	29 	RC6
	0b0000000010000000,	//	30 	RC7
	0b0000000100000000,	//	31 	RC8
	0b0000001000000000	//	32 	RC9
};

//static unsigned int LATPORT_LUT[] =
const __attribute__((space(auto_psv))) unsigned int LATPORT_LUT[] =
{
	0x02c4,				// 0 	LATA address
	0x02c4,				// 1	LATA address
	0x02c4,				// 2	LATA address
	0x02c4,				// 3	LATA address
	0x02c4,				// 4	LATA address
	0x02c4,				// 5	LATA address
	0x02c4,				// 6	LATA address
	0x02c4,				// 7	LATA address
	0x02c4,				// 8	LATA address
	0x02cc,				// 9	LATB address
	0x02cc,				// 10	LATB address
	0x02cc,				// 11	LATB address
	0x02cc,				// 12	LATB address
	0x02cc,				// 13	LATB address
	0x02cc,				// 14	LATB address
	0x02cc,				// 15	LATB address
	0x02cc,				// 16	LATB address
	0x02cc,				// 17	LATB address
	0x02cc,				// 18	LATB address
	0x02cc,				// 19	LATB address
	0x02cc,				// 20	LATB address
	0x02cc,				// 21	LATB address
	0x02cc,				// 22	LATB address
	0x02d4,				// 23	LATC address
	0x02d4,				// 24	LATC address
	0x02d4,				// 25	LATC address
	0x02d4,				// 26	LATC address
	0x02d4,				// 27	LATC address
	0x02d4,				// 28	LATC address
	0x02d4,				// 29	LATC address
	0x02d4,				// 30	LATC address
	0x02d4,				// 31	LATC address
	0x02d4				// 32	LATC address
};
//
// Interrupts
//
/******************************************************************************
* Function:        	void __attribute__ ((interrupt, no_auto_psv)) _SI2C1Interrupt(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	PIC24F I2C interrupt function
*
* Note:
*
*****************************************************************************/
#ifdef I2C
void __attribute__ ((interrupt, no_auto_psv)) _SI2C1Interrupt(void)
{
	unsigned char x;

	if (IFS1bits.SI2C1IF && IEC1bits.SI2C1IE)
	{
		if ((I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 0))
		{
			//
			// R/W bit = 0 --> indicates data transfer is input to slave
			// D/A bit = 0 --> indicates last byte was address
			// reset any state variables needed by a message sequence
			// perform a dummy read of the address
			//
			//temp = SlaveReadI2C1();
			while(!I2C1STATbits.RBF){}
			I2C1STATbits.I2COV = 0;
			x = I2C1RCV;

			// release the clock to restart I2C
			I2C1CONbits.SCLREL = 1; // release the clock

		}
		else if ((I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 1))
		{
			//
			// R/W bit = 0 --> indicates data transfer is input to slave
			// D/A bit = 1 --> indicates last byte was data
			// writing data to our module, just store it in rcvBuffer
			//
			//dataRead = SlaveReadI2C1();
			while(!I2C1STATbits.RBF){}
			I2C1STATbits.I2COV = 0;
			commRam.rcvBuffer[commRam.rcvCount] = I2C1RCV;
			commRam.rcvCount ++;
			//commRam.rcvCount &= 0x07;
			if (commRam.rcvCount > RCVBUFFERSIZE)
			{
				commRam.rcvCount = 0;
			}

			// release the clock to restart I2C
			I2C1CONbits.SCLREL = 1; // release clock stretch bit

		}
		else if ((I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 0))
		{
			//
			// R/W bit = 1 --> indicates data transfer is output from slave
			// D/A bit = 0 --> indicates last byte was address
			// read of the slave device, read the address
			//
			while(!I2C1STATbits.RBF)
			{
				if (globalCounter == 0)
				{
					break;
				}
			}
			I2C1STATbits.I2COV = 0;
			x = I2C1RCV;
			I2C_INT = 0;				// clear the interrupt
			I2C1TRN = commRam.txCount;	//commRam.txBuffer[commRam.txWritten];
			I2C1CONbits.SCLREL = 1;    	// Release the clock
		}
		else if ((I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 1))
		{
			//
			// R/W bit = 1 --> indicates data transfer is output from slave
			// D/A bit = 1 --> indicates last byte was data
			// output the data until the MASTER terminates the
			// transfer with a NACK, continuing reads return 0
			//
			if (commRam.txCount != commRam.txWritten)
			{
			I2C1TRN = commRam.txBuffer[commRam.txWritten];
//			commRam.txWritten ++;
			I2C1CONbits.SCLREL = 1;    // Release the clock
			}
			//if (commRam.txCount == commRam.txWritten)
			if (commRam.txCount-1 == commRam.txWritten)
			{
				while(!I2C1STATbits.P)
				{
					if (globalCounter == 0)
					{
						break;
					}
				}
				//LATBCLR = I2C_INT;
			}
			commRam.txWritten ++;
		}

		IFS1bits.SI2C1IF = 0;		// clear the interrupt
	}
}
#endif // ifdef I2C

/******************************************************************************
* Function:        	void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	PIC24F UART receive interrupt
*
* Note:
*
*****************************************************************************/
#ifdef UART
void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
	if (IFS0bits.U1RXIF && IEC0bits.U1RXIE)
	{
		commRam.rcvBuffer[commRam.rcvCount] = U1RXREG;
		commRam.rcvCount ++;
		//commRam.rcvCount &= 0x07;
		if (commRam.rcvCount > RCVBUFFERSIZE)
		{
			commRam.rcvCount = 0;
		}
		IFS0bits.U1RXIF = 0;
	}
}
#endif // ifdef UART

/******************************************************************************
* Function:        	void __attribute__ ((interrupt, no_auto_psv)) _TMR1Interrupt(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	PIC24F TMR4 global 1ms interrupt
*
* Note:
*
*****************************************************************************/
void __attribute__ ((interrupt, no_auto_psv)) _T4Interrupt(void)
{
	if(IFS1bits.T4IF && IEC1bits.T4IE)          // Timer4 enable and interrupt occur
    {
    	unsigned char x;

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

#ifdef DISPLAY
		if (lcdBuffer.guiGlyphTimer > 0)
		{
			lcdBuffer.guiGlyphState = GS_CLEAR;
			lcdBuffer.guiGlyphTimer--;
		}
#endif

		if(selfNoise.timerCount >= 0)
		{
			selfNoise.timerCount++;
			if(selfNoise.timerCount > 1000)			//1s timeout
			{
				selfNoise.timerCount = -1;
				selfNoise.noisePresent = 0;
				selfNoise.triedFrequencies = 0;
			}
		}

		if(mutNoise.timerCount >= 0)
		{
			mutNoise.timerCount++;
			if(mutNoise.timerCount > 1000)			//1s timeout
			{
				mutNoise.timerCount = -1;
				mutNoise.noisePresent = 0;
				mutNoise.triedFrequencies = 0;
			}
		}

        PR4 = 250;
        TMR4 = 0x0000;
        IFS1bits.T4IF = 0;                         // reset interruptflag
    }
   	varRam.flag |= INTERRUPTOCCURRED;
}

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
	// Set RB13/CS13 as a digital pin for use in CVD
	// (currently set regardless of CTMU or CVD)
	//
	AD1PCFGLbits.PCFG11 = 0;

	//
	// OSC
	//
	CLKDIV = 0b000000000100000;
	//
	// clear interrupts
	//

	//
	// Disable Watch Dog Timer
	//
	RCONbits.SWDTEN = 0;
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
	unsigned int y;
	unsigned int value=0;

	DataEEInit();
    dataEEFlags.val = 0;
	//
	// first check if word location 0 has an M
	//
	value = DataEERead(0x0000);
	if (value == 0x004D)
	{

		//
		// Data is stored as words
		//
		x = 0;
		y = 1;
		while (x < sizeof(userRam))
		{
			value = DataEERead(y);
			if (0 != x) // flag1 is never restored
			{
				((unsigned char*)&userRam)[x] = (unsigned char)value;
			}
			x++;
			if (x < sizeof(userRam))
			{
				((unsigned char*)&userRam)[x] = (unsigned char)(value>>8);
			}
			x++;
			y++;
		}
	}
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
* Overview:        	Writes the RAM to EEPROM
*
* Note:
*
*****************************************************************************/
void writeRAMToEEPROM(void)
{
	unsigned char x;
	unsigned int y;
	unsigned int value;

	DataEEInit();
    dataEEFlags.val = 0;

	//
	// Data is stored as words
	//
	x = 0;
	y = 1;
	while (x < sizeof(userRam))
	{
		value = (unsigned int)((unsigned char*)&userRam)[x];
		x++;
		if (x == sizeof(userRam))
		{
			value |= 0xff00;
		}
		else
		{
			value |= (unsigned int)(((unsigned char*)&userRam)[x]<<8);
		}
		x++;
		DataEEWrite(value, y);
		y++;
	}
	//
	// indicate we have data by writting an M to word location 0
	//
	DataEEWrite(0x004D, 0);
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
* Overview:        	Writes the RAM to EEPROM
*
* Note:
*
*****************************************************************************/
void eraseEEPROM(void)
{
	unsigned char x;

	DataEEInit();
    dataEEFlags.val = 0;

	for (x=0;x<((sizeof(userRam)+1)>>1);x++)
	{
		DataEEWrite(0xffff, x);
	}
}

/******************************************************************************
* Function:        	void delay1ms(unsigned short delay)
*
* PreCondition:    	None
*
* Input:           	delay time in milliseconds
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	1ms delay
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
	AD1CON1bits.ADON = 0;
	AD1PCFGL = 0xffff;//0x0000;
	AD1CON1 = 0x0000;
	AD1CHS = 0x0000;			// select analog channel 0
	AD1CSSL = 0x0000;
	AD1CON1bits.FORM = 0x0;		// integer right justified
	AD1CON3bits.ADCS = 0;
	AD1CON2 = 0x0000;
	AD1CON1bits.ADON = 1;		// turn on the ADC

	//
	// setup the CTMU
	//
	CTMUCON = 0x0000;
	CTMUCONbits.EDGEN = 0;//1;		// Edge enable
	CTMUICONbits.IRNG = 2;//1;//3;		// 100 times base for now
	CTMUCONbits.CTMUEN = 1;		// Enable CTMU

	//set up TMR4 as 1ms global timer	-- 1:64 prescaler = 4us resolution
	T4CONbits.TON = 0;
	T4CON = 0b0000000000100000;
	IEC1bits.T4IE = 1;
	IPC6bits.T4IP = 1;
	IFS1bits.T4IF = 0;
	TMR4 = 0x0000;
	PR4 = 250;							//PR of 250 * 4us = 1ms timeout
	T4CONbits.TON = 1;

	//
	// run a 50ms delay to let everything warm up
	//
	delay1ms(50);
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
	AD1PCFGL = 0xffff; 	// all analog configured as digital I/O

	LATA &= GTEMPLATA;			// Set to 0
	LATB &= GTEMPLATB;
	LATC &= GTEMPLATC;

	TRISA &= TEMPTRISA;			// Set to output
	TRISB &= TEMPTRISB;
	TRISC &= TEMPTRISC;

	TRISBbits.TRISB5 = 1;
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

	for(i=0;i<userRam.stutterMult;i++) //iterate through all stuttering waves
	{
		//clear and setup latches
		*varRam.gtempTRISPORT |= varRam.gtempTRIS;


		//setup temporary latches based on 'wave' banks
		varRam.gtempLATA = varRam.gtempLATAgrp[i];
		varRam.gtempLATB = varRam.gtempLATBgrp[i];
		varRam.gtempLATC = varRam.gtempLATCgrp[i];

		//
		CTMUCONbits.IDISSEN = 1;	// discharge CTMU
		IFS0bits.AD1IF = 0;			// clear ADC interrupt
		TMR1 = 0;					// clear the timer
		IFS0bits.T1IF = 0;			// clear T1IF
		CTMUCONbits.IDISSEN = 0;	// stop CTMU discharge

		//
		// Drive TX's
		//
		LATA |= varRam.gtempLATA;
		LATB |= varRam.gtempLATB;
		LATC |= varRam.gtempLATC;


		AD1CON1bits.SAMP = 1;		// Start sampling
		while (!IFS0bits.T1IF){}	// wait appropritae rise time

		AD1CON1bits.SAMP = 0;		// Start converting
		while (!AD1CON1bits.DONE){}

		*varRam.gtempTRISPORT &= varRam.gtempTRISClear;

		LATA &= ~varRam.gtempLATA;
		LATB &= ~varRam.gtempLATB;
		LATC &= ~varRam.gtempLATC;

		rawRam.rawADC[y]+=ADC1BUF0;

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
	ADC_CHARGE_ANSEL = 0;
	ADC_CHARGE_TRIS = 0;

	rawRam.rawADC[y] = 0; //initialize this since it will be an accumulator

	for(i=0;i<userRam.stutterMult;i++) //iterate through all stuttering waves
	{
		ADC_CHARGE_LAT = 1;

		AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;
		AD1CON1bits.SAMP = 1;

		*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;

		//setup temporary latch masks based on current 'wave' (i)
		varRam.gtempLATA = varRam.gtempLATAgrp[i];
		varRam.gtempLATB = varRam.gtempLATBgrp[i];
		varRam.gtempLATC = varRam.gtempLATCgrp[i];

		//
		// Drive TX's
		//
		LATA |= varRam.gtempLATA;							//
		LATB |= varRam.gtempLATB;							//drive TX lines
		LATC |= varRam.gtempLATC;							//

		AD1CHSbits.CH0SA = varRam.gADCONCHS;
		Nop();

		AD1CON1bits.SAMP = 0;		// Start converting
		Nop();
		Nop();
		Nop();
		AD1CHSbits.CH0SA = NO_CHANNEL;

		*(varRam.gtempTRISPORT+LAT_OFFSET) |= varRam.gtempTRIS;
		*(varRam.gtempTRISPORT) &= ~varRam.gtempTRIS;

		ADC_CHARGE_LAT = 0;

		while (!AD1CON1bits.DONE){}

		AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;
		AD1CON1bits.SAMP = 1;
		rawRam.rawADC[y] += ADC1BUF0;

		*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;

		LATA &= ~varRam.gtempLATA;									//
		LATB &= ~varRam.gtempLATB;							//clear TX lines
		LATC &= ~varRam.gtempLATC;									//

		AD1CHSbits.CH0SA = varRam.gADCONCHS;
		Nop();

		AD1CON1bits.SAMP = 0;		// Start converting
		Nop();
		Nop();
		Nop();
		*(varRam.gtempTRISPORT+LAT_OFFSET) &= ~varRam.gtempTRIS;
		*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;
		while(!AD1CON1bits.DONE){}

		rawRam.rawADC[y] += (1023-ADC1BUF0);
	}
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
		*varRam.gtempTRISPORT |= varRam.gtempTRIS;	// Set CTMU channel as input
		CTMUCONbits.IDISSEN = 1;	// discharge CTMU
		IFS0bits.AD1IF = 0;			// clear ADC interrupt
		TMR1 = 0;					// clear the timer
		IFS0bits.T1IF = 0;			// clear T1IF
		CTMUCONbits.IDISSEN = 0;	// stop CTMU discharge

		AD1CON1bits.SAMP = 1;		// Start sampling

		//
		// Drive TX's
		//
				*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];
		*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];

		while(!IFS0bits.T1IF){}		// wait appropriate rise time
		AD1CON1bits.SAMP = 0;		// Start converting

		while (!AD1CON1bits.DONE){}
		*varRam.gtempTRISPORT &= varRam.gtempTRISClear;
		rawRam.rawADC[y] = ADC1BUF0;

		*varRam.gtempTRISPORT |= varRam.gtempTRIS;	// Set CTMU channel as input

		CTMUCONbits.IDISSEN = 1;	// discharge CTMU
		IFS0bits.AD1IF = 0;			// clear ADC interrupt
		TMR1 = 0;					// clear the timer
		IFS0bits.T1IF = 0;			// clear T1IF
		CTMUCONbits.IDISSEN = 0;	// stop CTMU discharge

		AD1CON1bits.SAMP = 1;		// Start sampling

		//
		// Drive TX's
		//
				*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
		*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];

		while(!IFS0bits.T1IF){}		// wait appropriate rise time
		AD1CON1bits.SAMP = 0;		// Start converting
		while (!AD1CON1bits.DONE){}		// wait for conversion to complete

		*varRam.gtempTRISPORT &= varRam.gtempTRISClear;
		if(ADC1BUF0 > rawRam.rawADC[y])			//extra processing if 2nd measurement > 1st measurement
		{
			rawRam.rawADC[y] = (ADC1BUF0 - rawRam.rawADC[y]);
		}
		else
		{
			rawRam.rawADC[y] -= ADC1BUF0;			//differential measurement - subtract 1st minus 2nd
		}
	}
	else
	{
		*varRam.gtempTRISPORT |= varRam.gtempTRIS;	// Set CTMU channel as input
		CTMUCONbits.IDISSEN = 1;	// discharge CTMU
		IFS0bits.AD1IF = 0;			// clear ADC interrupt
		TMR1 = 0;					// clear the timer
		IFS0bits.T1IF = 0;			// clear T1IF
		CTMUCONbits.IDISSEN = 0;	// stop CTMU discharge

		//
		// Drive TX's
		//
				*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];
		*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];

		AD1CON1bits.SAMP = 1;		// Start sampling
		while(!IFS0bits.T1IF){}		// wait appropriate rise time

		AD1CON1bits.SAMP = 0;		// Start converting
		while (!AD1CON1bits.DONE){}		// wait for conversion to complete
		//
		// Drive TX's low
		//
				*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
		*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];

		*varRam.gtempTRISPORT &= varRam.gtempTRISClear;
		rawRam.rawADC[y] = ADC1BUF0;
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
	ADC_CHARGE_ANSEL = 0;
	ADC_CHARGE_TRIS = 0;
	ADC_CHARGE_LAT = 1;

	AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;
	AD1CON1bits.SAMP = 1;
	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;

	//
	// Drive TX's
	//
	*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];

	AD1CHSbits.CH0SA = varRam.gADCONCHS;
	Nop();

	AD1CON1bits.SAMP = 0;		// Start converting
	Nop();
	Nop();
	Nop();
	AD1CHSbits.CH0SA = NO_CHANNEL;

	while (!AD1CON1bits.DONE){}

	*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];

	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;

	temp = ADC1BUF0;

	*(varRam.gtempTRISPORT+LAT_OFFSET) |= varRam.gtempTRIS;
	*(varRam.gtempTRISPORT) &= ~varRam.gtempTRIS;

	ADC_CHARGE_LAT = 0;

	AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;
	AD1CON1bits.SAMP = 1;

	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;

	*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];

	AD1CHSbits.CH0SA = varRam.gADCONCHS;
	Nop();

	AD1CON1bits.SAMP = 0;		// Start converting
	Nop();
	Nop();
	Nop();
	*(varRam.gtempTRISPORT+LAT_OFFSET) &= ~varRam.gtempTRIS;
	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;
	while(!AD1CON1bits.DONE){}

	temp = (1023-ADC1BUF0) - temp;

	ADC_CHARGE_TRIS = 0;
	ADC_CHARGE_LAT = 1;

	AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;
	AD1CON1bits.SAMP = 1;
	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;

	//
	// Drive TX's
	//
	*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];

	AD1CHSbits.CH0SA = varRam.gADCONCHS;
	Nop();

	AD1CON1bits.SAMP = 0;		// Start converting
	Nop();
	Nop();
	Nop();
	AD1CHSbits.CH0SA = NO_CHANNEL;

	while (!AD1CON1bits.DONE){}

	*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];

	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;

	rawRam.rawADC[y] = ADC1BUF0;

	*(varRam.gtempTRISPORT+LAT_OFFSET) |= varRam.gtempTRIS;
	*(varRam.gtempTRISPORT) &= ~varRam.gtempTRIS;

	ADC_CHARGE_LAT = 0;

	AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;
	AD1CON1bits.SAMP = 1;

	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;

	*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];

	AD1CHSbits.CH0SA = varRam.gADCONCHS;
	Nop();

	AD1CON1bits.SAMP = 0;		// Start converting
	Nop();
	Nop();
	Nop();
	*(varRam.gtempTRISPORT+LAT_OFFSET) &= ~varRam.gtempTRIS;
	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;
	while(!AD1CON1bits.DONE){}

	rawRam.rawADC[y] = (1023-ADC1BUF0) - rawRam.rawADC[y];

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

	commRam.rcvCount = 0;
	commRam.txCount = 0;
	commRam.txWritten = 0;

	I2C_INT = 0;				// setup the interrupt pin
	I2C_INT_TRIS = 0;
	OpenI2C1(I2C_ON | I2C_7BIT_ADD | I2C_STR_EN, 200); // 200 not used
	I2C1ADD = 0x4A>>1;				// Our device address
	I2C1MSK = 0;//xff;				// No mask
	IPC4bits.SI2C1IP = 4;			// interrupt priority 4
	IFS1bits.SI2C1IF = 0;			// clear interrupt flag
	IEC1bits.SI2C1IE = 1;			// enable slave interrupt
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
	if (commRam.txCount == 0)
	{
		return;
	}

	//
	// Set the interrupt
	//
	commRam.txWritten = 0;


	globalCounter = 20;
	I2C_INT = 1;

	while (commRam.txCount != commRam.txWritten)
	{
		if (globalCounter == 0)
		{
			//
			// Check for I2C errors, reset if need be
			// IWCOL, I2COV, RBF, or TBF
			//if (I2C1STATbits.TBF)
			if (I2C1STAT&0x000000c3)
			{
				//
				// reset I2C on error
				//
				I2C1CONbits.I2CEN = 0;
				initI2C();
			}
			//I2C1CONbits.SCLREL = 1;
			break;
		}
	} // wait for data consumed

	I2C_INT = 0;

	commRam.txCount = 0;
	commRam.txWritten = 0;

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
	// Init UART 1
	//
	IFS0bits.U1RXIF = 0;
    IFS0bits.U1TXIF = 0;
	RPINR18 = 26;
	RPOR1bits.RP3R = 3;  // TX on RG7
	U1MODE = 0b1000100000001000;
	U1STA =  0b0010010000000000;
	U1BRG = 35; //35 - 115200 @ Fcy 16MHZ 68 57600

    IEC0bits.U1TXIE = 0;
	//
	// setup the receive interrupts
	//
	IPC2bits.U1RXIP = 4;	// interrupt priority to 4
	IEC0bits.U1RXIE = 1;
	IFS0bits.U1RXIF = 0;
	commRam.rcvCount = 0;
}
#endif // ifdef UART

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

}

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
//#ifdef I2C
#ifdef DISPLAY
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
	RPINR20bits.SDI1R = 20;			// SPI Data input on RP20, RC8
	RPOR11bits.RP22R = 8;			// SPI CLK on RP22, RC6
	RPOR11bits.RP23R = 7;			// SPI Data output on RP22, RC7
	SPI_SCK_TRIS = 0;
	SPI_SDO_TRIS = 0;
	//
	// Configure the SPI module
	//
	SPI1STATbits.SPIEN = 0;			// disable the module
	SPI1STAT = 0b0000000000010000;
	SPI1CON1 = 0b0000000001111111;
	SPI1CON2 = 0;
	SPI1STATbits.SPIEN = 1;			// enable the module
	//
	// Use SDI as CS
	//
	SPI_CS = 1;
	SPI_CS_TRIS = 0;
}
#endif// DISPLAY

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
#ifdef DISPLAY
void inline sendSPIchar(unsigned char c)
{
	while (SPI1STATbits.SPITBF); // Wait until at least one buffer location is available
	SPI1BUF = c;
}
#endif// DISPLAY

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
#ifdef DISPLAY
void inline waitOnSPITX(void)
{
	while (SPI1STAT&0x0700);
}
#endif// DISPLAY

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
#ifdef DISPLAY
void inline sendSPIstring(const BYTE *c, int size)
{
	int i = 0;
	for (i = 0; i < size; i++)
	{
		sendSPIchar(c[i]);
	}
}
#endif// DISPLAY

//#endif //ifdef I2C

/******************************************************************************
* Function:        	void peripheralShutdown(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	turns off all peripherals in use, disables all interrupts, and
*					configures I/O for minimal current draw
*
* Note:            	None
*
*****************************************************************************/
void peripheralShutdown(void)
{
	CTMUCONbits.CTMUEN = 0;			//disable CTMU
	T1CONbits.TON = 0;				//disable TMR1
	T2CONbits.TON = 0;				//disable TMR2
	T3CONbits.TON = 0;				//disable TMR3
	T4CONbits.TON = 0;				//disable TMR4
	U1MODEbits.UARTEN = 0;			//disable UART
	I2C1CONbits.I2CEN = 0;			//disable I2C
	SPI1STATbits.SPIEN = 0;			//disable SPI

	tempIE0 = IEC0;					//save current interrupt enable values
	tempIE1 = IEC1;
	tempIE2 = IEC2;
	tempIE3 = IEC3;
	tempIE4 = IEC4;
	tempIE5 = IEC5;

	IEC0 = 0x0000;					//disable all interrupts
	IEC1 = 0x0000;
	IEC2 = 0x0000;
	IEC3 = 0x0000;
	IEC4 = 0x0000;
	IEC5 = 0x0000;

	LATA = 0x0000;					//
	LATB = 0x0000;					//LAT output drivers cleared
	LATC = 0x0200;					//

	TRISA = 0xffff;					//
	TRISB = 0xffff;					//configure I/O as input
	TRISC = 0xfdff;					//
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
	IEC0 = tempIE0;
	IEC1 = tempIE1;
	IEC2 = tempIE2;
	IEC3 = tempIE3;
	IEC4 = tempIE4;
	IEC5 = tempIE5;

#ifdef I2C
	initI2C();
#endif
#ifdef DISPLAY
	initLCDSPI();
#endif
	decodeInit();
	initPCHardware();
	T4CONbits.TON = 0;
	T4CON = 0b0000000000100000;
	IEC1bits.T4IE = 1;
	IPC6bits.T4IP = 1;
	IFS1bits.T4IF = 0;
	TMR4 = 0x0000;
	PR4 = 250;
	T4CONbits.TON = 1;
	sleepCount = 0;
	initNoise();
	stutterMaskSetup();
#ifdef DISPLAY
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
#ifdef DISPLAY
	unsigned char sleepCycles = 0;
	lcdSetOverlay(OVERLAY_SLEEP,250);
	//lcdBufferBlit();
	lcdFrame();
#endif
	peripheralShutdown();
	varRam.flag &= NOTTOUCH;
	while((varRam.flag&TOUCH)!=TOUCH)
	{
		AD1CON1bits.ADON = 0;
		RCONbits.WDTO = 0;		//clear WDT flag
		RCONbits.SWDTEN = 1;	//turn on WDT
		Sleep();
		RCONbits.WDTO = 0;		//clear WDT flag
		RCONbits.SWDTEN = 0;	//turn off WDT

		AD1CON1bits.ADON = 1;
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
}
