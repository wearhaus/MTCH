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
#include "hardware/pcap_hardware_pic24fj64GB106_dev.h"

//
// Globals
//
extern DEBUGRAM debugRam;
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

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx1 & BKBUG_OFF)
// use following for internal oscillator
//_CONFIG2( IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & FNOSC_FRCPLL & POSCMOD_NONE & PLLDIV_DIV2)// & POSCMOD_NONE)
// use following with 8MHZ crystal
//_CONFIG2( IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & FNOSC_PRIPLL & POSCMOD_XT & PLLDIV_DIV2 & PLL_96MHZ_ON)
// use following with 12MHZ resonnator
_CONFIG2( IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & FNOSC_PRIPLL & POSCMOD_XT & PLLDIV_DIV3 & PLL_96MHZ_ON)
_CONFIG3( WPCFG_WPCFGDIS & WPDIS_WPDIS)		//Disable erase/write protect of all memory regions.


//
// define the look up tables for TRIS, and LAT, and
// analog select, note these are in the same order as
// RX and TX PINMAP's
//
const __attribute__((space(auto_psv))) unsigned int TRIS_LUT[] =
{
	0b0000000000000001,	//	0 	RB0, AN0
	0b0000000000000010,	//	1 	RB1, AN1
	0b0000000000000100,	//	2 	RB2, AN2
	0b0000000000001000,	//	3 	RB3, AN3
	0b0000000000010000,	//	4 	RB4, AN4
	0b0000000000100000,	//	5 	RB5, AN5
	0b0000000001000000,	//	6 	RB6, AN6
	0b0000000010000000,	//	7 	RB7, AN7
	0b0000000100000000,	//	8 	RB8, AN8
	0b0000001000000000,	//	9 	RB9, AN9
	0b0000010000000000,	//	10 	RB10, AN10
	0b0000100000000000,	//	11 	RB11, AN11
	0b0001000000000000,	//	12 	RB12, AN12
	0b0010000000000000,	//	13 	RB13, AN13
	0b0100000000000000,	//	14 	RB14, AN14
	0b1000000000000000	//	15 	RB15, AN15
};

//static unsigned int TRISPORT_LUT[] =
const __attribute__((space(auto_psv))) unsigned int TRISPORT_LUT[] =
{
	0x02c8, 			//	0	TRISB address
	0x02c8, 			//	1	TRISB address
	0x02c8, 			//	2	TRISB address
	0x02c8, 			//	3	TRISB address
	0x02c8, 			//	4	TRISB address
	0x02c8, 			//	5	TRISB address
	0x02c8, 			//	6	TRISB address
	0x02c8, 			//	7	TRISB address
	0x02c8, 			//	8	TRISB address
	0x02c8, 			//	9	TRISB address
	0x02c8, 			//	10	TRISB address
	0x02c8, 			//	11	TRISB address
	0x02c8, 			//	12	TRISB address
	0x02c8, 			//	13	TRISB address
	0x02c8, 			//	14	TRISB address
	0x02c8 				//	15	TRISB address
};

//static unsigned int LAT_LUT[] =
const __attribute__((space(auto_psv))) unsigned int LAT_LUT[] =
{
	0b0000000000000001,	//	0 	RB0
	0b0000000000000010,	//	1 	RB1
	0b0000000000000100,	//	2 	RB2
	0b0000000000001000,	//	3 	RB3
	0b0000000000010000,	//	4 	RB4
	0b0000000000100000,	//	5 	RB5
	0b0000000001000000,	//	6 	RB6
	0b0000000010000000,	//	7 	RB7
	0b0000000100000000,	//	8 	RB8
	0b0000001000000000,	//	9 	RB9
	0b0000010000000000,	//	10 	RB10
	0b0000100000000000,	//	11 	RB11
	0b0001000000000000,	//	12 	RB12
	0b0010000000000000,	//	13 	RB13
	0b0100000000000000,	//	14 	RB14
	0b1000000000000000,	//	15 	RB15
	0b0010000000000000,	//	16 	RC13
	0b0100000000000000,	//	17 	RC14
	0b0000000000000001,	//	18 	RD0
	0b0000000000000010,	//	19 	RD1
	0b0000000000000100,	//	20 	RD2
	0b0000000000001000,	//	21 	RD3
	0b0000000000010000,	//	22 	RD4
	0b0000000000100000,	//	23 	RD5
	0b0000000001000000,	//	24 	RD6
	0b0000000010000000,	//	25 	RD7
	0b0000000100000000,	//	26 	RD8
	0b0000100000000000,	//	27 	RD11
	0b0000000000000001,	//	28 	RE0
	0b0000000000000010,	//	29 	RE1
	0b0000000000000100,	//	30 	RE2
	0b0000000000001000,	//	31 	RE3
	0b0000000000010000,	//	32 	RE4
	0b0000000000100000,	//	33 	RE5
	0b0000000001000000,	//	34 	RE6
	0b0000000010000000,	//	35 	RE7
	0b0000000000000001,	//	36 	RF0
	0b0000000000000010,	//	37 	RF1
	0b0000000000001000,	//	38 	RF3
	0b0000000000010000,	//	39 	RF4
	0b0000000000100000	//	40 	RF5
};

//static unsigned int LATPORT_LUT[] =
const __attribute__((space(auto_psv))) unsigned int LATPORT_LUT[] =
{
	0x02cc,				// 0 	LATB address
	0x02cc,				// 1	LATB address
	0x02cc,				// 2	LATB address
	0x02cc,				// 3	LATB address
	0x02cc,				// 4	LATB address
	0x02cc,				// 5	LATB address
	0x02cc,				// 6	LATB address
	0x02cc,				// 7	LATB address
	0x02cc,				// 8	LATB address
	0x02cc,				// 9	LATB address
	0x02cc,				// 10	LATB address
	0x02cc,				// 11	LATB address
	0x02cc,				// 12	LATB address
	0x02cc,				// 13	LATB address
	0x02cc,				// 14	LATB address
	0x02cc,				// 15	LATB address
	0x02d4,				// 16	LATC address
	0x02d4,				// 17	LATC address
	0x02dc,				// 18	LATD address
	0x02dc,				// 19	LATD address
	0x02dc,				// 20	LATD address
	0x02dc,				// 21	LATD address
	0x02dc,				// 22	LATD address
	0x02dc,				// 23	LATD address
	0x02dc,				// 24	LATD address
	0x02dc,				// 25	LATD address
	0x02dc,				// 26	LATD address
	0x02dc,				// 27	LATD address
	0x02e4,				// 28	LATE address
	0x02e4,				// 29	LATE address
	0x02e4,				// 30	LATE address
	0x02e4,				// 31	LATE address
	0x02e4,				// 32	LATE address
	0x02e4,				// 33	LATE address
	0x02e4,				// 34	LATE address
	0x02e4,				// 35	LATE address
	0x02ec,				// 36	LATF address
	0x02ec,				// 37	LATF address
	0x02ec,				// 38	LATF address
	0x02ec,				// 39	LATF address
	0x02ec,				// 40	LATF address
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
				if (IFS0bits.T1IF)
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
					if (IFS0bits.T1IF)
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
* Function:        	void __attribute__ ((interrupt, no_auto_psv)) _TMR3Interrupt(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	PIC24F TMR3 noise interrupt
*
* Note:
*
*****************************************************************************/
void __attribute__ ((interrupt, no_auto_psv)) _T3Interrupt(void)
{
	if (IFS0bits.T3IF && IEC0bits.T3IE)
	{
		T3CONbits.TON = 0;

		if(selfNoise.timerCount >= 0)
		{
			selfNoise.timerCount++;
			if(selfNoise.timerCount > 1)
			{
				selfNoise.timerCount = -1;
				selfNoise.noisePresent = 0;
			}
		}
		if(mutNoise.timerCount >= 0)
		{
			mutNoise.timerCount++;
			if(mutNoise.timerCount > 1)
			{
				mutNoise.timerCount = -1;
				mutNoise.noisePresent = 0;
				mutNoise.triedFrequencies = 0;
			}
		}

		TMR3 = 0x0BDB;
		IFS0bits.T3IF = 0;
		T3CONbits.TON = 1;
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
	// Set RB12/CS13 as a digital pin for use in CVD
	// (currently set regardless of CTMU or CVD)
	//
	AD1PCFGLbits.PCFG0 = 0;


	//
	// OSC
	//
	CLKDIV = 0b000000000000000;
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
void delay100usTMR(unsigned short delay)
{
	unsigned short x;

	T1CON = 0b0010000000000000;	// 1:256 @32MHZ or 1 TMR1 count = 16/2 = 0.125uS
	TMR1 = 0;
	PR1 = 800;				// 100muS timeout
	IFS0bits.T1IF = 0;
	T1CONbits.TON = 1;		// turn on TMR1

	for (x=0;x<delay;x++)
	{
		TMR1=0;
		IFS0bits.T1IF = 0;
		while (!IFS0bits.T1IF){}
	}

	T1CONbits.TON = 0;		// turn off TMR1
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
	AD1PCFGL = 0;
	AD1PCFGH = 0;
	AD1CON1 = 0x0000;
	AD1CHS = 0x0000;			// select analog channel 0
	AD1CSSL = 0x0000;
	AD1CSSH = 0x0000;
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

	//
	// set up TMR3 as noise timer
	//
	T3CONbits.TON = 0;
	T3CONbits.TCKPS = 3;		//1:256 prescaler -- 16us resolution
	IEC0bits.T3IE = 1;
	IFS0bits.T3IF = 0;
	TMR3 = 0x0BDB;
	T3CONbits.TON = 1;

	//
	// run a 50ms delay to let everything warm up
	//
	T1CON = 0b0010000000110000;	// 1:256 @32MHZ or 1 TMR1 count = 16/2 = 0.125uS
	TMR1 = 0;
	PR1 = 1562;				// 50mS timeout
	IFS0bits.T1IF = 0;
	T1CONbits.TON = 1;		// turn on TMR1
	while (!IFS0bits.T1IF){}
	T1CONbits.TON = 0;		// turn off TMR1
	//run_50ms_delay();


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
	AD1PCFGL |= 0xffff;//~(TEMPTRISB); 	// all analog configured as digital I/O

	PORTB &= GTEMPLATB;			// Set to 0
	PORTC &= GTEMPLATC;
	PORTD &= GTEMPLATD;
	PORTE &= GTEMPLATE;
	PORTF &= GTEMPLATF;

	LATB &= GTEMPLATB;			// Set to 0
	LATC &= GTEMPLATC;
	LATD &= GTEMPLATD;
	LATE &= GTEMPLATE;
	LATF &= GTEMPLATF;

	TRISB &= TEMPTRISB;			// Set to output
	TRISC &= TEMPTRISC;
	TRISD &= TEMPTRISD;
	TRISE &= TEMPTRISE;
	TRISF &= TEMPTRISF;
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
	unsigned int cpTimeout;
	//
	// if we're running a charge pump we need to allow the
	// voltage to re-stabilize, run a short delay.  Note this
	// only might be an issue with voltage doubler where the
	// volatge droops
	//
	if ((userRam.customFlag&USECHARGEPUMPDELAY) == USECHARGEPUMPDELAY)
	{
		cpTimeout = (unsigned int) 8 * userRam.cpTimeOut; // 32us * 8 = 256us per count, 1 = 256us
		T1CON = 0b0010000000110000;	// 32us per count
		TMR1 = 0;
		PR1 = cpTimeout; // 1ms
		IFS0bits.T1IF = 0;
		T1CONbits.TON = 1;
		while (!IFS0bits.T1IF){}
		T1CONbits.TON = 0;
	}
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
		varRam.gtempLATC = varRam.gtempLATCgrp[i];
		varRam.gtempLATD = varRam.gtempLATDgrp[i];
		varRam.gtempLATE = varRam.gtempLATEgrp[i];
		varRam.gtempLATF = varRam.gtempLATFgrp[i];

		CTMUCONbits.IDISSEN = 1;	// discharge CTMU
		IFS0bits.AD1IF = 0;			// clear ADC interrupt
		TMR1 = 0;					// clear the timer
		IFS0bits.T1IF = 0;			// clear T1IF
		CTMUCONbits.IDISSEN = 0;	// stop CTMU discharge

		//
		// Drive TX's
		//
		LATC |= varRam.gtempLATC;
		LATD |= varRam.gtempLATD;
		LATE |= varRam.gtempLATE;
		LATF |= varRam.gtempLATF;

		AD1CON1bits.SAMP = 1;		// Start sampling

		while (!IFS0bits.T1IF){}	// wait appropritae rise time

		AD1CON1bits.SAMP = 0;		// Start converting
		while (!AD1CON1bits.DONE){}

		*varRam.gtempTRISPORT &= varRam.gtempTRISClear;

		LATC &= ~varRam.gtempLATC;
		LATD &= ~varRam.gtempLATD;
		LATE &= ~varRam.gtempLATE;
		LATF &= ~varRam.gtempLATF;

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

	rawRam.rawADC[y] = 0; //initialize this since it will be an accumulator

	for(i=0;i<userRam.stutterMult;i++) //iterate through all stuttering waves
	{
		ADC_CHARGE_TRIS = 0;								//set charge pin to output
		ADC_CHARGE_LAT = 1;								//drive charge pin high

		AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;				//connect charge pin to ADC
		AD1CON1bits.SAMP = 1;								//begin sampling
		*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;		//turn sense channel to input

		//setup temporary latch masks based on current 'wave' (i)
		varRam.gtempLATC = varRam.gtempLATCgrp[i];
		varRam.gtempLATD = varRam.gtempLATDgrp[i];
		varRam.gtempLATE = varRam.gtempLATEgrp[i];
		varRam.gtempLATF = varRam.gtempLATFgrp[i];

		//
		// Drive TX's
		//
		LATC |= varRam.gtempLATC;							//
		LATD |= varRam.gtempLATD;							//
		LATE |= varRam.gtempLATE;							//
		LATF |= varRam.gtempLATF;							//

		AD1CHSbits.CH0SA = varRam.gADCONCHS;				//connect sense channel to ADC
		Nop();

		AD1CON1bits.SAMP = 0;								//start ADC conversion
		Nop();
		Nop();
		Nop();
		AD1CHSbits.CH0SA = 31;								//swtich ADC to unused channel

		*(varRam.gtempTRISPORT+LAT_OFFSET) |= varRam.gtempTRIS;		//drive sense channel high
		*(varRam.gtempTRISPORT) &= ~varRam.gtempTRIS;		//set sense channel as output
		ADC_CHARGE_LAT = 0;								//drive charge pin low
		while (!AD1CON1bits.DONE){}							//wait for ADC conversion to finish

		AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;				//connect ADC to charge pin
		AD1CON1bits.SAMP = 1;								//begin sampling
		rawRam.rawADC[y] += ADC1BUF0;						//store first measurement

		*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;		//set sense channel as input

		LATC &= ~varRam.gtempLATC;							//Clear TX lines
		LATD &= ~varRam.gtempLATD;							//
		LATE &= ~varRam.gtempLATE;							//
		LATF &= ~varRam.gtempLATF;							//

		AD1CHSbits.CH0SA = varRam.gADCONCHS;				//connect sense channel to ADC
		Nop();

		AD1CON1bits.SAMP = 0;								//start ADC conversion
		Nop();
		Nop();
		Nop();
		*(varRam.gtempTRISPORT+LAT_OFFSET) &= ~varRam.gtempTRIS;	//drive sense channel low
		*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;			//restore TRIS to previous state

		while(!AD1CON1bits.DONE){}							//wait for ADC conversion to finish

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
		*varRam.gtempTRISPORT |= varRam.gtempTRIS;	// Set RX channel as input
		CTMUCONbits.IDISSEN = 1;	// discharge CTMU
		IFS0bits.AD1IF = 0;			// clear ADC interrupt
		TMR1 = 0;					// clear the timer
		IFS0bits.T1IF = 0;			// clear T1IF
		CTMUCONbits.IDISSEN = 0;	// stop CTMU discharge

		//
		// Drive TX's
		//
		*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];	//if not pulsing two TX, this line will do nothing
		*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];

		AD1CON1bits.SAMP = 1;		// Start sampling
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

		//
		// Drive TX's
		//
		*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
		*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];

		AD1CON1bits.SAMP = 1;		// Start sampling
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

	ADC_CHARGE_TRIS = 0;								//set charge pin to output
	Nop();
	ADC_CHARGE_LAT = 1;								//drive charge pin high

	AD1CON1bits.SAMP = 1;								//begin sampling
	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;		//turn sense channel to input

	AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;				//connect charge pin to ADC
	Nop();
	AD1CHSbits.CH0SA = varRam.gADCONCHS;				//connect sense channel to ADC
	Nop();
	AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;				//connect charge pin to ADC
	Nop();
	AD1CHSbits.CH0SA = varRam.gADCONCHS;				//connect sense channel to ADC
	Nop();	

	*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];
	Nop();

	AD1CON1bits.SAMP = 0;								//start ADC conversion
	Nop();
	Nop();
	Nop();
	ADC_CHARGE_LAT = 0;								//drive charge pin low
	*(varRam.gtempTRISPORT+LAT_OFFSET) |= varRam.gtempTRIS;		//drive sense channel high
	*(varRam.gtempTRISPORT) &= ~varRam.gtempTRIS;		//set sense channel as output
	while (!AD1CON1bits.DONE){}							//wait for ADC conversion to finish

	*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];

	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;

	temp = ADC1BUF0;						//store first measurement

	AD1CON1bits.SAMP = 1;								//begin sampling
	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;		//set sense channel as input

	AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;				//connect ADC to charge pin
	Nop();
	AD1CHSbits.CH0SA = varRam.gADCONCHS;				//connect sense channel to ADC
	Nop();
	AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;				//connect ADC to charge pin
	Nop();
	AD1CHSbits.CH0SA = varRam.gADCONCHS;				//connect sense channel to ADC
	Nop();

	*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];
	Nop();

	AD1CON1bits.SAMP = 0;								//start ADC conversion
	Nop();
	Nop();
	Nop();
	*(varRam.gtempTRISPORT+LAT_OFFSET) &= ~varRam.gtempTRIS;	//drive sense channel low
	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;			//restore TRIS to previous state
	while(!AD1CON1bits.DONE){}							//wait for ADC conversion to finish

	temp = (1023-ADC1BUF0) - temp;

	AD1PCFGLbits.PCFG0 = 0;
	ADC_CHARGE_TRIS = 0;								//set charge pin to output
	Nop();
	ADC_CHARGE_LAT = 1;								//drive charge pin high

	AD1CON1bits.SAMP = 1;								//begin sampling
	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;		//turn sense channel to input

	AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;				//connect charge pin to ADC
	Nop();
	AD1CHSbits.CH0SA = varRam.gADCONCHS;				//connect sense channel to ADC
	Nop();
	AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;				//connect charge pin to ADC
	Nop();
	AD1CHSbits.CH0SA = varRam.gADCONCHS;				//connect sense channel to ADC
	Nop();	

	*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];
	Nop();

	AD1CON1bits.SAMP = 0;								//start ADC conversion
	Nop();
	Nop();
	Nop();
	ADC_CHARGE_LAT = 0;								//drive charge pin low
	*(varRam.gtempTRISPORT+LAT_OFFSET) |= varRam.gtempTRIS;		//drive sense channel high
	*(varRam.gtempTRISPORT) &= ~varRam.gtempTRIS;		//set sense channel as output
	while (!AD1CON1bits.DONE){}							//wait for ADC conversion to finish

	*(varRam.gtempLATPORT[1]) |= varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) |= varRam.gtempLAT[0];

	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;

	rawRam.rawADC[y] = ADC1BUF0;						//store first measurement

	AD1CON1bits.SAMP = 1;								//begin sampling
	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;		//set sense channel as input

	AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;				//connect ADC to charge pin
	Nop();
	AD1CHSbits.CH0SA = varRam.gADCONCHS;				//connect sense channel to ADC
	Nop();
	AD1CHSbits.CH0SA = ADC_CHARGE_CHANNEL;				//connect ADC to charge pin
	Nop();
	AD1CHSbits.CH0SA = varRam.gADCONCHS;				//connect sense channel to ADC
	Nop();

	*(varRam.gtempLATPORT[1]) &= ~varRam.gtempLAT[1];
	*(varRam.gtempLATPORT[0]) &= ~varRam.gtempLAT[0];
	Nop();

	AD1CON1bits.SAMP = 0;								//start ADC conversion
	Nop();
	Nop();
	Nop();
	*(varRam.gtempTRISPORT+LAT_OFFSET) &= ~varRam.gtempTRIS;	//drive sense channel low
	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;			//restore TRIS to previous state
	while(!AD1CON1bits.DONE){}							//wait for ADC conversion to finish

	rawRam.rawADC[y] = (1023-ADC1BUF0) - rawRam.rawADC[y];

	rawRam.rawADC[y] = temp - rawRam.rawADC[y];
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

	T1CON = 0b0010000000110000;	// 1:256 @32MHZ or 1 TMR1 count = 16/2 = 0.125uS
	TMR1 = 0;
	PR1 = 625;				// 20mS timeout
	IFS0bits.T1IF = 0;
	T1CONbits.TON = 1;		// turn on TMR1
	I2C_INT = 1;

	while (commRam.txCount != commRam.txWritten)
	{
		if (IFS0bits.T1IF)
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
	T1CONbits.TON = 0;		// turn off TMR1
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
