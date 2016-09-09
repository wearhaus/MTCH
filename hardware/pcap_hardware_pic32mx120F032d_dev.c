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

#include "main/main.h"
#include "hardware/pcap_hardware_pic32mx120F032d_dev.h"

#ifdef DISPLAY
	#include "display\lcddisplay.h"
#endif



//
// Globals
//
extern USERRAM userRam;
extern HWCFGRAM hwCfgRam;
extern COMMRAM commRam;
extern RAWRAM rawRam;
extern VARRAM varRam;
extern BASERAM baseRam;
extern NOISERAM selfNoise;
extern NOISERAM mutNoise;
extern DEBUGRAM debugRam;
extern HWSTATUSRAM hwStatusRam;
#ifdef LONGADCSCAN
extern unsigned short longADC[512];
#endif

extern unsigned char selfScanFineTune[MAXRX];
extern unsigned char mutScanFineTune[MAXRX];
extern volatile unsigned short globalCounter;
extern unsigned char sleepFlag;
extern unsigned short sleepCount;
extern unsigned short baselineCounter;
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

//
//
// Configuration Bit settings
//
// SYSCLK = 38 MHz (8MHz FRC/ FPLLIDIV * FPLLMUL / FPLLODIV)
// SYSCLK = 8MHZ/2 * 19 / 2 = 38MHZ
// SYSCLK = 8MHZ/2 * 16 / 2 = 32MHZ
// PBCLK(TPB) = 32MHZ
// PBCLK = 32 MHz
#pragma config FPLLMUL  = MUL_16        // PLL Multiplier
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_2         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS128         // Watchdog Timer Postscale
#pragma config FCKSM    = CSECMD        // Clock Switching enabled & Fail Safe Clock Monitor disabled
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = OFF            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable
#pragma config FNOSC    = FRCPLL        // Oscillator Selection
#pragma config CP       = ON//OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select
#pragma config DEBUG    = ON           // Debugger Disabled for Starter Kit
#pragma config JTAGEN	= OFF			// JTAG is off

//
// define the look up tables for TRIS, and LAT, and
// analog select, note these are in the same order as
// RX and TX PINMAP's
//
const __attribute__((space(prog))) unsigned int TRIS_LUT[] =
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
	0b0001000000000000,	//	12 	RB12, AN12
};

//static unsigned int TRISPORT_LUT[] =
const __attribute__((space(prog))) unsigned int TRISPORT_LUT[] =
{
	0xbf886010,			//	0	TRISA address
	0xbf886010,			//	1	TRISA address
	0xbf886110,			//	2	TRISB address
	0xbf886110,			//	3	TRISB address
	0xbf886110,			//	4	TRISB address
	0xbf886110,			//	5	TRISB address
	0xbf886210,			//	6	TRISC address
	0xbf886210,			//	7	TRISC address
	0xbf886210,			//	8	TRISC address
	0xbf886110,			//	9	TRISB address
	0xbf886110,			//	10	TRISB address
	0xbf886110,			//	11	TRISB address
	0xbf886110			//	12	TRISB address
};

//static unsigned int LAT_LUT[] =
const __attribute__((space(prog))) unsigned int LAT_LUT[] =
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
	0b0000000001000000,	//	15 	RB6
	0b0000000010000000,	//	16 	RB7
	0b0000000100000000,	//	17 	RB8
	0b0000001000000000,	//	18 	RB9
	0b0000010000000000,	//	19 	RB10
	0b0000100000000000,	//	20 	RB11
	0b0001000000000000,	//	21 	RB12
	0b0010000000000000,	//	22 	RB13
	0b0100000000000000,	//	23 	RB14
	0b1000000000000000,	//	24 	RB15
	0b0000000000000001,	//	25 	RC0
	0b0000000000000010,	//	26 	RC1
	0b0000000000000100,	//	27 	RC2
	0b0000000000001000,	//	28 	RC3
	0b0000000000010000,	//	29 	RC4
	0b0000000000100000,	//	30 	RC5
	0b0000000001000000,	//	31 	RC6
	0b0000000010000000,	//	32 	RC7
	0b0000000100000000,	//	33 	RC8
	0b0000001000000000	//	34 	RC9
};

//static unsigned int LATPORT_LUT[] =
const __attribute__((space(prog))) unsigned int LATPORT_LUT[] =
{
	0xbf886030,			//	0	LATA address
	0xbf886030,			//	1	LATA address
	0xbf886030,			//	2	LATA address
	0xbf886030,			//	3	LATA address
	0xbf886030,			//	4	LATA address
	0xbf886030,			//	5	LATA address
	0xbf886030,			//	6	LATA address
	0xbf886030,			//	7	LATA address
	0xbf886030,			//	8	LATA address
	0xbf886130,			//	9	LATB address
	0xbf886130,			//	10	LATB address
	0xbf886130,			//	11	LATB address
	0xbf886130,			//	12	LATB address
	0xbf886130,			//	13	LATB address
	0xbf886130,			//	14	LATB address
	0xbf886130,			//	15	LATB address
	0xbf886130,			//	16	LATB address
	0xbf886130,			//	17	LATB address
	0xbf886130,			//	18	LATB address
	0xbf886130,			//	19	LATB address
	0xbf886130,			//	20	LATB address
	0xbf886130,			//	21	LATB address
	0xbf886130,			//	22	LATB address
	0xbf886130,			//	23	LATB address
	0xbf886130,			//	24	LATB address
	0xbf886230,			//	25	LATC address
	0xbf886230,			//	26	LATC address
	0xbf886230,			//	27	LATC address
	0xbf886230,			//	28	LATC address
	0xbf886230,			//	29	LATC address
	0xbf886230,			//	30	LATC address
	0xbf886230,			//	31	LATC address
	0xbf886230,			//	32	LATC address
	0xbf886230,			//	33	LATC address
	0xbf886230			//	34	LATC address
};


void _general_exception_handler(unsigned int cause, unsigned int status) 
{ 
	Nop();
	Nop();
} 

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
* Overview:        	PIC32MX I2C interrupt function
*
* Note:
*
*****************************************************************************/
#ifdef I2C
void __ISR(_I2C_1_VECTOR, IPL7AUTO) _SlaveI2CHandler(void)
{
	unsigned char x;

	//
	// I2C interrupt
	//
	RCONbits.WDTO = 0;		//clear WDT flag
	WDTCONbits.ON = 0;		//turn off WDT
	if (IFS1bits.I2C1SIF && IEC1bits.I2C1SIE)
	{
		//
		// check for MASTER and Bus events and respond accordingly
		//
		if (IFS1bits.I2C1MIF == 1)
		{
			mI2C1MClearIntFlag();
			return;
		}
		if (IFS1bits.I2C1BIF == 1)
		{
			//mI2C1BClearIntFlag();
			IFS1CLR = _IFS1_I2C1BIF_MASK;
			return;
		}

		if (I2C1STATbits.R_W == 0)
		{
			//
			// R/W bit = 0 --> indicates data transfer is input to slave
			// D/A bit = 0 --> indicates last byte was address
			// D/A bit = 1 --> indicates last byte was data
			// reset any state variables needed by a message sequence
			// perform a dummy read of the address
			//
			x = I2C1RCV;
			if (I2C1STATbits.D_A == 1)
			{ // it's data, so save it :)
				if((commRam.rcvCount == 0) && (x != 0x55))
				{
					commRam.rcvBuffer[0] = 0x55;
					commRam.rcvCount++;
				}
				commRam.rcvBuffer[commRam.rcvCount] = x;
				commRam.rcvCount ++;
				if (commRam.rcvCount > RCVBUFFERSIZE)
				{
					commRam.rcvCount = 0;
				}
			}
			I2C1STATbits.I2COV = 0; // clear the overflow bit
			// release the clock to restart I2C
			I2C1CONbits.SCLREL = 1; // release the clock
		}
		else
		{
			if (I2C1STATbits.D_A == 0)
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
				x = I2C1RCV;
				I2C1STATbits.I2COV = 0;
				I2C_IRQ = 0;			// clear the interrupt
				I2C1TRN = commRam.txCount;//commRam.txBuffer[commRam.txWritten];
				I2C1CONbits.SCLREL = 1;    // Release the clock
			}
			else
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
	//				commRam.txWritten ++;
					I2C1CONbits.SCLREL = 1;    // Release the clock
				}
				if (commRam.txCount-1 == commRam.txWritten)
				{
					while(!I2C1STATbits.P)
					{
						if (IFS0bits.T1IF)
						{
							break;
						}
					}
				}
				commRam.txWritten++;
			}
		}
		IFS1CLR = 0x00000800;		// clear I2CSIF
	}
	varRam.flag |= INTERRUPTOCCURRED;	// indicate interrupt occurred
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
void __ISR(_TIMER_4_VECTOR, IPL1AUTO) _T4Interrupt(void)
{
	if(IFS0bits.T4IF && IEC0bits.T4IE)          // Timer4 enable and interrupt occur
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
		if(sleepCount > hwCfgRam.sleepTimeout)//userRam.sleepTimeout)		//sleep if no touch for 8s
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
				//selfNoise.timerCount = -1;
				selfNoise.noisePresent = 0;
				selfNoise.triedFrequencies = 0;
			}
			if(selfNoise.timerCount > 5000)
			{
				userRam.selfSampleFreq = 1;
				selfNoise.timerCount = -1;
			}
		}

		if(mutNoise.timerCount >= 0)
		{
			mutNoise.timerCount++;
			if(mutNoise.timerCount > 1000)			//1s timeout
			{
			//	mutNoise.timerCount = -1;
				mutNoise.noisePresent = 0;
				mutNoise.triedFrequencies = 0;
			}
			if(mutNoise.timerCount > 5000)
			{
				userRam.mutSampleFreq = 1;
				mutNoise.timerCount = -1;
			}
		}

        PR4 = 500;
        TMR4 = 0x0000;
        IFS0bits.T4IF = 0;                         // reset interruptflag
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
	// OSC
	//

	//
	// clear interrupts
	//

	//
	// Disable Watch Dog Timer
	//
	//CVRCONbits.CVRSS = 1;
	//CVRCONbits.ON = 1;

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
	unsigned char tempVar;
	int x = 0;
	unsigned int *ptr;
	INTEnableSystemMultiVectoredInt();

	// build the LAT and TRIS Masks
	varRam.maskPortA = 0x0000;
	varRam.maskPortB = 0x0000;
	varRam.maskPortC = 0x0000;
	for (x = 0; x < userRam.numberOfRXChannels; x++)
	{
		// For each RX channel, we need to set our TRIS
		tempVar = userRam.rxPinMap[x];
		ptr = (unsigned int*)TRISPORT_LUT[tempVar];
		if (ptr == &TRISA)
		{
			varRam.maskPortA |= TRIS_LUT[tempVar];
		}
		else if (ptr == &TRISB)
		{
			varRam.maskPortB |= TRIS_LUT[tempVar];
		}
		else// if (ptr == &TRISC)
		{
			varRam.maskPortC |= TRIS_LUT[tempVar];
		}
	}

	for (x = 0; x < userRam.numberOfTXChannels; x++)
	{
		// For each TX channel, we need to set our TRIS
		tempVar = userRam.txPinMap[x];
		ptr = (unsigned int*)LATPORT_LUT[tempVar];
		if (ptr == &LATA)
		{
			varRam.maskPortA |= LAT_LUT[tempVar];
		}
		else if (ptr == &LATB)
		{
			varRam.maskPortB |= LAT_LUT[tempVar];
		}
		else// if (ptr == &TRISC)
		{
			varRam.maskPortC |= LAT_LUT[tempVar];
		}
	}
}

/******************************************************************************
* Function:        	void readEEPROMToRAM(unsigned int addr)
*
* PreCondition:    	None
*
* Input:           	address of ROM location
*
* Output:          	1 - error, 0 - success
*
* Side Effects:    	None
*
* Overview:        	call functions to read from ROM (EEPROM emulation)
*
* Note:
*
*****************************************************************************/
unsigned char readEEPROMToRAM(unsigned int addr)
{
	unsigned int *romPtr;

	//
	//check first word to see if there is valid RAM data stored in ROM
	//
	romPtr = (unsigned int*)addr;
	if(*romPtr != 0x87654321)
	{
		return _ERROR;
	}
	romPtr++;

	//
	//read RAM structures from ROM
	//
	if(EE_Read((unsigned int*)(&userRam), sizeof(userRam), &romPtr))
	{
		return _ERROR;
	}
	if(EE_Read((unsigned int*)(&hwCfgRam), sizeof(hwCfgRam), &romPtr))
	{
		return _ERROR;
	}
	if(EE_Read((unsigned int*)(&selfScanFineTune), sizeof(selfScanFineTune), &romPtr))
	{
		return _ERROR;
	}
	if(EE_Read((unsigned int*)(&mutScanFineTune), sizeof(mutScanFineTune), &romPtr))
	{
		return _ERROR;
	}

	return _SUCCESS;
}

/******************************************************************************
* Function:        	void EE_Read(unsigned int *data, unsigned char size, unsigned int** addr)
*
* PreCondition:    	None
*
* Input:           	pointer to RAM structure, structure size, storage address
*
* Output:          	1 - error, 0 - success
*
* Side Effects:    	None
*
* Overview:        	Read RAM structure from ROM (EEPROM emulation)
*
* Note:
*
*****************************************************************************/
unsigned char EE_Read(unsigned int *data, unsigned char size, unsigned int **addr)
{
	unsigned char x, index;
	unsigned int packData;

	x = 0;
	index = 0;

	//
	//read ROM one 32-bit word at a time
	//
	if(size > 3)
	{
		while(x < (size-3))
		{
			data[index] = **addr;
			(*addr)++;
			index++;
			x+=4;
		}
	}

	//
	//read final 32-bit word and parse it (little endian) one char at a time
	//
	if(size > x)
	{
		packData = **addr;
		while(x < size)
		{
			((unsigned char *)data)[x] = (unsigned char)(packData&0x000000ff);
			packData >>= 8;
			x++;
		}
		(*addr)++;
	}

	return _SUCCESS;
}
/******************************************************************************
* Function:        	void writeRAMToEEPROM(unsigned int addr)
*
* PreCondition:    	ROM locations denoted by addr must be writable
*
* Input:           	ROM address
*
* Output:          	1 - error, 0 - success
*
* Side Effects:    	takes up program memory
*
* Overview:        	Writes RAM structures to ROM (EEPROM emulation)
*
* Note:
*
*****************************************************************************/
unsigned char writeRAMToEEPROM(unsigned int addr)
{
	unsigned int *romPtr;

	//
	//check if data is in EEPROM space and erase it if present
	//
	romPtr = (unsigned int*)addr;
	if(*romPtr == 0x87654321)
	{
		if(eraseEEPROM(EE_ADDRESS))
		{
			return _ERROR;
		}
	}

	//
	//write 0x87654321 as identifier that actual data is stored in ROM
	//

	romPtr++;

	//
	//write RAM structures to ROM
	//
	if(EE_Write((unsigned int*)(&userRam), sizeof(userRam), &romPtr))
	{
		return _ERROR;
	}
	if(EE_Write((unsigned int*)(&hwCfgRam), sizeof(hwCfgRam), &romPtr))
	{
		return _ERROR;
	}
	if(EE_Write((unsigned int*)(&selfScanFineTune), sizeof(selfScanFineTune) ,&romPtr))
	{
		return _ERROR;
	}
	if(EE_Write((unsigned int*)(&mutScanFineTune), sizeof(mutScanFineTune), &romPtr))
	{
		return _ERROR;
	}


	romPtr = (unsigned int*)addr;
	if(NVMWriteWord((void*)romPtr, 0x87654321))
	{
		return _ERROR;
	}
	return _SUCCESS;
}

/******************************************************************************
* Function:        	unsigned char EE_Write(unsigned int *data, unsigned int size, unsigned int** addr)
*
* PreCondition:    	ROM locations denoted by addr must be writable
*
* Input:           	pointer to RAM structure, structure size, storage address
*
* Output:          	1 - error, 0 - success
*
* Side Effects:    	None
*
* Overview:        	Write RAM structure to ROM (EEPROM emulation)
*
* Note:
*
***********************************************/
unsigned char EE_Write(unsigned int* data, unsigned char size, unsigned int** addr)
{
	unsigned char x, index, shiftVal;
	unsigned int packData, tempData;

	x = 0;
	index = 0;
	shiftVal = 0;
	packData = 0x00000000;

	//
	//write 32-bit word one at a time - watching for final word if it's not 32 bits
	//
	if(size > 3)
	{
		while(x < (size-3))
		{
			if(NVMWriteWord((void*)(*addr), data[index]))
			{
				return _ERROR;
			}
			x+=4;
			index++;
			(*addr)++;
		}
	}

	//
	//pack final 32-bit word one char at a time (little endian) and send it
	//
	if(size > x)
	{
		while(x < size)
		{
			tempData = ((unsigned char *)data)[x];
			tempData <<= shiftVal;
			packData += tempData;
			shiftVal += 8;
			x++;
		}

		if(NVMWriteWord((void*)(*addr), packData))
		{
			return _ERROR;
		}
		(*addr)++;
	}
	return _SUCCESS;
}

/******************************************************************************
* Function:        	void eraseEEPROM(unsigned int addr)
*
* PreCondition:    	4096 bytes of program memory starting at addr must be writable
*
* Input:           	ROM address
*
* Output:          	1 - error, 0 - success
*
* Side Effects:    	None
*
* Overview:        	Erases 1 page in ROM (4096 contiguous bytes)
*
* Note:
*
*****************************************************************************/
unsigned char eraseEEPROM(unsigned int addr)
{
	if(NVMErasePage((void*)addr))
	{
		return _ERROR;
	}
	return _SUCCESS;
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
	//
	// setup the ADC, use PBCLK which is now 38MHZ, this means TAD
	// must be 3
	// 1/(3 *(12 + 1) * 26.3ns) = 974.9 ksps
	//
	AD1CON1CLR = 0xFFFF;			// turn ADC off
	AD1CON1 = 0b0010000000000000;	// 16 bit right justified
	AD1CON2 = 0b0000000000000000;
	AD1CON3CLR = 0xFFFFFFFF;
	AD1CHS = 0;
	AD1CSSL = 0;
	AD1CON1SET = 0x8000;			// turn ADC on
	//setup_adc();

	//
	//setup TM4 as 1ms global timer
	//
	T4CON = 0x00000000;
	T4CONbits.TCKPS = 0b110;	//1:64 prescaler = 4us timeout
	IEC0bits.T4IE = 1;			//enable interrupt
	IFS0bits.T4IF = 0;			//clear TMR4 interrupt flag
	PR4 = 500;					//4us * 250 = 1ms
	IPC4bits.T4IP = 1;
	T4CONbits.ON = 1;

	//
	// run a 50ms delay to let everything warm up
	//
	IEC0bits.T1IE = 0;
	T1CON = 0x00000030; // 1:1 - 31.25ns, 1:256 - 8us per count @ 32MHZ
	TMR1=0;
	PR1 = 0x00E78;		// 50mS timeout
	IFS0bits.T1IF = 0;
	T1CONbits.ON = 1;
	while(!IFS0bits.T1IF){}
	T1CONbits.ON = 0;
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
	ANSELACLR = varRam.maskPortA;//~(TEMPTRISA);	// all analog configured as digital I/O
	#ifndef I2C
	ANSELBCLR = varRam.maskPortB;//~(TEMPTRISB);
	#else
	ANSELBCLR = ((varRam.maskPortB) | 0b0000001100000000); // SCL and SDA digital I/O
	#endif
	ANSELCCLR = (varRam.maskPortC);

	PORTACLR = (varRam.maskPortA);
	PORTBCLR = (varRam.maskPortB);
	PORTCCLR = (varRam.maskPortC);

	TRISACLR = (varRam.maskPortA);
	TRISBCLR = (varRam.maskPortB);
	TRISCCLR = (varRam.maskPortC);
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

	I2C1CONbits.DISSLW = 1;	//must disable for RA0 and RA1 - errata silicon issue #9

	rawRam.rawADC[y] = 0; //initialize this since it will be an accumulator

	for(i=0;i<userRam.stutterMult;i++) //iterate through all stuttering waves
	{
		if(varRam.gADCONCHS == CVD_CHARGE_CHS2)
		{
			CVD_CHARGE_TRIS1 = 0;
			Nop();
			CVD_CHARGE_LAT1 = 1;
			AD1CHSbits.CH0SA = CVD_CHARGE_CHS1;
		}
		else
		{
			CVD_CHARGE_TRIS2 = 0;
			Nop();
			CVD_CHARGE_LAT2 = 1;
			AD1CHSbits.CH0SA = CVD_CHARGE_CHS2;
		}
		AD1CON1SET = 0x00000002;	// Start sampling
		*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;

		//setup temporary latches based on 'wave' banks
		varRam.gtempLATA = varRam.gtempLATAgrp[i];
		varRam.gtempLATB = varRam.gtempLATBgrp[i];
		varRam.gtempLATC = varRam.gtempLATCgrp[i];
		//
		// Drive TX's
		//
		LATASET = varRam.gtempLATA;
		LATBSET = varRam.gtempLATB;
		LATCSET = varRam.gtempLATC;

		AD1CHSbits.CH0SA = varRam.gADCONCHS;
		Nop();
		Nop();
		Nop();

		AD1CON1CLR = 0x00000002;	// Start conversion

		*(varRam.gtempTRISPORT+LAT_OFFSET) |= varRam.gtempTRIS;
		*(varRam.gtempTRISPORT) &= ~varRam.gtempTRIS;
		if(varRam.gADCONCHS == CVD_CHARGE_CHS2)
		{
			CVD_CHARGE_LAT1 = 0;
		}
		else
		{
			CVD_CHARGE_LAT2 = 0;
		}
		while (!AD1CON1bits.DONE){}

		if(varRam.gADCONCHS == CVD_CHARGE_CHS2)
		{
			AD1CHSbits.CH0SA = CVD_CHARGE_CHS1;
		}
		else
		{
			AD1CHSbits.CH0SA = CVD_CHARGE_CHS2;
		}
		AD1CON1SET = 0x00000002;	// Start sampling
		rawRam.rawADC[y] += ADC1BUF0;

		*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;

		LATACLR = (varRam.maskPortA);//GTEMPLATA;
		LATBCLR = (varRam.maskPortB);//GTEMPLATB;
		LATCCLR = (varRam.maskPortC);//GTEMPLATC;

		AD1CHSbits.CH0SA = varRam.gADCONCHS;
		Nop();
		Nop();
		Nop();

		AD1CON1CLR = 0x00000002;	// Start conversion

		*(varRam.gtempTRISPORT+LAT_OFFSET) &= ~varRam.gtempTRIS;
		*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;
		while(!AD1CON1bits.DONE){}

		rawRam.rawADC[y] += (1023 - ADC1BUF0);
	}
	I2C1CONbits.DISSLW = 0;
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

	I2C1CONbits.DISSLW = 1;				//must disable for RA0 and RA1 - errata silicon issue #9
	if(varRam.gADCONCHS == CVD_CHARGE_CHS2)
	{
		CVD_CHARGE_TRIS1 = 0;
		Nop();
		CVD_CHARGE_LAT1 = 1;
		AD1CHSbits.CH0SA = CVD_CHARGE_CHS1;
	}
	else
	{
		CVD_CHARGE_TRIS2 = 0;
		Nop();
		CVD_CHARGE_LAT2 = 1;
		AD1CHSbits.CH0SA = CVD_CHARGE_CHS2;
	}
	AD1CON1SET = 0x00000002;	// Start sampling
	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;



	//
	// Drive TX's
	//
	*varRam.gtempLATPORT[0] |= varRam.gtempLAT[0];
	*varRam.gtempLATPORT[1] |= varRam.gtempLAT[1];

	AD1CHSbits.CH0SA = varRam.gADCONCHS;
	Nop();

	AD1CON1CLR = 0x00000002;	// Start conversion

	*(varRam.gtempTRISPORT+LAT_OFFSET) |= varRam.gtempTRIS;
	*(varRam.gtempTRISPORT) &= ~varRam.gtempTRIS;
	if(varRam.gADCONCHS == CVD_CHARGE_CHS2)
	{
		CVD_CHARGE_LAT1 = 0;
	}
	else
	{
		CVD_CHARGE_LAT2 = 0;
	}

	while (!AD1CON1bits.DONE){}

	*varRam.gtempLATPORT[0] &= ~(varRam.gtempLAT[0]);
	*varRam.gtempLATPORT[1] &= ~(varRam.gtempLAT[1]);

	if(varRam.gADCONCHS == CVD_CHARGE_CHS2)
	{
		AD1CHSbits.CH0SA = CVD_CHARGE_CHS1;
	}
	else
	{
		AD1CHSbits.CH0SA = CVD_CHARGE_CHS2;
	}
	AD1CON1SET = 0x00000002;	// Start sampling

	temp = ADC1BUF0;

	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;

	*varRam.gtempLATPORT[0] |= varRam.gtempLAT[0];
	*varRam.gtempLATPORT[1] |= varRam.gtempLAT[1];

	AD1CHSbits.CH0SA = varRam.gADCONCHS;
	Nop();

	AD1CON1CLR = 0x00000002;	// Start conversion
	*(varRam.gtempTRISPORT+LAT_OFFSET) &= ~varRam.gtempTRIS;
	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;

	while(!AD1CON1bits.DONE){}

	temp = (1023 - ADC1BUF0) - temp;

	if(varRam.gADCONCHS == CVD_CHARGE_CHS2)
	{
		CVD_CHARGE_TRIS1 = 0;
		Nop();
		CVD_CHARGE_LAT1 = 1;
		AD1CHSbits.CH0SA = CVD_CHARGE_CHS1;
	}
	else
	{
		CVD_CHARGE_TRIS2 = 0;
		Nop();
		CVD_CHARGE_LAT2 = 1;
		AD1CHSbits.CH0SA = CVD_CHARGE_CHS2;
	}
	AD1CON1SET = 0x00000002;	// Start sampling

	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;

	//
	// Drive TX's
	//
	*varRam.gtempLATPORT[0] &= ~(varRam.gtempLAT[0]);
	*varRam.gtempLATPORT[1] &= ~(varRam.gtempLAT[1]);

	AD1CHSbits.CH0SA = varRam.gADCONCHS;
	Nop();

	AD1CON1CLR = 0x00000002;	// Start conversion
	*(varRam.gtempTRISPORT+LAT_OFFSET) |= varRam.gtempTRIS;
	*(varRam.gtempTRISPORT) &= ~varRam.gtempTRIS;
	if(varRam.gADCONCHS == CVD_CHARGE_CHS2)
	{
		CVD_CHARGE_LAT1 = 0;
	}
	else
	{
		CVD_CHARGE_LAT2 = 0;
	}
	while (!AD1CON1bits.DONE){}

	*varRam.gtempLATPORT[0] |= varRam.gtempLAT[0];
	*varRam.gtempLATPORT[1] |= varRam.gtempLAT[1];

	if(varRam.gADCONCHS == CVD_CHARGE_CHS2)
	{
		AD1CHSbits.CH0SA = CVD_CHARGE_CHS1;
	}
	else
	{
		AD1CHSbits.CH0SA = CVD_CHARGE_CHS2;
	}
	AD1CON1SET = 0x00000002;	// Start sampling

	rawRam.rawADC[y] = ADC1BUF0;

	*(varRam.gtempTRISPORT) |= varRam.gtempTRIS;

	*varRam.gtempLATPORT[0] &= ~(varRam.gtempLAT[0]);
	*varRam.gtempLATPORT[1] &= ~(varRam.gtempLAT[1]);

	AD1CHSbits.CH0SA = varRam.gADCONCHS;
	Nop();

	AD1CON1CLR = 0x00000002;	// Start conversion
	*(varRam.gtempTRISPORT+LAT_OFFSET) &= ~varRam.gtempTRIS;
	*(varRam.gtempTRISPORT) &= varRam.gtempTRISClear;
	while(!AD1CON1bits.DONE){}

	rawRam.rawADC[y] = (1023 - ADC1BUF0) - rawRam.rawADC[y];

	rawRam.rawADC[y] -= temp;

	I2C1CONbits.DISSLW = 0;
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
* Overview:        	Initializes I2C
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
	I2C_IRQ = 0;
	I2C_IRQ_TRIS = 0;
	OpenI2C1(I2C_ON | I2C_IDLE_CON | I2C_7BIT_ADD | I2C_STR_EN, 200);
	I2C1ADD = 0x4A>>1;				// Our device address
	I2C1MSK = 0;//xff;				// No mask
	I2C1CONbits.GCEN = 0;
	mI2C1ClearAllIntFlags();
	mI2C1SetIntPriority(7);
	mI2C1SIntEnable(1);
	EnableIntSI2C1;
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
	//
	// Set TMR1 for a 25ms timeout
	//
	 T1CON = 0x00000030; // 1:1 - 31.25ns, 1:256 - 8us per count @ 32MHZ
	 TMR1=0;
	 PR1 = 0x09c4;//0x005CA;	// 20mS timeout//0x002E5;		// 10mS timeout
	 IFS0bits.T1IF = 0;
	 T1CONbits.ON = 1;
	//globalCounter = 25;
	//
	// check for over flow and clear
	//
	if ((I2C1STATbits.RBF) || (I2C1STATbits.I2COV))
	{
		I2C1STATbits.RBF = 0;
		I2C1STATbits.I2COV = 0;
	}
	I2C_IRQ = 1;
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
				I2C1CONbits.ON = 0;
				I2C1STAT = 0;
				initI2C();
			}
			//I2C1CONbits.SCLREL = 1;
			//I2C1STAT = 0;
			I2C1CONbits.ON = 0;
			I2C1STAT = 0;
			initI2C();
			break;
		}
	} // wait for data consumed

	I2C_IRQ = 0;
	T1CONbits.ON = 0;
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
unsigned char checkIO(unsigned char withComm)
{
	unsigned char x;
	unsigned short temp, portAError, portBError, portCError;
	unsigned short rxError, txError;

	rxError = 0;
	txError = 0;

	groundAll();

	//
	//check for shorts to other pins
	//
	portAError = 0;
	portBError = 0;
	portCError = 0;
	for(x=0;x<16;x++)
	{
		LATASET = varRam.maskPortA;
		LATBSET = varRam.maskPortB;
		LATCSET = varRam.maskPortC;
		delay1ms(1);

		if((varRam.maskPortA&(1<<x)) != 0)
		{
			LATACLR = 1<<x;
			delay1ms(1);
			temp = PORTA;
			if((temp&(1<<x)) != 0)
			{
				portAError += (1<<x);
			}
		}

		if((varRam.maskPortB&(1<<x)) != 0)
		{
			LATBCLR = 1<<x;
			temp = PORTB;
			delay1ms(1);
			if((temp&(1<<x)) != 0)
			{
				portBError += (1<<x);
			}
		}

		if((varRam.maskPortC&(1<<x)) != 0)
		{
			LATCCLR = 1<<x;
			temp = PORTC;
			delay1ms(1);
			if((temp&(1<<x)) != 0)
			{
				portCError += (1<<x);
			}
		}

		LATACLR = varRam.maskPortA;
		LATBCLR = varRam.maskPortB;
		LATCCLR = varRam.maskPortC;
	}

	//
	//check for shorts to VDD
	//
	groundAll();

	temp = PORTA;
	portAError = (portAError|(temp&varRam.maskPortA))&varRam.maskPortA;//(temp&varRam.maskPortA);

	temp = PORTB;
	portBError = (portBError|(temp&varRam.maskPortB))&varRam.maskPortB;//(temp&varRam.maskPortB);

	temp = PORTC;
	portCError = (portCError|(temp&varRam.maskPortC))&varRam.maskPortC;//(temp&varRam.maskPortC);

	//
	//check for shorts to GND
	//
	groundAll();

	LATASET = varRam.maskPortA;
	delay1ms(3);
	temp = PORTA;
	portAError = (portAError|((temp&varRam.maskPortA)^varRam.maskPortA))&varRam.maskPortA;
	LATACLR = varRam.maskPortA;

	LATBSET = varRam.maskPortB;
	delay1ms(1);
	temp = PORTB;
	portBError = (portBError|((temp&varRam.maskPortB)^varRam.maskPortB))&varRam.maskPortB;
	LATBCLR = varRam.maskPortB;

	LATCSET = varRam.maskPortC;
	delay1ms(1);
	temp = PORTC;
	portCError = (portCError|((temp&varRam.maskPortC)^varRam.maskPortC))&varRam.maskPortC;
	LATCCLR = varRam.maskPortC;

	groundAll();

	//
	//convert error data from port poins to RX/TX channels
	//
	for(x=0;x<userRam.numberOfRXChannels;x++)
	{
		if(TRISPORT_LUT[userRam.rxPinMap[x]] == 0xbf886010)
		{
			if((portAError&(TRIS_LUT[userRam.rxPinMap[x]])) != 0)
			{
				rxError += (1<<x);
			}
		}
		else if(TRISPORT_LUT[userRam.rxPinMap[x]] == 0xbf886110)
		{
			if((portBError&(TRIS_LUT[userRam.rxPinMap[x]])) != 0)
			{
				rxError += (1<<x);
			}
		}
		else if(TRISPORT_LUT[userRam.rxPinMap[x]] == 0xbf886210)
		{
			if((portCError&(TRIS_LUT[userRam.rxPinMap[x]])) != 0)
			{
				rxError += (1<<x);
			}
		}
	}

	for(x=0;x<userRam.numberOfTXChannels;x++)
	{
		if(LATPORT_LUT[userRam.txPinMap[x]] == 0xbf886030)
		{
			if((portAError&(LAT_LUT[userRam.txPinMap[x]])) != 0)
			{
				txError += (1<<x);
			}
		}
		else if(LATPORT_LUT[userRam.txPinMap[x]] == 0xbf886130)
		{
			if((portBError&(LAT_LUT[userRam.txPinMap[x]])) != 0)
			{
				txError += (1<<x);
			}
		}
		else if(LATPORT_LUT[userRam.txPinMap[x]] == 0xbf886230)
		{
			if((portCError&(LAT_LUT[userRam.txPinMap[x]])) != 0)
			{
				txError += (1<<x);
			}
		}
	}

	//
	//load error information into hardware status registers
	//
	hwStatusRam.rxShortStatus = rxError;
	hwStatusRam.txShortStatus = txError;

	if (rxError || txError)
	{
		hwStatusRam.generalStatus = 1<<0;		//set bit zero
		if (withComm)
		{
			sendCommStatusReport(STATUS_SHORT);
		}
		return 1;
	}
	else
	{
		return 0;
	}
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
void initLCDSPI(void)
{
	unsigned int x;

	//LCD_RST = 0;					// 0 for LCD reset, 1 for active
	LCD_RST_TRIS = 0;				// output
	LCD_A0 = 0;						// LCD A0 line
	LCD_A0_TRIS = 0;				// output
	LCD_RST = 1;					// 0 for LCD reset, 1 for active

	//
	// set the peripreal pin select
	//
	//RPINR20bits.SDI1R = 20;			// SPI Data input on RP20, RC8
	//RPOR11bits.RP22R = 8;			// SPI CLK on RP22, RC6
	//RPOR11bits.RP23R = 7;			// SPI Data output on RP22, RC7
	RPB5R = 3;						// SPI SDO on RB5, SCK on RB14
	SPI_SCK_TRIS = 0;
	SPI_SDO_TRIS = 0;
	//
	// Configure the SPI module
	//
	//SPI1STATbits.SPIEN = 0;			// disable the module
	//SPI1STAT = 0b0000000000010000;
	//SPI1CON1 = 0b0000000001111111;
	//SPI1CON2 = 0;
	//SPI1STATbits.SPIEN = 1;			// enable the module
	SPI1CONbits.ON = 0;					// Disable SPI module
	x = SPI1BUF;
	SPI1CON = 0b00000000000000000000000001110000;
	SPI1CON2 = 0b00000000000000000000000000000000;
	SPI1STAT = 0b00000000000000000000000000000000;
	SPI1BRG = 0x1;
	SPI1CONbits.ON = 1;					// Enable SPI module
	//
	// Use SDI as CS
	//
	SPI_CS = 1;
	SPI_CS_TRIS = 0;
}

/******************************************************************************
* Function:        	sendSPIchar(unsigned char byte)
*
* PreCondition:    	None
*
* Input:           	Byte to send
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Sends a single SPI byte
*
* Note:            	None
*
*****************************************************************************/
void sendSPIchar(unsigned char byte)
{

	//IFS1bits.SPI1RXIF = 0;
	SPI1BUF = byte;
	//while (!IFS1bits.SPI1RXIF){}
	while (SPI1STATbits.SPIBUSY){}
	//delay100usTMR(1);
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
#ifdef DISPLAY
void inline waitOnSPITX(void)
{
	//while (SPI1STAT&0x0700);
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
//	T1CONbits.TON = 0;				//disable TMR1
	T2CONbits.TON = 0;				//disable TMR2
	T4CONbits.TON = 0;				//disable TMR4
	AD1CON1bits.ON = 0;				// disable ADC

	tempIE0 = IEC0;					//save current interrupt enable values
	tempIE1 = IEC1;

	IEC0 = 0x0000;					//disable all interrupts
	IEC1 = 0x0800;

#ifdef I2CDEBUGIRQ
	LATACLR = 0xffff;					//
	LATBCLR = 0xffff;					//LAT output drivers cleared
	LATCCLR = 0xffbf;					//

	TRISASET = 0xffff;					//
	TRISBSET = 0xffff;					//tri-state all I/O before sleeping (sett them to output causes
	TRISCSET = 0xffbf;					//acquisition to get bad measurements and think a touch has occurred)
#else
	LATACLR = 0xffff;					//
	LATBCLR = 0xfbff;					//LAT output drivers cleared
	LATCCLR = 0xffff;					//

	TRISASET = 0xffff;					//
	TRISBSET = 0xfbff;					//tri-state all I/O before sleeping (sett them to output causes
	TRISCSET = 0xffff;					//acquisition to get bad measurements and think a touch has occurred)

#endif


//	TRISACLR = 0xffff;					//
//	TRISBCLR = 0xffff;					//configure I/O as input
//	TRISCCLR = 0xffbf;					//
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
	IEC0SET = tempIE0;
	IEC1SET = tempIE1;

	baselineCounter = 0;
	varRam.baseUpdateCount = 0;
	sleepCount = 0;
	sleepFlag = 0;

	initPCHardware();
	decodeInit();

	initNoise();
	stutterMaskSetup();
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
	peripheralShutdown();
	varRam.flag &= NOTTOUCH;
	commRam.rcvCount = 0;

	while((varRam.flag&TOUCH) == 0 && commRam.rcvCount == 0)
	{
		AD1CON1bits.ADON = 0;
		T2CONbits.TON = 0;
		T4CONbits.TON = 0;

		RCONbits.WDTO = 0;		//clear WDT flag
		WDTCONbits.ON = 1;		//turn on WDT
		PowerSaveSleep();
		RCONbits.WDTO = 0;		//clear WDT flag
		WDTCONbits.ON = 0;		//turn off WDT

		if (commRam.rcvCount == 0)
		{
			AD1CON1bits.ON = 1;
			T2CONbits.TON = 1;				//TMR2
			T4CONbits.TON = 1;				//TMR4

			groundAll();
			initGestures();
			checkTouch();
		}
		//commReceive();
	}
	peripheralWakeup();
}

// Updates hardware configuration based upon parameters in hwCfgRam
void updateHardwareCfg(void)
{
	//WDTCONbits.SWDTPS = hwCfgRam.wdtTimeout; // time
}
