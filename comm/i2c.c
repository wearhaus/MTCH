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
* name                  11/17/11    ...
******************************************************************************/
/*
	TSCG				11/17/11


*/
#include "main\main.h"
//#include "plib.h"


//
// Globals
//
extern COMMRAM commRam;

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
	#ifdef PCAPPIC32MX	
	LATBCLR = I2C_INT;
	TRISBCLR = I2C_INT;
	OpenI2C1(I2C_ON | I2C_7BIT_ADD | I2C_STR_EN, 200); // 200 not used
	I2C1ADD = 0x4A>>1;				// Our device address
	I2C1MSK = 0;//xff;				// No mask
	mI2C1SetIntPriority(I2C_INT_PRI_3 | I2C_INT_SLAVE);
	IFS1CLR = 0x00000800;		// clear I2CSIF
	IEC1SET = 0x00000800;		// set I2CSIE
	#endif // ifdef PCAPPIC32MX

	#ifdef PCAPPIC24F
	I2C_INT = 0;				// setup the interrupt pin
	I2C_INT_TRIS = 0;
	OpenI2C1(I2C_ON | I2C_7BIT_ADD | I2C_STR_EN, 200); // 200 not used
	I2C1ADD = 0x4A>>1;				// Our device address
	I2C1MSK = 0;//xff;				// No mask
	IPC4bits.SI2C1IP = 4;			// interrupt priority 4
	IFS1bits.SI2C1IF = 0;			// clear interrupt flag
	IEC1bits.SI2C1IE = 1;			// enable slave interrupt
	#endif // ifdef PCAPPIC24F
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
	#ifdef PCAPPIC32MX
	//
	// Set TMR1 for a 25ms timeout
	//
	T1CON = 0x00000030; // 1:1 - 31.25ns, 1:256 - 8us per count @ 32MHZ
	TMR1=0;
	PR1 = 0x09c4;//0x005CA;	// 20mS timeout//0x002E5;		// 10mS timeout
	IFS0bits.T1IF = 0;
	T1CONbits.ON = 1;
	LATBSET = I2C_INT;
	#endif
	#ifdef PCAPPIC24F
	T1CON = 0b0010000000110000;	// 1:256 @32MHZ or 1 TMR1 count = 16/2 = 0.125uS
	TMR1 = 0;
	PR1 = 625;				// 20mS timeout
	IFS0bits.T1IF = 0;
	T1CONbits.TON = 1;		// turn on TMR1
	I2C_INT = 1;
	#endif
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
				#ifdef PCAPPIC32MX
				I2C1CONbits.ON = 0;
				#endif
				#ifdef PCAPPIC24F
				I2C1CONbits.I2CEN = 0;
				#endif
				initI2C();
			}
			//I2C1CONbits.SCLREL = 1;
			break;
		}
	} // wait for data consumed

	#ifdef PCAPPIC32MX
	LATBCLR = I2C_INT;
	T1CONbits.ON = 0;
	#endif
	#ifdef PCAPPIC24F
	I2C_INT = 0;
	T1CONbits.TON = 0;		// turn off TMR1
	#endif
	commRam.txCount = 0;
	commRam.txWritten = 0;	
	
}
#endif //ifdef I2C
