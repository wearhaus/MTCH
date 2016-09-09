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
* name                  11/18/11    ...
******************************************************************************/
/*
	TSCG				11/18/11


*/

/*! \file comm.c
	\brief General Purpose communications functions.

	This file contains all of the communications related functions for the
	projected capacitive firmware.  Note that some of the functions are
	projected capacitive specific - for instance, transmitting the data
	packet.
*/

#include "main\main.h"
#ifdef DISPLAY
	#include "display\lcddisplay.h"
#endif
//
// Globals
//
extern GESTURE gesture[MAX_TOUCHES];
extern USERRAM userRam;
extern HWCFGRAM hwCfgRam;
extern HWSTATUSRAM hwStatusRam;
extern COMMRAM commRam;
extern RAWRAM rawRam;
extern VARRAM varRam;
#ifdef LONGADCSCAN
extern unsigned short longADC[512];
#endif
extern DEBUGRAM debugRam;
extern TOUCHSET touchSet;
extern TOUCHIDSET touchIDSet;

extern unsigned char selfScanFineTune[MAXTX];
extern unsigned char mutScanFineTune[MAXTX];

extern volatile unsigned short globalCounter;
extern unsigned short sleepCount;


static __attribute__((address(0x80000000))) forceBootEntry = 0;

#ifdef ENABLE_BOOTLOADER
void enterBootloader(void)
{
//	DWORD *AppPtr;
	unsigned int dummy;
//	void (*fptr)(void);
//	forceBootEntry = 0x12345678;
//	AppPtr = (DWORD *)BOOTLOADER_ADDRESS;
//	if(*AppPtr != 0xFFFFFFFF)
//	{
//		fptr = (void (*)(void))(0x9D001800);
//		fptr();
//	}
	SYSKEY = 0x00000000;
	SYSKEY = 0xAA996655;	//unlock sequence
	SYSKEY = 0x556699AA;
	RSWRSTSET = 1;			//arm software reset
	dummy = RSWRST;			//fire software reset
	Nop();					//delay for reset to occur
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
	Nop();
}
#endif // #ifdef ENABLE_BOOTLOADER

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
/*
void initUART(void)
{
}
*/


/*
╔══════╦═══════════════════╦══════╦══════╦══════╗
║  ID  ║       Name        ║ 0x01 ║ 0x02 ║ 0x03 ║
╠══════╬═══════════════════╬══════╬══════╬══════╣
║ 0x00 ║ ENABLECONTROLLER  ║ ?    ║ X    ║ X    ║
║ 0x01 ║ DISABLECONTROLLER ║ ?    ║ X    ║ X    ║
║ 0x14 ║ SCANBASELINE      ║ ?    ║ X    ║ X    ║
║ 0x15 ║ WRITERAM          ║ ?    ║ X    ║ X    ║
║ 0x16 ║ READRAM           ║ ?    ║ X    ║ X    ║
║ 0x17 ║ WRITEUSEREEPROM   ║ ?    ║ X    ║ X    ║
║ 0x18 ║ SOFTWARESLEEP     ║ ?    ║ X    ║ X    ║
║ 0x19 ║ ERASEEEPROM       ║ ?    ║ X    ║ X    ║
║ 0x1A ║ CHECKIO           ║ ?    ║ X    ║ X    ║
║ 0x80 ║ CFGIDHIGHCMD      ║ ?    ║ X    ║      ║
║ 0x81 ║ CFGIDLOWCMD       ║ ?    ║ X    ║      ║
║ 0x82 ║ CMDSETVERCMD      ║ ?    ║ X    ║ X    ║
║ 0x83 ║ CFGCMD            ║ ?    ║      ║ X    ║
║ 0xD0 ║ GETDIAGMASKCMD    ║ ?    ║ X    ║ X    ║
║ 0xD1 ║ SETDIAGMASKCMD    ║ ?    ║ X    ║ X    ║
╚══════╩═══════════════════╩══════╩══════╩══════╝

*/


/******************************************************************************
* Function:        	void commReceive(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Retrieves received data if avialble
*
* Note:            	Host - <0x55><DataSize><Data...> - DataSize is the number
*						of bytes to follow after DataSize
*					Controller - <0x55><0x01(only one response byte><Response>
*
*					Response:
*						0x00	DEFAULTSUCCESS
						0xFD	INVALIDPARAMETER
*						0xFE	COMMANDTIMEOUT
*						0xFF	UNRECOGNIZEDCOMMAND
*
*					Commands:
*						0x00	ENABLECONTROLLER
*						0x01	DISABLECONTROLLER
*						0x14	SCANBASELINE
*						0x15	WRITERAM
*						0x16	READRAM
*						0x17	WRITEUSEREEPROM
*						0x18	SOFTWARESLEEP
*
*					Command 0x00 (ENABLECONTROLLER)
*					Send -
*						<0x55><0x01><0x00>
*					Receive -
*						<0x55><0x01><0x00>
*
*					Command 0x00 (DISABLECONTROLLER)
*					Send -
*						<0x55><0x01><0x01>
*					Receive -
*						<0x55><0x01><0x00>
*
*					Command 0x14 (SCANBASELINE)
*					Send -
*						<0x55><0x01><0x14>
*					Receive -
*						<0x55><0x01><0x00>
*
*					Command 0x15 (WRITERAM)
*					Send -
*						<0x55><0x03><0x15><offsetHi><offsetLo><data>
*					Receive -
*						<0x55><0x01><0x00>
*
*					Command 0x16 (READRAM)
*					Send -
*						<0x55><0x02><0x16><offsetHi><offsetLo>
*					Receive -
*						<0x55><0x01><Data>
*
*					Command 0x17 (WRITEUSEREEPROM)
*					Send -
*						<0x55><0x01><0x17>
*					Receive -
*						<0x55><0x01><0x00>
*
*					Command 0x18 (SOFTWARESLEEP)
*					Send -
*						<0x55><0x01><0x18>
*					Receive -
*						<0x55><0x01><0x00> -> Sleep
*
*					Command 0x19 (ERASEEEPROM)
*					Send -
*						<0x55><0x01><0x19>
*					Receive -
*						<0x55><0x01><0x00>
*
*					Command 0x1A (CHECKIO)
*					Send -
*						<0x55><0x01><0x1A>
*					Receive -
*						<0x55><0x08><RXLSB><RX><RX><RXMSB><TXLSB><TX><TX><TXMSB>
*
*					Command 0x80 (CFGCMD)
*					Send -
*						<0x55><0x01><0x80>
*					Receive -
*						<0x55><0x01><Config ID High><Config ID Low> ...
*						<FW ver. Major><FW ver. Minor><Command Set><0><0><0>
*
*					Command 0xB0 (ENTERBOOTLOADERCMD)
*					Send -
*						<0x55><0x01><0xB0>
*					Receive -
*						<0x55><0x02><Result><0xB0>
*						This will cause the firmware to shift into bootloader
*						mode.  If no bootloader command is received after 30
*						seconds it will reset the chip and return to normal
*						firmware.
*
*					Command 0xD0 (GETDIAGMASKCMD)
*					Send -
*						<0x55><0x01><0xD0>
*					Receive -
*						<0x55><size><Result><0xD0><DIAGNOSTIC_MASK_SIZE><Mask0>...<Maskn>
*						Diagnostic messages will only be transmitted if their ID
*						matches one of the MASK IDs as set by SETDIAGMASKCMD.
*						This command returns the list of valid MASK IDs. (Ignoring 0xff MASK IDs)
*
*					Command 0xD1 (SETDIAGMASKCMD)
*					Send -
*						<0x55><0x02 or more><0xD1><MASK1><MASK2>...<MASKn>
*					Receive -
*						<0x55><0x02><Result><0xD1>
*						Diagnostic messages will only be transmitted if their ID
*						matches one of the MASK IDs.  This command sets the list of
*						MASK IDs.  Any unused portions of the MASK will be filled with 0xff.
*
*
*****************************************************************************/
void commReceive(void)
{
#ifdef ENABLE_DEBUG
    unsigned char counter;
#endif
	//#ifdef PCAPPIC18F
	unsigned char timeout=0;
	//#endif

	#ifdef USB
	usbReceive();
	#endif

	if (commRam.rcvCount == (unsigned char)0)
	{
		return;
	}

	if (commRam.rcvBuffer[0] == (unsigned char)0x55) // we start with a header byte
	{
	    commRam.response.result = UNRECOGNIZEDCOMMAND;
	    commRam.response.size = 0;
		//
		// Set TMR for a 50ms timeout
		//
		comm_rcv_timeout_setup();
		for(;;) // wait until we have a complete data set or time-out
		{
			#ifdef USB
			usbReceive();
			#endif
			if (commRam.rcvCount > (unsigned char)1)
			{
				if (commRam.rcvCount >= ((unsigned char)commRam.rcvBuffer[1]+2))
				{
					goto gotData;
				}
			}
			comm_rcv_timeout(); // check for timeout
		}
gotData:
		commRam.response.result = UNRECOGNIZEDCOMMAND;
        commRam.response.command = commRam.rcvBuffer[2];
		switch(commRam.rcvBuffer[2]) // Command
		{
		case ENABLECONTROLLER:
			userRam.flag1 &= NOTDISABLE;
			commRam.response.result = DEFAULTSUCCESS;
			break;
		case DISABLECONTROLLER:
			userRam.flag1 |= DISABLE;
			commRam.response.result = DEFAULTSUCCESS;
			break;
		case WRITERAM:
			cmdAccessRam(WRITERAM);
			break;
		case READRAM:
			cmdAccessRam(READRAM);
			break;
		case SCANBASELINE:
			varRam.flag |= SOFTWAREBASE;
			commRam.response.result = DEFAULTSUCCESS;
			break;
		case WRITEUSEREEPROM:
			writeRAMToEEPROM(EE_ADDRESS);
			commRam.response.result = DEFAULTSUCCESS;
			break;
		case SOFTWARESLEEP:
			sleepCount = hwCfgRam.sleepTimeout;	
			commRam.response.result = DEFAULTSUCCESS;
			break;
		case ERASEEEPROM:
			eraseEEPROM(EE_ADDRESS);
			commRam.response.result = DEFAULTSUCCESS;
			break;
		case CHECKIO:
			commRam.txBuffer[4] = checkIO(0); // suppress normal comm for our command response
			commRam.response.size = 1;
			commRam.response.result = DEFAULTSUCCESS;
			break;
		case CFGCMD:
			commRam.txBuffer[4] = APPIDHIGH;
			commRam.txBuffer[5] = APPIDLOW;
			commRam.txBuffer[6] = FWREVMAJOR;
			commRam.txBuffer[7] = FWREVMINOR;
            commRam.response.size = 4;
            commRam.response.result = DEFAULTSUCCESS;
			break;
#ifdef ENABLE_BOOTLOADER
		case ENTERBOOTLOADERCMD:
			commRam.response.size = 0;
			commRam.response.result = DEFAULTSUCCESS;
			sendCommandResponse();
			enterBootloader();
			break;
#endif // ifdef ENABLE_BOOTLOADER
		case GETDIAGMASKCMD:
			commRam.txBuffer[4] = DEBUG_MASK_SIZE;
			commRam.response.size = 1;
#ifdef ENABLE_DEBUG
			for (counter = 0; counter < DEBUG_MASK_SIZE; counter++)
			{
				if (debugRam.debugMask[counter] != 0xff)
				{
					commRam.txBuffer[4+commRam.response.size++] = debugRam.debugMask[counter];
				}
			}
#endif
			commRam.response.result = DEFAULTSUCCESS;
			break;
		case SETDIAGMASKCMD:
#ifdef ENABLE_DEBUG
			for (counter = 0; (counter < commRam.rcvBuffer[1]-1) && counter < DEBUG_MASK_SIZE; counter++)
			{
				debugRam.debugMask[counter] = commRam.rcvBuffer[3+counter];
			}
			for (; counter < DEBUG_MASK_SIZE; counter++)
			{
				debugRam.debugMask[counter] = DIAGNONE;
			}
#endif
			commRam.response.result = DEFAULTSUCCESS;
			break;
		}
		sendCommandResponse();
		sleepCount = 0;
	}
receiveComplete:
	if (timeout != 0)
	{
		commRam.response.result = COMMANDTIMEOUT;
        commRam.response.command = commRam.rcvCount;//commRam.rcvBuffer[2];
		commRam.response.size = 0;

/*commRam.txBuffer[0]=0x55;
commRam.txBuffer[1]=7;
commRam.txBuffer[2]=commRam.rcvCount;
commRam.txBuffer[3]=commRam.rcvBuffer[0];
commRam.txBuffer[4]=commRam.rcvBuffer[1];
commRam.txBuffer[5]=commRam.rcvBuffer[2];
commRam.txBuffer[6]=commRam.rcvBuffer[3];
commRam.txBuffer[7]=commRam.rcvBuffer[4];
commRam.txBuffer[8]=test1;
commRam.response.size = 7;
commRam.txCount = 9;*/

		sendCommandResponse();
	}
	commRam.rcvCount = 0; // clear and start over
	comm_rcv_timeout_off();
}

#define ACCESSRAM(_location_,_offset_,_value_) \
		if (cmd == WRITERAM) \
		{ \
			((unsigned char*)&(_location_))[_offset_] = _value_; \
		} \
		else \
		{ \
			_value_ = ((unsigned char*)&(_location_))[_offset_]; \
			commRam.response.size = 1; \
		}

// Reads or writes RAM as appropriate
void cmdAccessRam(unsigned char cmd)
{
	unsigned char RAMOffset = 0;//ommRam.rcvBuffer[4];
	unsigned char dataByte = 0;

	commRam.response.size = 0;
	commRam.response.result = DEFAULTSUCCESS;

	if (cmd == WRITERAM)
	{
		if (commRam.rcvCount == 6)
		{
			dataByte = commRam.rcvBuffer[5];
			RAMOffset = commRam.rcvBuffer[4];
		}
		else
		{
			commRam.response.result = PARAMETERCOUNTERROR;
			return;
		}
	}
	else
	{
		if (commRam.rcvCount == 5)
		{
			RAMOffset = commRam.rcvBuffer[4];
		}
		else
		{
			commRam.response.result = PARAMETERCOUNTERROR;
			return;
		}
	}

	switch (commRam.rcvBuffer[3]) // which memory "bank" to work with
	{
		case RAM_USERRAMGENERAL: 	// userRam "general"
			if (RAMOffset < sizeof(userRam))
			{
				ACCESSRAM(userRam,RAMOffset,dataByte);  // reads or writes, as appropriate
				hardwarePostInit();
			}
			else
			{
				commRam.response.result = INVALIDPARAMETER;
			}
			break;
		case RAM_RXPINMAP: // rxPinMap
			if (RAMOffset < sizeof(userRam.rxPinMap))
			{
				ACCESSRAM(userRam.rxPinMap,RAMOffset,dataByte);  // reads or writes, as appropriate
				hardwarePostInit();
			}
			else
			{
				commRam.response.result = INVALIDPARAMETER;
			}
			break;
		case RAM_TXPINMAP: // txPinMap
			if (RAMOffset < sizeof(userRam.txPinMap))
			{
				ACCESSRAM(userRam.txPinMap,RAMOffset,dataByte);  // reads or writes, as appropriate
				hardwarePostInit();
			}
			else
			{
				commRam.response.result = INVALIDPARAMETER;
			}
			break;
		case RAM_SELFPARAM: // Self Parameters
			if (RAMOffset < (sizeof(userRam)-((unsigned int)&userRam.selfScanTime-(unsigned int)&userRam)))
			{
				ACCESSRAM(userRam.selfScanTime,RAMOffset,dataByte);  // reads or writes, as appropriate
			}
			else
			{
				commRam.response.result = INVALIDPARAMETER;
			}
			break;
		case RAM_SELFTUNE: // Self Tuning Array
			if (RAMOffset < sizeof(selfScanFineTune))
			{
				ACCESSRAM(selfScanFineTune,RAMOffset,dataByte);  // reads or writes, as appropriate
			}
			else
			{
				commRam.response.result = INVALIDPARAMETER;
			}
			break;
		case RAM_MUTPARAM: // Mutual Parameters
			if (RAMOffset < (sizeof(userRam)-((unsigned int)&userRam.mutScanTime-(unsigned int)&userRam)))
			{
				ACCESSRAM(userRam.mutScanTime,RAMOffset,dataByte);  // reads or writes, as appropriate
			}
			else
			{
				commRam.response.result = INVALIDPARAMETER;
			}
			break;
		case RAM_MUTTUNE: // Mutual Tuning Array
			if (RAMOffset < sizeof(mutScanFineTune))
			{
				ACCESSRAM(mutScanFineTune,RAMOffset,dataByte);  // reads or writes, as appropriate
			}
			else
			{
				commRam.response.result = INVALIDPARAMETER;
			}
			break;
		case RAM_DECODETRACK: // Decode & Tracking
			if (RAMOffset < (sizeof(userRam)-((unsigned int)&userRam.flipState-(unsigned int)&userRam)))
			{
				ACCESSRAM(userRam.flipState,RAMOffset,dataByte);  // reads or writes, as appropriate
			}
			else
			{
				commRam.response.result = INVALIDPARAMETER;
			}
			break;
		case RAM_NOISE: // Noise & Charge Pump
			if (RAMOffset < (sizeof(userRam)-((unsigned int)&userRam.cpTimeOut-(unsigned int)&userRam)))
			{
				ACCESSRAM(userRam.cpTimeOut,RAMOffset,dataByte);  // reads or writes, as appropriate
			}
			else
			{
				commRam.response.result = INVALIDPARAMETER;
			}
			break;
		case RAM_GESTURES: // Gestures
			if (RAMOffset < (sizeof(userRam)-((unsigned int)&userRam.swipeLengthX-(unsigned int)&userRam)))
			{
				ACCESSRAM(userRam.swipeLengthX,RAMOffset,dataByte);  // reads or writes, as appropriate
			}
			else
			{
				commRam.response.result = INVALIDPARAMETER;
			}
			break;
		case RAM_HWCFG: // Hardware Configuration
			if (RAMOffset < sizeof(hwCfgRam))
			{
				ACCESSRAM(hwCfgRam,RAMOffset,dataByte);  // reads or writes, as appropriate
			}
			else
			{
				commRam.response.result = INVALIDPARAMETER;
			}
			break;
		case RAM_HWSTATUS: // Hardware Status
			if (RAMOffset < sizeof(hwStatusRam))
			{
				ACCESSRAM(hwStatusRam,RAMOffset,dataByte);  // reads or writes, as appropriate
			}
			else
			{
				commRam.response.result = INVALIDPARAMETER;
			}
			break;
		default:
			commRam.response.result = INVALIDPARAMETER;
			break;
	}
	if (cmd == READRAM)
	{
		commRam.txBuffer[4] = dataByte;
	}

}


/******************************************************************************
* Function:        	void sendCommandResponse(unsigned char response)
*
* PreCondition:    	None
*
* Input:           	response contains the response to send
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Sends the response from the command
*
* Note:            	The response format is:
*					<0x55> <0x01> <response to send>
*
*****************************************************************************/
void sendCommandResponse()
{
    unsigned char currentDataIndex = 0;

	commRam.txBuffer[0] = 0x55;
	commRam.txBuffer[1] = commRam.response.size+2;
	commRam.txBuffer[2] = commRam.response.result;
	commRam.txBuffer[3] = commRam.response.command;

//	for (currentDataIndex = 0; currentDataIndex < commRam.response.size; currentDataIndex++)
//	{
//		commRam.txBuffer[currentDataIndex+4] = commRam.response.buffer[currentDataIndex];
//	}

	commRam.txCount = commRam.response.size+4;

	#ifdef UART
	for (currentDataIndex = 0; currentDataIndex < commRam.txCount; currentDataIndex++)
	{
		while(!UART_TRMT)
		{}
		UART_TXREG = commRam.txBuffer[currentDataIndex];
	}
	while(!UART_TRMT)
	{}
	#endif

	#ifdef I2C
		#ifdef TOUCHPADDEMONSTRATOR
			if(!SELECT_PIN)
			{
				writeI2C();
			}
		#else
			writeI2C();
		#endif
	#endif // ifdef I2C

	#ifdef USB
	sendUSBChar(commRam.txBuffer, commRam.txCount);
	#endif // ifdef USB
}

/******************************************************************************
* Function:        	void sendCommTouchReport(void)
*
* PreCondition:    	None
*
* Input:           	scratchRam.p0 is the array number for the report
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	sends the touch report
*
* Note:            	Format
*					Packet	Bit	7	6	5	4	3	2	1	0
*					0			1	t3	t2	t1	t0	0	0	p0
*					1			0	x6	x5	x4	x3	x2	x1	x0
*					2			0	0	0	x11	x10	x9	x8	x7
*					3			0	y6	y5	y4	y3	y2	y1	y0
*					4			0	0	0	y11	y10	y9	y8	y7
*
*					Where:
*					p	- pen status
*					t3:t0 	- touch ID number 0 based
*
*****************************************************************************/
void sendCommTouchReport(unsigned char whichReport)
{
	#ifdef UART
    unsigned char currentDataIndex = 0;
	#endif
    unsigned int xi0 = 0;
	//#ifdef USB
	//char data[5];
	//#endif

	// pertinent coordinates are in touchIDSet.tData[whichReport].touchLoc[0].fineLocation.x;

#ifdef ENABLE_DEBUG
	if (userRam.flag1&CONTROLLERDIAGNOSTICS)
	{
		// Diagnostic Output - we want the following information
		// Touch ID
		// Touch State
		// X Location (2 bytes)
		// Y Locaiton (2 bytes)
        debugRam.ID = DIAGTOUCHREPORT;
        debugRam.size = 12;
        debugRam.buffer[0] = whichReport;
        debugRam.buffer[1] = touchIDSet.tData[whichReport].touchState;
        debugRam.buffer[2] = (touchIDSet.tData[whichReport].touchLoc[0].fineLocation.x>>8)&0xff;
        debugRam.buffer[3] = touchIDSet.tData[whichReport].touchLoc[0].fineLocation.x&0xff;
        debugRam.buffer[4] = (touchIDSet.tData[whichReport].touchLoc[0].fineLocation.y>>8)&0xff;
        debugRam.buffer[5] = touchIDSet.tData[whichReport].touchLoc[0].fineLocation.y&0xff;
        debugRam.buffer[6] = (touchIDSet.tData[whichReport].scaleLocation.x>>8)&0xff;
        debugRam.buffer[7] = touchIDSet.tData[whichReport].scaleLocation.x&0xff;
        debugRam.buffer[8] = (touchIDSet.tData[whichReport].scaleLocation.y>>8)&0xff;
        debugRam.buffer[9] = touchIDSet.tData[whichReport].scaleLocation.y&0xff;
		debugRam.buffer[10] = touchIDSet.tData[whichReport].touchLoc[0].roughLocation.x;
		debugRam.buffer[11] = touchIDSet.tData[whichReport].touchLoc[0].roughLocation.y;
        sendDebugBytes();
	}
#endif // ENABLE_DEBUG

	if (hwCfgRam.touchPacketCfg&PACKETCFGENABLEMASK)
	{
		commRam.txBuffer[0] = 0x80 | whichReport << 3;

		if (touchIDSet.tData[whichReport].touchState != TS_TOUCH_UP)
		{
			commRam.txBuffer[0] |= 0x01; // pen state
		}

		if(userRam.touchSuppressNum == 0 || whichReport<userRam.touchSuppressNum) //supress touch if necessary
		{
			// X Coordinate
			xi0 = touchIDSet.tData[whichReport].scaleLocation.x;
			commRam.txBuffer[1] = xi0 & 0x7f;
			xi0 >>= 7;
			commRam.txBuffer[2] = xi0 & 0x7f;

			// Y Coordinate
			xi0 = touchIDSet.tData[whichReport].scaleLocation.y;
			commRam.txBuffer[3] = xi0 & 0x7f;
			xi0 >>= 7;
			commRam.txBuffer[4] = xi0 & 0x7f;

			commRam.txCount = 5;

			#ifdef UART
			for (currentDataIndex = 0; currentDataIndex < commRam.txCount; currentDataIndex++)
			{
				while(!UART_TRMT)
				{}
				UART_TXREG = commRam.txBuffer[currentDataIndex];
			}
			while(!UART_TRMT)
			{}
			#endif

			#ifdef I2C
				#ifdef TOUCHPADDEMONSTRATOR
					if(!SELECT_PIN)
					{
						writeI2C();
					}
				#else
					writeI2C();
				#endif
			#endif // ifdef I2C

			#ifdef USB
			sendUSBChar(commRam.txBuffer, commRam.txCount);
			#endif // ifdef USB

	#ifdef DISPLAY
			//lcdPutPoint(touchIDSet.tData[whichReport].scaleLocation.x>>3,touchIDSet.tData[whichReport].scaleLocation.y>>4,COLOR_INVERT);
			//lcdSetGUIState(UI_DRAW);
			addTouch(whichReport, touchIDSet.tData[whichReport].scaleLocation.x>>3, touchIDSet.tData[whichReport].scaleLocation.y>>4, touchIDSet.tData[whichReport].touchState, TDM_FULL);
	#endif
		}
	}
	else
	{
		#ifdef DISPLAY
		addTouch(whichReport, touchIDSet.tData[whichReport].scaleLocation.x>>3, touchIDSet.tData[whichReport].scaleLocation.y>>4, touchIDSet.tData[whichReport].touchState, TDM_MAPONLY);
		#endif
	}
}

/******************************************************************************
* Function:        	void sendCommGestureReport(void)
*
* PreCondition:    	None
*
* Input:           	scratchRam.p0 is the array number for the report
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	sends the touch report out the UART
*
* Note:            	Format
*					Packet	Bit	7	6	5	4	3	2	1	0
*					0			1	t3	t2	t1	t0	1	0	0
*					1			0	g6	g5	g4	g3	g2	g1	g0
*
*					Where:
*					t3:t0 	- touch ID number 0 based
*					g6:g4	- gesture type
*					g3:g0	- gesture information
*
*****************************************************************************/
void sendCommGestureReport(unsigned char whichID)
{
	#ifdef UART
    unsigned char currentDataIndex = 0;
	#endif
    unsigned int xi0 = 0;
	//#ifdef USB
	//char data[5];
	//#endif

	// pertinent coordinates are in touchIDSet.tData[whichReport].touchLoc[0].fineLocation.x;

#ifdef ENABLE_DEBUG
	if (userRam.flag1&CONTROLLERDIAGNOSTICS)
	{
		// Diagnostic Output - we want the following information
		// Touch ID
		// Touch State
		// X Location (2 bytes)
		// Y Locaiton (2 bytes)
        debugRam.ID = DIAGGESTUREREPORT;
        debugRam.size = 12;
        debugRam.buffer[0] = whichID;
        debugRam.buffer[1] = touchIDSet.tData[whichID].touchState;
        debugRam.buffer[2] = (touchIDSet.tData[whichID].touchLoc[0].fineLocation.x>>8)&0xff;
        debugRam.buffer[3] = touchIDSet.tData[whichID].touchLoc[0].fineLocation.x&0xff;
        debugRam.buffer[4] = (touchIDSet.tData[whichID].touchLoc[0].fineLocation.y>>8)&0xff;
        debugRam.buffer[5] = touchIDSet.tData[whichID].touchLoc[0].fineLocation.y&0xff;
		debugRam.buffer[6] = touchIDSet.tData[whichID].touchLoc[0].roughLocation.x;
		debugRam.buffer[7] = touchIDSet.tData[whichID].touchLoc[0].roughLocation.y;
		debugRam.buffer[8] = gesture[whichID].data;
		debugRam.buffer[9] = gesture[whichID].state;
        sendDebugBytes();
	}
#endif // ENABLE_DEBUG

	commRam.txBuffer[0] = 0x84 | whichID << 3;


	if(userRam.touchSuppressNum == 0 || whichID<userRam.touchSuppressNum) //supress touch if necessary
	{
		commRam.txBuffer[1] = gesture[whichID].data;

		commRam.txCount = 2;

		if (hwCfgRam.gesturePacketCfg&PACKETCFGENABLEMASK)
		{
			#ifdef UART
			for (currentDataIndex = 0; currentDataIndex < commRam.txCount; currentDataIndex++)
			{
				while(!UART_TRMT)
				{}
				UART_TXREG = commRam.txBuffer[currentDataIndex];
			}
			while(!UART_TRMT)
			{}
			#endif

			#ifdef I2C
				#ifdef TOUCHPADDEMONSTRATOR
					if(!SELECT_PIN)
					{
						writeI2C();
					}
				#else
					writeI2C();
				#endif
			#endif // ifdef I2C

			#ifdef USB
			sendUSBChar(commRam.txBuffer, commRam.txCount);
			#endif // ifdef USB
		}
	}
}

/******************************************************************************
* Function:        	void sendCommStatusReport(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	sends the status report
*
* Note:            	Format
*					Packet	Bit	7	6	5	4	3	2	1	0
*					0			1	0	0	0	0	0	1	0
*					1			0	e0	w0	s4	s3	s2	s1	s0
*
*					Where:
*					e0		- ERROR Status (Limited/No functionality)
*					w0		- WARNING Status (Funcionality may be degraded)
*							- If no ERROR or WARNING bit, INFO (Full functionality)
*					s4:s0	- status information
*
*					STATUS VALUES:
*					0x00 - "Normal Operation (No Error)"
*
*****************************************************************************/
void sendCommStatusReport(unsigned char statusValue)
{
	#ifdef UART
    unsigned char currentDataIndex = 0;
	#endif

	commRam.txBuffer[0] = 0x82;

	commRam.txBuffer[1] = statusValue&0x7f;

	commRam.txCount = 2;

	if (hwCfgRam.statusPacketCfg&PACKETCFGENABLEMASK)
	{
		#ifdef UART
		for (currentDataIndex = 0; currentDataIndex < commRam.txCount; currentDataIndex++)
		{
			while(!UART_TRMT)
			{}
			UART_TXREG = commRam.txBuffer[currentDataIndex];
		}
		while(!UART_TRMT)
		{}
		#endif

		#ifdef I2C
			#ifdef TOUCHPADDEMONSTRATOR
				if(!SELECT_PIN)
				{
					writeI2C();
				}
			#else
				writeI2C();
			#endif
		#endif // ifdef I2C

		#ifdef USB
		sendUSBChar(commRam.txBuffer, commRam.txCount);
		#endif // ifdef USB
	}
}

/******************************************************************************
* Function:        	void sendDebugBytes(void)
*
* PreCondition:    	None
*
* Input:           	None - utilizes "debugRam" data structure.
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Sends the diagnostic header, debug ID and data
*
* Note:            	The format is:
*					<0xAA> <0x55> <size + 1> <ID> <bytesToSend>
*
*****************************************************************************/
void sendDebugBytes(void)
{
#ifdef ENABLE_DEBUG
    unsigned char i;
	unsigned char selectedID = 0;

	if((debugRam.ID < 0x40) || (debugRam.ID > 0x60)) // USERDEBUG is always transmitted (0x40-0x60)
	{
		for (i = 0; i < DEBUG_MASK_SIZE && selectedID == 0 && debugRam.debugMask[i] != 0xff; i++)
		{
			if (debugRam.ID == debugRam.debugMask[i])
			{
				selectedID = 1;
			}
		}

		if (!selectedID)
		{
			return;
		}
	}

	if (hwCfgRam.diagPacketCfg&PACKETCFGENABLEMASK)
	{
		commRam.txBuffer[0] = 0xAA;
		commRam.txBuffer[1] = 0x55;
		if (debugRam.size < 60)
		{
			commRam.txBuffer[2] = debugRam.size+1;
			commRam.txBuffer[3] = debugRam.ID;
			//commRam.txCount = 4;
			for (i = 0; i < debugRam.size; i++)
		    {
				commRam.txBuffer[i+4] = debugRam.buffer[i];
		    	//commRam.txCount ++;
		    }
			commRam.txCount = 4 + debugRam.size;
		}
		else
		{ // diagnostic packet too large
			commRam.txBuffer[2] = 3;
			commRam.txBuffer[3] = DIAGNOSTICERROR;
			commRam.txBuffer[4] = debugRam.ID;
			commRam.txBuffer[5] = debugRam.size;
			commRam.txCount = 6;
		}

		#ifdef UART
		for (i = 0; i < commRam.txCount; i++)
		{
			while(!UART_TRMT)
			{}
			UART_TXREG = commRam.txBuffer[i];
		}
		while(!UART_TRMT)
		{}
		#endif

		#ifdef I2C
		//commRam.commFlag &= NOTUSEI2CDELAY;
			#ifdef TOUCHPADDEMONSTRATOR
				if(!SELECT_PIN)
				{
					writeI2C();
				}
			#else
				writeI2C();
			#endif
		#endif // ifdef I2C

		#ifdef USB
		sendUSBChar(commRam.txBuffer, commRam.txCount);
		#endif // ifdef USB
	}

#endif //ifdef ENABLE_DEBUG
}

/******************************************************************************
* Function:        	void sendLongData(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Sends the rawADC 512 unsigned ints to the host
*					64 bytes at a time
*
* Note:
*
*****************************************************************************/
#ifdef LONGADCSCAN
void sendLongData(unsigned char diagnosticID)
{
	#ifdef ENABLE_DEBUG
	unsigned short x;

	x = 0;
	while (x < 512)
	{
		debugRam.ID = diagnosticID;
		debugRam.buffer[0] = x & 0xff;
		debugRam.buffer[1] = (x >> 8) & 0xff;
		debugRam.size = 2;
		while (debugRam.size < 58 && x < 512)
		{
			debugRam.buffer[debugRam.size] = longADC[x] & 0xff;
			debugRam.size++;
			debugRam.buffer[debugRam.size] = longADC[x] >> 8;
			debugRam.size++;
			x++;
		}
		if (x == 512)
		{ // append the timestamp onto the end
			debugRam.buffer[debugRam.size] = rawRam.rawADC[0] & 0xff;
			debugRam.size++;
			debugRam.buffer[debugRam.size] = (rawRam.rawADC[0]>>8) & 0xff;
		}
		sendDebugBytes();

		delay1ms(2);
		commReceive(); // This can take a long time, be sure to respond to commands
	}
	#endif
}
#endif //ifdef LONGADCSCAN
