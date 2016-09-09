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
* name                  11/15/11    ...
******************************************************************************/
/*
	TSCG				11/15/11


*/

/*! \file main.c
	\brief Main loop and interrupt handler.

	This file contains the main loop for the firmware, as well as the interrupt
	handler.  This file is also where all of the variables are declared.
*/

#include "main\main.h"

#ifdef DISPLAY
	#include "display\lcddisplay.h"
#endif
//
// Globals
//
//#pragma udata
COMMRAM commRam;
RAWRAM rawRam;
VARRAM varRam;
BASERAM baseRam;
NOISERAM mutNoise;
NOISERAM selfNoise;
USERRAM userRam;
HWCFGRAM hwCfgRam;
HWSTATUSRAM hwStatusRam;
GESTURE gesture[MAX_TOUCHES];

volatile unsigned short globalCounter = 0;

unsigned short baselineCounter = 0;
unsigned short sleepCount = 0;
unsigned char sleepFlag = 0;

unsigned char selfScanFineTune[MAXRX];
unsigned char mutScanFineTune[MAXRX];

unsigned short *basePtr;
unsigned char baselineUpdateFlag;
#ifdef LONGADCSCAN
unsigned short longADC[512];
#endif
TOUCHSET touchSet;
TOUCHIDSET touchIDSet;
COLCACHE colCache;
#ifdef ENABLE_DEBUG
DEBUGRAM debugRam;
#endif // ENABLE_DEBUG

void (*selfScan)(unsigned char);
void (*mutualScan)(unsigned char);

//#pragma code

/*************************************************************************//**
* Function:        	main(void)
*
* \pre		    	None
*
* \result          	never exited, infinite loop to scan sensor
*
* \post		    	None
*
* \brief        	drives the pcap routines
*
* \note           	None
*
*****************************************************************************/
#ifndef PCAPPIC18F
int main(void)
#else
void main(void)
#endif
{
	unsigned char x;

#ifdef TOUCHPADDEMONSTRATOR
	SELECT_PIN_LAT = 0;
	SELECT_PIN_TRIS = 1;
#endif

	init();
#ifdef DISPLAY
	initLCDSPI();
	lcdInit(INIT_FULL);
	//lcdTest();
	lcdSetGUIState(UI_STARTUP);
	lcdFrame();
	delay1ms(1000);
	getInitialSelfBase();
	getInitialSelfBase();
	getInitialMutualBase();
	getInitialMutualBase();
	delay1ms(2000);
	lcdSetGUIState(UI_LOGO);
	lcdFrame();
#endif
	sleepFlag = 0;
	for(;;)
	{
		if ((userRam.flag1&DISABLE) != (unsigned char)DISABLE)
		{
#ifdef ENABLE_DEBUG
			if (!debugFeatures())
#endif // ENABLE_DEBUG
			{
				//
				// normal touch


				if((sleepFlag && hwCfgRam.sleepConfig&SLEEP_MASK) && (hwCfgRam.sleepTimeout > 0))
				{
					goToSleep();
				}
				else
				{
					checkTouch();
				}
				updateBaseline();
                findTouches();
				associateTouches();
				checkGestureState();
				checkStuckTouches(); // re-baseline out a stuck touch
				handleTouches();

#ifdef DISPLAY
				for(x=0;x<MAX_TOUCHES;x++)
				{
					//lcdPutHexValue(gesturePacket[x],0,8+(x<<3));ST
					switch (gesture[x].data)
					{
						case GESTURE_RIGHT_SWIPE:
							lcdSetOverlay(OVERLAY_SWIPE_LEFT,OVERLAY_TIMEOUT);
							break;
						case GESTURE_RIGHT_SWIPE_HOLD:
							lcdSetOverlay(OVERLAY_SWIPE_LEFT_HOLD,OVERLAY_TIMEOUT);
							break;
						case GESTURE_LEFT_SWIPE:
							lcdSetOverlay(OVERLAY_SWIPE_RIGHT,OVERLAY_TIMEOUT);
							break;
						case GESTURE_LEFT_SWIPE_HOLD:
							lcdSetOverlay(OVERLAY_SWIPE_RIGHT_HOLD,OVERLAY_TIMEOUT);
							break;
						case GESTURE_UP_SWIPE:
							lcdSetOverlay(OVERLAY_SWIPE_UP,OVERLAY_TIMEOUT);
							break;
						case GESTURE_UP_SWIPE_HOLD:
							lcdSetOverlay(OVERLAY_SWIPE_UP_HOLD,OVERLAY_TIMEOUT);
							break;
						case GESTURE_DOWN_SWIPE:
							lcdSetOverlay(OVERLAY_SWIPE_DOWN,OVERLAY_TIMEOUT);
							break;
						case GESTURE_DOWN_SWIPE_HOLD:
							lcdSetOverlay(OVERLAY_SWIPE_DOWN_HOLD,OVERLAY_TIMEOUT);
							break;
						case GESTURE_DOUBLETAP:
							lcdBufferClear(CLEAR_BLANK);
							lcdSetGUIState(UI_CLEARSCREEN);
							lcdSetOverlay(OVERLAY_DOUBLE_CLICK, OVERLAY_TIMEOUT);
							break;
						case GESTURE_SINGLETAP:
							lcdSetOverlay(OVERLAY_CLICK, OVERLAY_TIMEOUT);
							break;
						case GESTURE_HOLDTAP:
							lcdSetOverlay(OVERLAY_CLICK_HOLD, OVERLAY_TIMEOUT);
						break;
					}
				}
#endif // DISPLAY
			}
		}
		commReceive();

#ifdef DISPLAY
		lcdFrame();
#endif // DISPLAY
	}
	#ifndef PCAPPIC18F
	return 0;
	#endif
}

/*************************************************************************//**
* Function:        	unsigned char debugFeatures(void)
*
* \pre		    	None
*
* \result          	Performs all debugging diagnostics for the development tools.
*
* \post		    	None
*
* \brief        	performs debugging routines for development tools.
*
* \note           	None
*
*****************************************************************************/
#ifdef ENABLE_DEBUG
unsigned char debugFeatures(void)
{
	unsigned char  x;
	unsigned char  y;
	unsigned short value;
	unsigned char  inDebugMode = 1;

	// check for updated stutter value
	// Typically, the stutter groups are setup once during init - this will check
	// for a modified value and allow dynamic changes to the stutterMult.
	if(userRam.stutterMult != varRam.stutterMultCache)
	{
		stutterMaskSetup();
		varRam.stutterMultCache = userRam.stutterMult;
	}

	if ((userRam.flag1&SELFDIAGNOSTICS) == (unsigned char)SELFDIAGNOSTICS)
	{
		if ((userRam.flag1&(CONTROLLERDIAGNOSTICS|RAWADC|TIMEMINMAX)) == (unsigned char)0x00)
		{
			scanChannels(0, userRam.numberOfRXChannels, NORMAL_SCAN);

			debugRam.ID = SELFRAWDIAGNOSTICS;
			debugRam.size = userRam.numberOfRXChannels<<1;
			for (x=0;x<userRam.numberOfRXChannels;x++)
			{
				debugRam.buffer[(x<<1)] = rawRam.rawSelf[x]&0xff;
				debugRam.buffer[(x<<1)+1] = rawRam.rawSelf[x] >> 8;
			}
			sendDebugBytes();
		}
		else if ((userRam.flag1&(CONTROLLERDIAGNOSTICS|RAWADC|TIMEMINMAX)) == CONTROLLERDIAGNOSTICS)
		{
			checkTouch();
			updateBaseline();
			debugRam.ID = SELFCONTROLLERDIAGNOSTICS;
			debugRam.size = userRam.numberOfRXChannels<<1;
			for (x=0;x<userRam.numberOfRXChannels;x++)
			{
				value = getNormalizedSelf(x); 	// get the normalized value
				if (value <= (unsigned short)userRam.selfTouchThres)
				{
					value = 0;
				}
				debugRam.buffer[x<<1] = value&0xff;
				debugRam.buffer[(x<<1)+1] = value >> 8;
			}
			sendDebugBytes();
		}
		else if ((userRam.flag1&(RAWADC|TIMEMINMAX)) == RAWADC)// RAWADC
		{
			scanChannels(userRam.rxDiagChannel, userRam.rxDiagChannel+1, NORMAL_SCAN);

			y = 0;
			while (y < userRam.selfScanTime)
			{
				debugRam.ID = RAWSELFADCDIAGNOSTICS;
				debugRam.buffer[0] = y;
				debugRam.buffer[1] = userRam.selfScanTime;
				debugRam.size = 2;
				while (debugRam.size < 58 && y < userRam.selfScanTime)
				{
					debugRam.buffer[debugRam.size] = rawRam.rawADC[y] & 0xff;
					debugRam.size++;
					debugRam.buffer[debugRam.size] = rawRam.rawADC[y] >> 8;
					debugRam.size++;
					y++;
				}
				sendDebugBytes();
				commReceive();
			}
			delay1ms(2);
		}
		#ifdef LONGADCSCAN
		else
		{
			longSelfScan();
			sendLongData(LONGSELFSCANDIAGNOSTICS);
		}
		#endif
	}
	else if ((userRam.flag1&DIAGNOSTICMASK) == (unsigned char)MUTUALDIAGNOSTICS)
	{
		if ((userRam.flag1&(CONTROLLERDIAGNOSTICS|RAWADC|TIMEMINMAX)) == (unsigned char)0x00)
		{
			for (x=0;x<userRam.numberOfRXChannels;x++)
			{
				commReceive(); 	// mutual can have long scan cycles

			 	mutualCapacitance(x, NORMAL_SCAN);
				debugRam.ID = MUTUALRAWDIAGNOSTICS;
				debugRam.size = (userRam.numberOfTXChannels<<1)+1;
				debugRam.buffer[0] = x; // Current RX Channel;

				for (y = 0; y < userRam.numberOfTXChannels; y++)
				{
					debugRam.buffer[(y<<1)+1] = rawRam.rawMut[y]&0xff;
					debugRam.buffer[(y<<1)+2] = rawRam.rawMut[y]>>8;
				}
				sendDebugBytes();
			}
		}
		else if ((userRam.flag1&(CONTROLLERDIAGNOSTICS|RAWADC|TIMEMINMAX)) == CONTROLLERDIAGNOSTICS)
		{
			checkTouch();
			updateBaseline();
			for (x=0;x<userRam.numberOfRXChannels;x++)
			{
				commReceive();	// mutual can have long scan cycles
				mutualCapacitance(x, NORMAL_SCAN);
				debugRam.ID = MUTUALCONTROLLERDIAGNOSTICS;
				debugRam.size = (userRam.numberOfTXChannels<<1)+1;

				debugRam.buffer[0] = x; // current RX Channel

				for (y = 0; y < userRam.numberOfTXChannels; y++)
				{
					value = getNormalizedMutual(x, y);
					debugRam.buffer[(y<<1)+1] = value&0xff;
					debugRam.buffer[(y<<1)+2] = value>>8;
				}
				sendDebugBytes();
			}
		}
		else if ((userRam.flag1&(RAWADC|TIMEMINMAX)) == RAWADC) // RAWADC
		{
			scanMutual(userRam.rxDiagChannel, userRam.txDiagChannel, NORMAL_SCAN);
			y = 0;
			while (y < userRam.mutScanTime)
			{

				debugRam.ID = RAWMUTUALADCDIAGNOSTICS;
				debugRam.buffer[0] = y;
				debugRam.buffer[1] = userRam.mutScanTime;
				debugRam.size = 2;
				while (debugRam.size < 58 && y < userRam.mutScanTime)
				{
					debugRam.buffer[debugRam.size] = rawRam.rawADC[y] & 0xff;
					debugRam.size++;
					debugRam.buffer[debugRam.size] = rawRam.rawADC[y] >> 8;
					debugRam.size++;
					y++;
				}
				sendDebugBytes();
				commReceive();
				delay1ms(2);
				//delay100usTMR(20);	// 2ms
			}
		}
		#ifdef LONGADCSCAN
		else
		{
			longMutualScan();
			sendLongData(LONGMUTUALSCANDIAGNOSTICS);
		}
		#endif
	}
	else
	{
		inDebugMode = 0;
	}
	return inDebugMode;
}
#endif // ENABLE_DEBUG

