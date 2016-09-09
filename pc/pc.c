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
* name                  11/21/11    ...
******************************************************************************/
/*
	TSCG				11/21/11


*/
#include "main\main.h"

//
// Globals
//
extern USERRAM userRam;
extern COMMRAM commRam;
extern RAWRAM rawRam;
extern VARRAM varRam;
extern BASERAM baseRam;
extern TOUCHSET touchSet;
extern TOUCHIDSET touchIDSet;
#ifdef ENABLE_DEBUG
extern DEBUGRAM debugRam;
#endif

#ifdef LONGADCSCAN
extern unsigned short longADC[512];
#endif
extern unsigned short *basePtr;
extern unsigned char baselineUpdateFlag;

#ifndef __18CXX
extern unsigned int TRIS_LUT[];
extern unsigned int TRISPORT_LUT[];
extern unsigned int LAT_LUT[];
extern unsigned int LATPORT_LUT[];
#endif

extern unsigned short baselineCounter;
extern unsigned short sleepCount;
extern unsigned char selfScanFineTune[MAXRX];
extern unsigned char mutScanFineTune[MAXRX];

extern void (*selfScan)(unsigned char);
extern void (*mutualScan)(unsigned char);

// "Local" variables
unsigned short firstX[MAX_TOUCHES], firstY[MAX_TOUCHES];


/******************************************************************************
* Function:        	void initPC(void)
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
void initPC(void)
{
	//
	// initialize the hardware
	//
	initPCHardware();
	//
	// Set the ports
	//
	groundAll();

	//
	//initialize noise structures
	//
	initNoise();

	//
	//setup stuttering masks
	//
	stutterMaskSetup();

	//
	// Make sure nothing is shorted
	//
	//checkIO(1);

	//
	// get the initial baselines
	//
	varRam.flag |= INITIALIZEBASE;

	getInitialSelfBase();
	getInitialSelfBase();
	getInitialMutualBase();
	getInitialMutualBase();

	varRam.flag &= NOTINITIALIZEBASE;

	sleepCount = 0;
}

/*************************************************************************//**
* Function:        	void stutterMaskSetup(void)
*
* \pre				userRam.stutterMult, userRam.txPinMap[], userRam.numberOfTXChannels all valid
*
* \result          	varRam.gtempLATX[] (where X=A,B,C,etc) setup for stuttering
*
* \post		    	None
*
* \brief        	Calculates the TX masks that are used for 'stuttering' the
*					pulses during a self measurement. Only needs to be done upon
*					initialization, and anytime the stutter multipler (userRam.stutterMult) is
*					modified.
*
* \note
*
*****************************************************************************/
void stutterMaskSetup(void)
{
	unsigned char wave;
	unsigned char y;
	unsigned char tempVar;

	for(wave=0;wave<userRam.stutterMult;wave++)
	{
		setup_self_clear_lat_stutter();
		for (y=wave;y<=userRam.numberOfTXChannels;y+=userRam.stutterMult)
		{
			if(y<userRam.numberOfTXChannels)
			{
				//set latch bit tempVar for current wave
				tempVar = userRam.txPinMap[y];
				setup_self_set_lat_bit_stutter();
			}
		}
	}
}

/******************************************************************************
* Function:        	void scanChannels(unsigned char channel)
*
* PreCondition:    	None
*
* Input:           	CTMU channel to scan
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Scans the CTMU channel
*
* Note:
*
*****************************************************************************/
unsigned short scanChannels(unsigned char start, unsigned char stop, unsigned char scanMode)
{
	unsigned char x, y, scanCount;
	PCAPULONG xl0;

	preInitSelf();					// run any special initialization for the processor

	if(userRam.selfSampleFreq == 0)
	{
		userRam.selfSampleFreq = NOISE_INC_VALUE;	
	}

	for (x=start;x<stop;x++)
	{
		//
		// Setup the varibles for the self measuement
		//
		setupSelf(x);

		if(scanMode == NOISE_SCAN)
		{
			scanCount = userRam.selfNoiseScanTime;
		}
		else
		{
			scanCount = userRam.selfScanTime;
		}

		scanCount += selfScanFineTune[x];

		y=0;
		for(;;)
		{
			if(y>=scanCount)
			{
				break;
			}
			//run sampling delay
			setup_delay_tmr();
			PC_DELAY_PR = userRam.selfSampleFreq;
			DELAY_TIMER_IF = 0;
			start_delay_timer();
			while (!DELAY_TIMER_IF){}
			stop_delay_tmr();

			varRam.flag &= NOTINTERRUPTOCCURRED; // clear the interrupt flag
			selfCVDScan(y);
			if(!(varRam.flag&INTERRUPTOCCURRED))
			{
				varRam.guint3 += rawRam.rawADC[y];
				y++;
			}
		}

		//
		// run any digital filters
		//
		if(scanMode != NOISE_SCAN)
		{
			if(userRam.filterCoeff > 0)
			{
				maFilter(scanCount);
			}
		}

		if ((userRam.customFlag&INVERTSELF) == INVERTSELF)
		{
			varRam.guint3 = 65535 - varRam.guint3; // we want our numbers to increase when touched
		}
		//
		// average the last and current together
		//
		xl0 = (unsigned short) varRam.guint3;
		if ((varRam.flag&INITIALIZEBASE) == 0x00)
		{
			xl0 += (PCAPULONG) rawRam.rawSelf[x];
			xl0 >>= 1;
		}

		if(scanMode != NOISE_SCAN)
		{
			rawRam.rawSelf[x] = (unsigned short) xl0;
		}
	}
	stop_delay_tmr();		// turn off the timers
	groundAll();

	if(scanMode == NOISE_SCAN)
	{
		return varRam.guint3;
	}
	else
	{
		return 0;
	}
}

/******************************************************************************
* Function:        	void setupSelf(unsigned char x)
*
* PreCondition:    	None
*
* Input:           	x - the channel number
*
* Output:          	Sets the gloabal varibles for the specific channel to scan
*
* Side Effects:    	None
*
* Overview:        	Sets the gloabal varibles for the specific channel to scan
*
* Note:            	None
*
*****************************************************************************/
void setupSelf(unsigned char x)
{
	unsigned char newTemp;

	//
	// Get the TRIS settings for the channel
	//
	varRam.gtempTRIS = TRIS_LUT[userRam.rxPinMap[x]];
	varRam.guint3 = 0;		// will hold the SUM

	newTemp = userRam.rxPinMap[x];

	set_tris_port();
	set_ansel_port(varRam.gANSEL, newTemp);

	varRam.gtempTRISClear = ~(TRIS_LUT[newTemp]);

	varRam.gADCONCHS = newTemp;
	set_ansel_bit();

	*varRam.gANSEL = varRam.gANSELbit;
	ADC_CHS = varRam.gADCONCHS;
}


/******************************************************************************
* Function:        	void longSelfScan(void)
*
* PreCondition:    	rxDiagChannel has been set to the channel to scan
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Take 512 ADC samples
*
* Note:            	Much of the RAM used in this call is overlapped and
*					will destroy baseline values, you will need to scan the
*					hardware baseline after this function to restore
*
*****************************************************************************/
#ifdef LONGADCSCAN
void longSelfScan(void)
{
	unsigned int x;

	preInitSelf();					// run any special initialization for the processor

	if(userRam.selfSampleFreq == 0)
	{
		userRam.selfSampleFreq = NOISE_INC_VALUE;
	}

	setupSelf(userRam.rxDiagChannel);

	for (x=0;x<512;x++)
	{
		setup_delay_tmr();			// setup the sample risetime timer
		PC_DELAY_PR = userRam.selfSampleFreq;
		DELAY_TIMER_IF = 0;
		start_delay_timer();
		while (!DELAY_TIMER_IF){}
		stop_delay_tmr();

		selfCVDScan(0);
		longADC[x] = rawRam.rawADC[0];
	}

	rawRam.rawADC[0] = 1;// fix this  TMR1;

	stop_delay_tmr();		// turn off the timers
	groundAll();						// put sensor and controller back to known state

}
#endif //ifdef LONGADCSCAN

/******************************************************************************
* Function:        	void mutualCapacitance(unsigned char channel)
*
* PreCondition:    	None
*
* Input:           	channel to scan
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Scans the CTMU channel
*
* Note:
*
*****************************************************************************/
void mutualCapacitance(unsigned char channel, unsigned char scanMode)
{
	unsigned char y;

	for (y=0;y<userRam.numberOfTXChannels;y++)
	{
		scanMutual(channel, y, scanMode);
	}
}

/******************************************************************************
* Function:        	void scanMutual(unsigned char channel, unsigned char TX)
*
* PreCondition:    	None
*
* Input:           	The channel to scan (X), x Y channel to toggle, scanCycles slow or fast scan
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Scans the actual X&Y node
*
* Note:
*
*****************************************************************************/
void scanMutual(unsigned char channel, unsigned char TX, unsigned char scanMode)
{
	unsigned char y, scanCount;

	preInitMutual();					// run any special initialization for the processor

	if(userRam.mutSampleFreq == 0)
	{
		userRam.mutSampleFreq = NOISE_INC_VALUE;
	}

	if(scanMode == NOISE_SCAN)
	{
		scanCount = userRam.mutNoiseScanTime;
	}
	else
	{
		scanCount = userRam.mutScanTime;
	}

	scanCount += mutScanFineTune[channel];

	//
	// Setup the varibles for the mutual scan
	//
	setupMutual(channel, TX);

	y=0;
	for(;;)
	{
	if(y>=scanCount)
		{
			break;
		}
		//setup sampling delay
		setup_delay_tmr();
		PC_DELAY_PR = userRam.mutSampleFreq;
		DELAY_TIMER_IF = 0;
		start_delay_timer();
		while (!DELAY_TIMER_IF){}
		stop_delay_tmr();

		varRam.flag &= NOTINTERRUPTOCCURRED; // clear the interrupt flag
		mutualCVDScan(y);
		if(!(varRam.flag&INTERRUPTOCCURRED))
		{
			if(y>0)
			{
				varRam.guint3 += rawRam.rawADC[y];
			}
			y++;
		}
	}

	//	run a moving average filter to clean up the signal
	if(scanMode != NOISE_SCAN)
	{
		if(userRam.filterCoeff != 0)
		{
			maFilter(scanCount);
		}
	}

	//
	// guint3 contains the SUM
	//
	if ((userRam.customFlag&INVERTMUT) == INVERTMUT)
	{
		varRam.guint3 = 65535 - varRam.guint3; // we want our numbers to increase when touched
	}

	rawRam.rawMut[TX] = (unsigned short) varRam.guint3;

	stop_delay_tmr();		// turn off the timers
	groundAll();			// put sensor and controller back to known state
}

/******************************************************************************
* Function:        	void setupMutual(unsigned char channel, unsigned char TX)
*
* PreCondition:    	None
*
* Input:           	channel - the channel number, TX the TX channel number
*
* Output:          	Sets the gloabal varibles for the specific node to scan
*
* Side Effects:    	None
*
* Overview:        	Sets the gloabal varibles for the specific node to scan
*
* Note:            	None
*
*****************************************************************************/
void setupMutual(unsigned char channel, unsigned char TX)
{
	unsigned char tempVar[2];
	unsigned char newTemp;

	//
	// get the TRIS value
	//
	varRam.gtempTRIS = TRIS_LUT[userRam.rxPinMap[channel]];

	varRam.guint3 = 0;
	//
	// Setup the specific TX pin
	//
	varRam.gtempLAT[0] = 0;
	varRam.gtempLAT[1] = 0;

	tempVar[0] = userRam.txPinMap[TX];
	varRam.gtempLAT[0] = LAT_LUT[tempVar[0]];
	if((userRam.customFlag&PULSE2TX) != 0x00)
	{
		if(TX == 0)
		{
			tempVar[1] = userRam.txPinMap[TX+1];
			varRam.gtempLAT[1] = LAT_LUT[tempVar[1]];
		}
		else
		{
			tempVar[1] = userRam.txPinMap[TX-1];
			varRam.gtempLAT[1] = LAT_LUT[tempVar[1]];
		}
	}
	else
	{
			tempVar[1] = tempVar[0];
			varRam.gtempLAT[1] = LAT_LUT[tempVar[1]];
	}

	set_lat_port();
	newTemp = userRam.rxPinMap[channel];
	set_tris_port();
	set_ansel_port(varRam.gANSEL, newTemp);
	varRam.gtempTRISClear = ~(TRIS_LUT[newTemp]);
	varRam.gADCONCHS = newTemp;
	set_ansel_bit();
	*varRam.gANSEL = varRam.gANSELbit;
	ADC_CHS = varRam.gADCONCHS;
}

/******************************************************************************
* Function:        	void longMutualScan(void)
*
* PreCondition:    	rxDiagChannel has been set to the channel to scan
*					and txDiagChannel has the TX channel
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Take 512 ADC samples
*
* Note:
*
*****************************************************************************/
#ifdef LONGADCSCAN
void longMutualScan(void)
{
	unsigned int x;

	preInitMutual();					// run any special initialization for the processor

	if(userRam.mutSampleFreq == 0)
	{
		userRam.mutSampleFreq = NOISE_INC_VALUE;
	}

	setupMutual(userRam.rxDiagChannel, userRam.txDiagChannel);

	for (x=0;x<512;x++)
	{
		//run sampling delay
		setup_delay_tmr();
		PC_DELAY_PR = userRam.mutSampleFreq;
		DELAY_TIMER_IF = 0;
		start_delay_timer();
		while (!DELAY_TIMER_IF){}
		stop_delay_tmr();

		mutualCVDScan(0);
		longADC[x] = rawRam.rawADC[0];
	}

	rawRam.rawADC[0] = 1;// fix this TMR1;

	stop_delay_tmr();		// turn off the timers
	groundAll();					// put sensor and controller back to known state

}
#endif //ifdef LONGADCSCAN

/******************************************************************************
* Function:        	void getInitialSelfBase(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	Fills selfBase with the self baseline values
*
* Overview:        	Scans the CSM channels collecting the self baseline
*
* Note:            	None
*
*****************************************************************************/
void getInitialSelfBase(void)
{
	unsigned char x;

	scanChannels(0, userRam.numberOfRXChannels, BASE_SCAN);
	for (x=0;x<userRam.numberOfRXChannels;x++)
	{
		baseRam.selfBase[x] = rawRam.rawSelf[x];
	}
}

/******************************************************************************
* Function:        	unsigned int getNormalizedSelf(unsigned char arrayPosition)
*
* PreCondition:    	None
*
* Input:           	The array position to normalize
*
* Output:          	return unsigned int raw - base
*
* Side Effects:    	None
*
* Overview:        	Normalizes the data from the selfBase
*
* Note:            	None
*
*****************************************************************************/
unsigned short getNormalizedSelf(unsigned char arrayPosition)
{
	unsigned short normalizedValue = 0;
	#ifdef PCAPPIC32MX
	PCAPULONG xl0;
	unsigned char pin;
	#endif


	if (rawRam.rawSelf[arrayPosition] > baseRam.selfBase[arrayPosition])
	{
		normalizedValue = rawRam.rawSelf[arrayPosition] - baseRam.selfBase[arrayPosition];




	#ifdef PCAPPIC32MX
			// 
			// RA0/1 multiplier (PIC32MX only)
			//
			pin = userRam.rxPinMap[arrayPosition];
			if(pin == RA0RX)
			{
				xl0 = (unsigned short) normalizedValue;
				xl0 *= 42;
				xl0 >>= 5;
				normalizedValue = (unsigned short) xl0;
			}
			else if(pin == RA1RX)
			{
				xl0 = (unsigned short) normalizedValue;
				xl0 *= 53;
				xl0 >>= 5;
				normalizedValue = (unsigned short) xl0;
			}
	#endif



	}
	return normalizedValue;
}

/******************************************************************************
* Function:        	void getInitialMutualBase(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	Fills mutBase with the mutual baseline values
*
* Overview:        	Scans the RX channels collecting the mutual baseline
*
* Note:            	None
*
*****************************************************************************/
void getInitialMutualBase(void)
{
	unsigned char x, y;
	for (x=0;x<userRam.numberOfRXChannels;x++)
	{
		mutualCapacitance(x, BASE_SCAN);	// result in rawRam.rawMut
		for (y=0;y<userRam.numberOfTXChannels;y++)
		{
			baseRam.mutBase[x][y] = rawRam.rawMut[y];
		}
	}
}

/******************************************************************************
* Function:        	unsigned int getNormalizedMutual(unsigned char x, unsigned char y)
*
* PreCondition:    	None
*
* Input:           	X and Y array location, fast or slow scan
*
* Output:          	Normalized mutual result
*
* Side Effects:    	None
*
* Overview:        	Normalizes the data from the mutfBase
*
* Note:            	None
*
*****************************************************************************/
unsigned short getNormalizedMutual(unsigned char x, unsigned char y)
{
	unsigned short normalizedValue = 0;

	if (rawRam.rawMut[y] < baseRam.mutBase[x][y])
	{
		normalizedValue = baseRam.mutBase[x][y] - rawRam.rawMut[y];
	}
	return normalizedValue;
}

/******************************************************************************
* Function:        	void checkTouch(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	varRam.flag TOUCH bit set if touch, cleared if not
*
* Side Effects:    	None
*
* Overview:        	Does a self scan, compares each channel to selfTouchThres
*					to determine if a touch occured.  Sets or clears
*					varRam.flag TOUCH bit
*
* Note:            	None
*
*****************************************************************************/
void checkTouch(void)
{
	unsigned char x;
	unsigned short temp;

	//
	// start with no touch
	//
	varRam.flag &= NOTTOUCH;
	//
	// do a full self scan looking for values above selfTouchThres
	//
	scanChannels(0, userRam.numberOfRXChannels, NORMAL_SCAN);
	for (x=0;x<userRam.numberOfRXChannels;x++)
	{
		temp = getNormalizedSelf(x); 	// get the normalized value
		if (temp > (unsigned short)userRam.selfTouchThres)
		{
			varRam.flag |= TOUCH;
			sleepCount = 0;
			break;
		}
	}

	if((varRam.flag&TOUCH)!=TOUCH)
	{
		baselineCounter = 0;
	}
}

/******************************************************************************
* Function:        	void updateBaseline(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	Stores the new baseline if available
*
* Overview:        	Updates the baseline based on conditions
*
* Note:            	The baseline(s) are updated in the following manner
*					1) If a touch reset the baseUpdateCount and return
*					2) If no touch start the baseUpdateCount(er) and wait
*					   baseUpdateTime counts, confirming no touch, and
*					   capture a temporary baseline
*					3) Once a temporary baseline is acquired clear the
*					   baseUpdateCount and wait baseUpdateTime.  If no touch
*					   has occured update the baseline
*					4) Keep repeating the process
*
*					updateBaseline is driven by a state machine controlled
*					by varRam.baseUpdateState.  The sates are:
*					State	Description
*					0		store all TX for RX[n-2](if not first time through), time delay before sampling self base
*					1		sample self measurement and store in tempSelfBase
*					2		store tempSelfBase to selfBase and sample all TX for RX[0]
*					3		store all TX for RX[0], sample all TX for RX[1]
*					4		store all TX for RX[1], sample all TX for RX[2]
*					5		store all TX for RX[2], sample all TX for RX[3]
*					...
*					...
*					n		store all TX for RX[n-3], sample all TX for RX[n-2]
*					NOTE: n = userRam.numberOfRXChannels + 3
*
*					*** reset and start over
*
*****************************************************************************/
void updateBaseline(void)
{
	unsigned char x, channel;
	//
	// check for update baseline command
	//
	if ((varRam.flag&SOFTWAREBASE) == SOFTWAREBASE)
	{
		//
		// When issuing a software baseline in we need to wait
		// until the communications has completed before running
		//
		varRam.flag &= NOTSOFTWAREBASE; // clear
		varRam.flag |= INITIALIZEBASE;
		getInitialSelfBase();
		getInitialSelfBase();
		getInitialMutualBase();
		getInitialMutualBase();
		varRam.flag &= NOTINITIALIZEBASE;
		//
		// reset the baseline state
		//
		varRam.baseUpdateState = 0;
		varRam.baseUpdateCount = 0;
	}


	if((varRam.flag&TOUCH)==TOUCH)									//do not baseline if there is a touch
	{
		varRam.baseUpdateCount = 0;									//reset counter
		varRam.baseUpdateState = 0;									//reset state
		return;
	}

	if(varRam.baseUpdateCount >= userRam.baseUpdateTime)			//ready to take baseline
	{
		if(baselineUpdateFlag == 1)									//store self measurement from last time
		{
			for(x=0;x<userRam.numberOfRXChannels;x++)
				{
				baseRam.selfBase[x] = baseRam.tempselfBase[x];
					}
			baselineUpdateFlag = 0;									//clear update flag
				}
		else if(baselineUpdateFlag == 2)							//store mutual measurement from last time
		{
			for(x=0;x<userRam.numberOfTXChannels;x++)
				{
				*(basePtr+x) = baseRam.tempMutBase[x];
				}
			baselineUpdateFlag = 0;									//clear update flag
			}

		switch(varRam.baseUpdateState)
					{
			case 0:		//delay before self sample
				break;

			case 1:		//sample RX bars with self measurement
				scanChannels(0, userRam.numberOfRXChannels, BASE_SCAN);
				for(x=0;x<userRam.numberOfRXChannels;x++)
			{
					baseRam.tempselfBase[x] = rawRam.rawSelf[x];
			}
				baselineUpdateFlag = 1;								//set flag to store self measurements at next iteration
			break;

			default:	//sample all nodes using mutual measurement
				channel = varRam.baseUpdateState - 2;				//get the RX channel to sample
				basePtr = &baseRam.mutBase[channel][0];

				mutualCapacitance(channel, BASE_SCAN);				//sample RX channel with fast mutual measurement
				for(x=0;x<userRam.numberOfTXChannels;x++)
					{
						baseRam.tempMutBase[x] = rawRam.rawMut[x];
					}
				baselineUpdateFlag = 2;								//set flag to store mutual measurements at next iteration
				if(channel >= userRam.numberOfRXChannels-1)			//check if this is last channel
				{
					varRam.baseUpdateState = 0;						//reset state
					varRam.baseUpdateCount = 0;						//reset counter
					return;
			}
			break;
		}
		varRam.baseUpdateCount = 0;									//reset counter for baseline routine
		varRam.baseUpdateState++;									//increment baseline state machine
	}
	else
	{
		varRam.baseUpdateCount++;									//increment counter
	}
}


/******************************************************************************
* Function:        	void checkStuckTouches(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	none
*
* Side Effects:    	None
*
* Overview:        	Checks to see if a touch has not moved for 6+ seconds,
*					if it has not, forces a baseline to remove the "stuck"
*					activation.
*
* Note:            	None
*
*****************************************************************************/
void checkStuckTouches(void)
{
	unsigned char x;
	unsigned short tempX, tempY;
	for(x=0;x<MAX_TOUCHES;x++)
	{
		if(touchIDSet.tData[x].touchState == TS_TOUCH_DOWN)
		{
			firstX[x] = touchIDSet.tData[x].touchLoc[0].fineLocation.x;
			firstY[x] = touchIDSet.tData[x].touchLoc[0].fineLocation.y;
		}
		else if(touchIDSet.tData[x].touchState == TS_TOUCH_STREAM)
		{
			tempX = magnitude(touchIDSet.tData[x].touchLoc[0].fineLocation.x, firstX[x]);
			tempY = magnitude(touchIDSet.tData[x].touchLoc[0].fineLocation.y, firstY[x]);

			if((tempX > userRam.stuckThreshold) || (tempY > userRam.stuckThreshold))
			{
				baselineCounter = 0;
				break;
			}
		}
	}
	if(baselineCounter > userRam.stuckTimeout)		//if touch for over 6s, re-baseline
	{
		varRam.flag |= INITIALIZEBASE;
		getInitialSelfBase();
		getInitialMutualBase();
		varRam.flag &= NOTINITIALIZEBASE;
	}
}
