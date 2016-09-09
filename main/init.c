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
* name                  11/16/11    ...
******************************************************************************/
/*
	TSCG				11/16/11


*/

#include "main.h"

//
// Globals
//
extern USERRAM userRam;
extern HWCFGRAM hwCfgRam;
extern COMMRAM commRam;
extern VARRAM varRam;
extern DEBUGRAM debugRam;

extern unsigned char selfScanFineTune[MAXRX];
extern unsigned char mutScanFineTune[MAXRX];

/******************************************************************************
* Function:        	void init(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Initializes the chip
*
* Note:
*
*
*****************************************************************************/
void init(void)
{
	unsigned char temprxPinMap[MAXRX] = RXPINMAP;
	unsigned char temptxPinMap[MAXTX] = TXPINMAP;
	unsigned char tempSelfScanFineTune[MAXRX] = SELFSCANFINETUNE;
	unsigned char tempMutScanFineTune[MAXRX] = MUTSCANFINETUNE;
	unsigned char x;

#ifdef ENABLE_DEBUG
	unsigned char maskInit[] = DEBUGMASK;

	for (x = 0; x < DEBUG_MASK_SIZE; x++)
	{
		debugRam.debugMask[x] = maskInit[x];//0xff;
	}
#endif // ENABLE_DEBUG

	//
	// The first thing we do is setup our pin mappings to the
	// RX and TX channels.  Note this could be hard-coded but tempMap
	// allows us to use the #defines in main.h to pre-populate
	// this data
	//

	if(readEEPROMToRAM(EE_ADDRESS))
	{
		for (x=0;x<MAXRX;x++)
		{
			userRam.rxPinMap[x] = temprxPinMap[x];
			selfScanFineTune[x] = tempSelfScanFineTune[x];
			mutScanFineTune[x] = tempMutScanFineTune[x];
		}
		for (x=0;x<MAXTX;x++)
		{
			userRam.txPinMap[x] = temptxPinMap[x];
		}

		hardwarePreInit();

		//
		// init any user RAM
		//
		userRam.flag1 = 0;
		userRam.numberOfRXChannels = NUMBEROFRXCHANNELS;
		userRam.numberOfTXChannels = NUMBEROFTXCHANNELS;
		userRam.customFlag = CUSTOMFLAG;
		userRam.xmul = XMUL;
		userRam.ymul = YMUL;
		userRam.rxDiagChannel = RXDIAGCHANNEL;
		userRam.txDiagChannel = TXDIAGCHANNEL;

		userRam.stuckThreshold = STUCKTHRESHOLD;
		userRam.stuckTimeout = STUCKTIMEOUT;

		userRam.selfScanTime = SELFSCANTIME;
		userRam.selfTouchThres = SELFTOUCHTHRES;
		userRam.selfSampleFreq = SELFSAMPLEFREQ;
		userRam.stutterMult = STUTTERMULT;

		userRam.mutScanTime = MUTSCANTIME;
		userRam.mutTouchThres = MUTTOUCHTHRES;
		userRam.mutSampleFreq = MUTSAMPLEFREQ;

		userRam.baseUpdateTime = BASEUPDATETIME;

		userRam.flipState = FLIPSTATE;
		userRam.numOfAvg = NUMOFAVG;
		userRam.minCuspDelta = MINCUSPDELTA;
		userRam.weightThreshold = WEIGHTTHRESHOLD;
		userRam.minTouchDistance = MINTOUCHDISTANCE;
		userRam.penDownTimer = PENDOWNTIMER;
		userRam.penUpTimer = PENUPTIMER;
		userRam.touchSuppressNum = TOUCHSUPPRESSNUM;
		userRam.largeActThres = LARGEACTIVATIONTHRESH;
		userRam.cpTimeOut = CPTIMEOUT;


		userRam.selfNoiseThresh = SELFNOISETHRESH;
		userRam.mutNoiseThresh = MUTNOISETHRESH;
		userRam.frequencyChanges = FREQUENCYCHANGES;
		userRam.sampleSize = SAMPLESIZE;
		userRam.selfNoiseScanTime = SELFNOISESCANTIME;
		userRam.mutNoiseScanTime = MUTNOISESCANTIME;
		userRam.filterCoeff = FILTERCOEFF;

		userRam.swipeLengthX = SWIPELENGTHX;
		userRam.swipeLengthY = SWIPELENGTHY;
		userRam.holdSwipeBoundary = HOLDSWIPEBOUNDARY;
		userRam.swipeTime = SWIPETIME;
		userRam.swipeHoldThresh = SWIPEHOLDTHRESH;
		userRam.tapThresh = TAPTHRESH;
		userRam.tapTime = TAPTIME;
		userRam.minSwipeVelocity = MINSWIPEVELOCITY;
		userRam.edgeKeepoutDistance = EDGEKEEPOUTDISTANCE;
		userRam.maxClickTime = MAXCLICKTIME;

		// init hardware Configuration
		hwCfgRam.sleepTimeout = SLEEPTIMEOUT;
		hwCfgRam.sleepConfig = SLEEPCONFIG;
		hwCfgRam.wdtTimeout = WDTTIMEOUT;
		hwCfgRam.diagPacketCfg = DIAGPACKETCFG;
		hwCfgRam.touchPacketCfg = TOUCHPACKETCFG;
		hwCfgRam.commandPacketCfg = COMMANDPACKETCFG;
		hwCfgRam.gesturePacketCfg = GESTUREPACKETCFG;
		hwCfgRam.statusPacketCfg = STATUSPACKETCFG;
	}

	//
	// init any varibles
	//
	varRam.baseUpdateCount = 0;
	varRam.baseUpdateState = 0;
	varRam.flag = 0;

	initPC();

	//
	// Setup the UART
	//
	#ifdef UART
	initUART();
	#endif
	//
	// Setup I2C
	//
	#ifdef I2C
	initI2C();
	#endif
	//
	// Setup USB
	//
	#ifdef USB
	initUSB();
	#endif
	//
	// enable interrupts
	//
	hardwarePostInit();
	//
	// setup the projected capacitive
	//

	initGestures();
	decodeInit();
}

