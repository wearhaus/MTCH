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

/*! \file main.h
	\brief Contains all headers and data structures.

	This file contains all headers and data structure definitions.  It is the
	only header file utilized in the entire projected capacitive project.
*/

#ifndef __MAIN_H
#define __MAIN_H
//
// defines
//


//#define DEVELOPMENT //!< This is not a "release" version
#define DEVKIT_HARDWARE //!< This is targeted at DEVKIT_HARDWARE
#define ENABLE_DEBUG //!< Enable debug messaging functionality

//#define TOUCHPADDEMONSTRATOR // This firmware is being compiled for the touch pad demonstrator

// define specific development board
//
//#define PCAPPIC18FDEVBMPK22V5		// K22 MPKV5 development board
//#define PCAPPIC32MXDEVMPXV2		// PIC32MX development baord with PIC32MX120F032D

//#define ENABLE_BOOTLOADER
// If the bootloader is enabled, be sure to include the linker script

//
// Communications, support is dependent on the micro and the hardware
//
//#define UART
#define I2C
//#define USB

//#define DISPLAY

//
// define LONGADCSCAN if you wish to use the longScan functions
// in the diagnostics.  This will take 1024 bytes
//
#ifdef ENABLE_DEBUG
	#define LONGADCSCAN
#endif // ENABLE_DEBUG

#ifdef TOUCHPADDEMONSTRATOR
	#define DISPLAY
#endif

//
// Projected Constants, pre-defines for dev kit sensors
// defined in sensor.h
//
#ifdef PCAPPIC24F
#include "hardware/pcap_hardware_pic24fj64GB106_dev.h"
#include "sensor/sensor_pic24fj64gb106_dev.h"
#endif
#ifdef PCAPPIC32MX
#include "hardware/pcap_hardware_pic32mx120f032d_dev.h"
#include "sensor/sensor_MTC6301.h"
#endif
#ifdef PCAPPIC18F
#include "hardware/pcap_hardware_pic18f46k22_dev.h"
#include "sensor/sensor_pic18f46k22_dev.h"
#endif
#ifdef PCAPPIC24FJ64GB004
#include "hardware/pcap_hardware_pic24fj64GB004_dev.h"
#include "sensor/sensor_pic24fj64gb004_dev.h"
#endif


//Application ID
#define APPIDHIGH 0x00
#define APPIDLOW  0x10

//FW revision number 
#define FWREVMAJOR 0x02
#define FWREVMINOR 0x05

//
// Define the number of touch points supprted, the more used the
// more RAM
//

#define MAX_TOUCHES 	10			// support up to 8 touches
#define TOUCH_HISTORY 	16			// support history up to 16 deep

//
// Scaling multiplier used in conjuntion with bitshifting in decode.c to
// scale final output 
//
#define XMUL 65536/NUMBEROFRXCHANNELS	
#define YMUL 65536/NUMBEROFTXCHANNELS	

//
// touch settings
//
#ifdef TOUCHPADDEMONSTRATOR
	#define TOUCHSUPPRESSNUM 		2
#else
	#define TOUCHSUPPRESSNUM 		0	// limits the amount of touches that are *communicated* (still decodes MAX_TOUCHES) 0=disabled
#endif
#define LARGEACTIVATIONTHRESH 	150

// Constants to define ColCache
#define COLCACHESIZE 5

//
// The receive buffer size
//
#define RCVBUFFERSIZE 16

#define SLEEPTIMEOUT 8000	//go to sleep after 8 seconds of inactivity

//scan types for filtering
#define NORMAL_SCAN	0  // full filtering
#define NOISE_SCAN	1  // no filtering, since we're looking for noise
#define BASE_SCAN 	2  // no touch = no significant noise coupled in, thus no filters

// comm constants
//
#define PARAMETERCOUNTERROR	0xFC	//!< Missing or Extra parameter for the given command
#define INVALIDPARAMETER	0xFD	//!< Invalid parameter for the given command
#define UNRECOGNIZEDCOMMAND 0xFF	//!< Response to an unrecognized command
#define COMMANDTIMEOUT		0xFE	//!< Response to a partial command
#define PARAMETEROUTOFRANGE 0x80	//!< Response if a command is sent with data that is out of range
#define DEFAULTSUCCESS 		0x00	//!< Response when the controller successfully completes a command

//
// Commands
//
#define ENABLECONTROLLER    0x00
#define DISABLECONTROLLER   0x01
#define SCANBASELINE        0x14	// re-scan the baselines
#define WRITERAM            0x15	// write user RAM
#define READRAM             0x16	// read user RAM
#define WRITEUSEREEPROM     0x17	// write userRam to EEPROM
#define SOFTWARESLEEP       0x18	// put the controller to sleep
#define ERASEEEPROM         0x19	// erase the EEPROM
#define CHECKIO             0x1A	// Check configured I/O for shorts
#define CFGCMD       	    0x83	// send CFG information ID Byte									// when the command set is modified
#define ENTERBOOTLOADERCMD	0xB0	// Enter the bootloader
#define GETDIAGMASKCMD      0xD0	// Retrieve the current Diagnostic Mask
#define SETDIAGMASKCMD      0xD1	// Set the current Diagnostic Mask

//
// ReadRam/WriteRam High offsets
//
#define RAM_USERRAMGENERAL	0x00
#define RAM_RXPINMAP		0x01
#define RAM_TXPINMAP		0x02
#define RAM_SELFPARAM		0x10
#define RAM_SELFTUNE		0x11
#define RAM_MUTPARAM		0x20
#define RAM_MUTTUNE			0x21
#define RAM_DECODETRACK		0x30
#define RAM_NOISE			0x40
#define RAM_GESTURES		0x50
#define RAM_HWCFG			0xF0
#define RAM_HWSTATUS		0xF1

#define EE_ADDRESS	0x9D007000

//
// PCAP Diagnostic Message IDs
//
#define SELFRAWDIAGNOSTICS              0x01
#define SELFCONTROLLERDIAGNOSTICS       0x05
#define MUTUALRAWDIAGNOSTICS            0x02
#define MUTUALCONTROLLERDIAGNOSTICS     0x06

#define RAWSELFADCDIAGNOSTICS           0x09
#define RAWMUTUALADCDIAGNOSTICS         0x0A
#define LONGSELFSCANDIAGNOSTICS         0x11
#define LONGMUTUALSCANDIAGNOSTICS       0x12

//
// Touch Diagnostic Decoding Message IDs
//
#define FINDTOUCHES             0x1A	//!< Diagnostic Flag for Debug output
#define FINDNEXTPEAK            0x1B	//!< Diagnostic Flag for Debug output
#define FINDFINELOCATION        0x1C	//!< Diagnostic Flag for Debug output

#define DIAGIMAGESTART          0x1F	//!< start of a new "image"
#define DIAGIMAGEEND            0x1E	//!< end of the "image"
#define DIAGSELFDATA            0x20	//!< Location (single byte - X) followed by value(s) - if more than one value, must be adjacent locations (along X)
#define DIAGMUTDATA             0x21	//!< Location (two bytes - X & Y) followed by value(s) - if more than one value, must be adjacent locations (along Y)
#define TOUCHDATAROUGH          0x22	//!< Rough Touch Location & ID
#define TOUCHDATAFINE           0x23	//!< Fine Touch Location * ID
#define DIAGNUDGE               0x24	//!< Diagnostics from the Nudge function
#define DIAGCOLCACHE            0x25	//!< Diagnostics for the Column Cache
#define DIAGTOUCHREPORT         0x26	//!< Diagnostics for touch reporting
#define DIAGASSOCIATETOUCH      0x27	//!< Diagnostics for Touch Association
#define DIAGGESTURE				0x40	//!< Diagnostics for gesture state
#define DIAGGESTUREREPORT		0x41	//!< Diagnostics for gesture report

#define DIAGUSER1 0x50

#define DIAGFINEX 0x30
#define DIAGFINEY 0x31

//filters for noise diagnostic messages
#define SELFNOISEDEV	0x32
#define SELFNOISEFREQ	0x33
#define MUTNOISEDEV 	0x34
#define MUTNOISEFREQ	0x35

#define DIAGNOSTICERROR 0xFE		// Invalid packet size received
#define DIAGNONE 		0xFF		// No diagnostic message

#define DEBUG_MASK_SIZE 10

// This mask generates ALL data necessary for Decode diagnostics
#define DEBUGMASK {DIAGNOSTICERROR, \
				DIAGTOUCHREPORT, \
				DIAGSELFDATA, \
				DIAGIMAGESTART, \
				DIAGMUTDATA, \
				DIAGNUDGE, \
				DIAGNONE, \
				DIAGNONE, \
				DIAGNONE, \
				DIAGNONE}


// Diagnostic Message Enables
#ifdef ENABLE_DEBUG
	#define DIAGNUDGE_ENABLE			//!< Enable DIAGNUDGE MESSAGES
	#define DIAGCOLCACHE_ENABLE			//!< Enable DIAGCOLCACHE MESSAGES
	#define DIAGASSOCIATETOUCH_ENABLE	//!< Enable DIAGAT MESSAGES
#endif

//
// Diagnostic SubIDs for Associate Touch
//
#define DIAGAT_INTRO					0x01	//!< basic intro info (number of touches, etc)
#define DIAGAT_TOUCHSETINFO				0x02	//!< Initial information upon entering Associate Touch
#define DIAGAT_TOUCHIDSETINFO			0x03	//!< Initial information upon entering Associate Touch
#define DIAGAT_NEWTOUCH					0x04	//!< info on a new touch allocation
#define DIAGAT_TOUCHWEIGHT              0x05	//!< Diagnostic info on touch weighting.
#define DIAGAT_TOUCHWEIGHT_NEWMATCH     0x06	//!< Diagnostics for a new touch match.
#define DIAGAT_TOUCHMATRIX              0x07	//!< Touch Matrix Dump
#define DIAGAT_TOUCHMATRIXLINE          0x08	//!< Single line of the touch matrix, when changed
#define DIAGAT_TOUCHLINK				0x09	//!< Touch Linked to an ID

// DIAGCOLCACHE subdiagnostic IDs
#define DIAGCOLCACHE_INIT           0x01
#define DIAGCOLCACHE_QUERY          0x02
#define DIAGCOLCACHE_QUERYVALUE     0x03

//
// userRam.flag1 constants
//
#define DIAGNOSTICMASK          0x03          // diagnostic mask
#define NODIAGNOSTICS           0xFC          // take out of diagnostic mode
#define SELFDIAGNOSTICS         0x01      // self diagnostics
#define MUTUALDIAGNOSTICS       0x02      // mutual diagnostics
#define CONTROLLERDIAGNOSTICS   0x04  // send diagnostics of controller calculated values
#define RAWADC                  0x08                  // send raw ADC values back for single channel
#define NOTRAWADC               0xF7
#define TIMEMINMAX              0x10              // send max-min and time stamp
#define NOTTIMEMINMAX           0xE0
#define DISABLE                 0x40              // used to disable the controller
#define NOTDISABLE              0xBF

//
// varRam.flag constants
//
#define TOUCH                   0x08         // set if touch detected, clear if not
#define NOTTOUCH                0xF7         // clears the TOUCH flag
#define LASTTOUCH				0x04
#define NOTLASTTOUCH			0xFB
#define SOFTWAREBASE            0x20          // Software baseline
#define NOTSOFTWAREBASE         0xDF
#define INITIALIZEBASE          0x01
#define NOTINITIALIZEBASE       0xfe
#define INTERRUPTOCCURRED       0x02
#define NOTINTERRUPTOCCURRED    0xfd

//
// userRam.customFlag
//
//#define USECVD                  0b00000010    // This configuration uses CVD, if clear use CTMU
//#define NOTUSECVD               0b11111101
#define USECHARGEPUMPDELAY      0b00000100    // Use a delay when using the charge pump
#define NOTUSECHARGEPUMPDELAY   0b11111011
#define INVERTSELF              0b00010000    // Invert Self
#define NOTINVERTSELF           0b11101111
#define INVERTMUT               0b00100000    // Invert Mutual
#define NOTINVERTMUT            0b11011111
#define USEMUTDIFF              0b10000000    //Use differential on mutual
#define NOTUSEMUTDIFF           0b01111111
#define PULSE2TX				0b00001000    //pulse 2 TX channels during mutual scan
#define NOTPULSE2TX             0b11110111
#define RUNNOISEROUTINES        0b01000000    //run noise routines on self and mutual
#define NOTRUNNOISEROUTINES     0b10111111

// Gesture Types

#define GESTURE_SINGLETAP			0x10
#define GESTURE_HOLDTAP				0x11
#define GESTURE_DOUBLETAP			0x20
#define GESTURE_RIGHT_SWIPE 	  	0x41
#define GESTURE_RIGHT_SWIPE_HOLD	0x42
#define GESTURE_LEFT_SWIPE 			0x61
#define GESTURE_LEFT_SWIPE_HOLD		0x62
#define GESTURE_UP_SWIPE 			0x31
#define GESTURE_UP_SWIPE_HOLD		0x32
#define GESTURE_DOWN_SWIPE 			0x51
#define GESTURE_DOWN_SWIPE_HOLD		0x52

//GESTURE DATA VALUES//
#define GV_NONE					0
#define GV_SINGLECLICK			1
#define GV_DOUBLECLICK         	2
#define GV_UP_SWIPE				3
#define GV_RIGHT_SWIPE			4
#define GV_DOWN_SWIPE			5
#define GV_LEFT_SWIPE			6

//TYPES//
#define GT_NONE				0
#define GT_UNDEFINED		1
#define GT_SWIPE			2
#define GT_HOLDCLICK		3
#define GT_DONE				4

#define magnitude(x, y) (x>=y)?(x-y):(y-x)
#define direction(x, y) (x>=y)?1:0

// STATUS Types
#define STATUS_ERROR 	0x40
#define STATUS_WARN		0x20
#define STATUS_INFO		0x00

// STATUS Responses
#define STATUS_OK 		STATUS_INFO | 0x00
#define STATUS_SHORT	STATUS_ERROR | 0x01


// Hardware configuration Types
#define	SLEEP_NONE 0
#define SLEEP_IDLE 1
#define SLEEP_SLEEP 2
#define SLEEP_MASK 0x03

#define WDT_PS1		0b00000
#define WDT_PS2		0b00001
#define WDT_PS4		0b00010
#define WDT_PS8		0b00011
#define WDT_PS16	0b00100
#define WDT_PS32	0b00101
#define WDT_PS64	0b00110
#define WDT_PS128	0b00111
#define WDT_PS256	0b01000
#define WDT_PS512	0b01001
#define WDT_PS1024	0b01010
#define WDT_PS2048	0b01011
#define WDT_PS4096	0b01100

//
// structures
//
typedef struct _USERRAM
{											//	Offset
	unsigned char  flag1;				//	0
	unsigned char  numberOfRXChannels;	//	1
	unsigned char  numberOfTXChannels;	//	2
	unsigned char  customFlag;			//	3
	unsigned short xmul;				//	4
	unsigned short ymul;				//	6
	// general
	unsigned char  rxDiagChannel;		//	8
	unsigned char  txDiagChannel;		//	9
	unsigned char  baseUpdateTime;		//	10
	unsigned char  stuckThreshold;		//	11
	unsigned short stuckTimeout;		//	12
	// self
	unsigned char  selfScanTime;		//	14
	unsigned char  selfTouchThres;		//	15
	unsigned char  selfSampleFreq;		//	18
	unsigned char  stutterMult;			//	19
	// mutual
	unsigned char  mutScanTime;			//	20
	unsigned char  mutTouchThres;		//	21
	unsigned char  mutSampleFreq;		//	24
	// Decode & tracking
	unsigned char  flipState;			//	25
	unsigned char  numOfAvg;			//	26
	unsigned char  minCuspDelta;		//	27
	unsigned char  weightThreshold;		//	28
	unsigned char  minTouchDistance;	//	29
	unsigned char  penDownTimer;		//	30
	unsigned char  penUpTimer;			//	31
	unsigned char  touchSuppressNum;	//	32
	unsigned short largeActThres;		//	33
	// charge pump
	unsigned char  cpTimeOut;			//	35
	//noise parameters
	unsigned char  selfNoiseThresh;		//	36
	unsigned char  mutNoiseThresh;		//	37
	unsigned char  frequencyChanges;	//	38
	unsigned char  sampleSize;			//	39
	unsigned char  selfNoiseScanTime;	//	40
	unsigned char  mutNoiseScanTime;	//	41
	unsigned char  filterCoeff;			//	42
	//gesture parameters
	unsigned char swipeLengthX;			//	43
	unsigned char swipeLengthY;			//	44
	unsigned char holdSwipeBoundary;	//	45
	unsigned char swipeHoldThresh;		//	46
	unsigned short swipeTime;			//	47
	unsigned short tapTime;				//	49
	unsigned char tapThresh;			//	51
	unsigned char minSwipeVelocity;		//	52
	unsigned short maxClickTime;		//	53
	unsigned char edgeKeepoutDistance;	//	55
	// port maps
	unsigned char  rxPinMap[MAXRX];		//	starts at 56
	unsigned char  txPinMap[MAXTX];		//
} USERRAM;

typedef struct _HWCFGRAM
{
	unsigned int sleepTimeout;
	unsigned char sleepConfig;
	unsigned char wdtTimeout;
	unsigned char diagPacketCfg;		// ADV, "Diagnostic Packet Config", Selects Disabled/Enabled and which port the packets are presented on
	unsigned char touchPacketCfg;		// ADV, "Touch Packet Config", Selects Disabled/Enabled and which port the packets are presented on
	unsigned char commandPacketCfg;		// ADV, "Command Packet Config", Selects Disabled/Enabled and which port the packets are presented on
	unsigned char gesturePacketCfg;		// ADV, "Gesture Packet Config", Selects Disabled/Enabled and which port the packets are presented on
	unsigned char statusPacketCfg;		// ADV, "Status Packet Config", Selects Disabled/Enabled and which port the packets are presented on
} HWCFGRAM;

typedef struct _HWSTATUSRAM
{
	unsigned char generalStatus;
	unsigned int txShortStatus;
	unsigned int rxShortStatus;

} HWSTATUSRAM;

// Packet Config bytes:
//		Bit 7 - Enable
//		Bit 0-6 - Port Enable
#define PACKETCFGENABLEMASK 0x80
#define PACKETCFGENABLE  	0x80
#define PACKETCFGPORT0 		0x01
#define PACKETCFGPORT1		0x02
#define PACKETCFGPORT2		0x03
#define PACKETCFGPORT3		0x04

//
// The C32 compiler requires this to be volatile or
// the interrupt rouitne will smash any global data
// used in the receive function
//
#ifdef PCAPPIC32MX
volatile typedef struct _COMMRAM
#else
typedef struct _COMMRAM
#endif
{
	unsigned char rcvCount;
	unsigned char rcvBuffer[RCVBUFFERSIZE];
	struct {
		unsigned char command;
		unsigned char result;
		unsigned char size;
//		unsigned char buffer[32];
	} response;
	unsigned char txCount;
	unsigned char txBuffer[64];
	unsigned char txWritten;
	#ifdef USB
	unsigned char usbBuffer[16];
	unsigned char loadIndex;
	unsigned char retrieveIndex;
	#endif
} COMMRAM;

typedef struct _DEBUGRAM
{
	unsigned char debugMask[DEBUG_MASK_SIZE];
    unsigned char ID;
    unsigned char size;
    unsigned char buffer[70];
} DEBUGRAM;

typedef struct _RAWRAM
{
	unsigned short rawSelf[MAXRX];
	unsigned short rawMut[MAXTX];
	unsigned short rawADC[128];
} RAWRAM;

typedef struct _VARRAM
{
	unsigned char baseUpdateCount;
	unsigned char baseUpdateState;
	unsigned char flag;
	unsigned int  guint3;
	unsigned int  gtempTRIS;
	unsigned int  gtempLAT[2];
	unsigned char gADCONCHS;


	#ifdef DEVKIT_HARDWARE
		#define STUTTER_SIZE MAXTX
	#else //DEVKIT_HARDWARE
		#define STUTTER_SIZE STUTTERMULT
	#endif //DEVKIT_HARDWARE

	#ifndef PCAPPIC18F
	volatile unsigned int *gtempTRISPORT;
	volatile unsigned int *gANSEL;
	volatile unsigned int *gtempLATPORT[2];
	unsigned int   gtempTRISClear;
	unsigned int   gANSELbit;
	unsigned short gtempLATA;
	unsigned short gtempLATAgrp[STUTTER_SIZE];
	unsigned short gtempLATB;
	unsigned short gtempLATBgrp[STUTTER_SIZE];
	unsigned short gtempLATC;
	unsigned short gtempLATCgrp[STUTTER_SIZE];
	unsigned short gtempLATD;
	unsigned short gtempLATDgrp[STUTTER_SIZE];
	unsigned short gtempLATE;
	unsigned short gtempLATEgrp[STUTTER_SIZE];
	unsigned short gtempLATF;
	unsigned short gtempLATFgrp[STUTTER_SIZE];
	unsigned short maskPortA;
	unsigned short maskPortB;
	unsigned short maskPortC;
	unsigned short maskPortD;
	unsigned short maskPortE;
	unsigned short maskPortF;
	#else
	unsigned char *gtempTRISPORT;
	unsigned char *gANSEL;
	unsigned char *gtempLATPORT[2];
	unsigned char  gtempTRISClear;
	unsigned char  gANSELbit;
	unsigned char  gtempLATA;
	unsigned char  gtempLATAgrp[STUTTER_SIZE];
	unsigned char  gtempLATB;
	unsigned char  gtempLATBgrp[STUTTER_SIZE];
	unsigned char  gtempLATC;
	unsigned char  gtempLATCgrp[STUTTER_SIZE];
	unsigned char  gtempLATD;
	unsigned char  gtempLATDgrp[STUTTER_SIZE];
	unsigned char  gtempLATE;
	unsigned char  gtempLATEgrp[STUTTER_SIZE];
	unsigned char  gtempLATF;
	unsigned char  gtempLATFgrp[STUTTER_SIZE];
	unsigned char  maskPortA;
	unsigned char  maskPortB;
	unsigned char  maskPortC;
	unsigned char  maskPortD;
	unsigned char  maskPortE;
	unsigned char  maskPortF;
	#endif
	unsigned char  stutterMultCache;
} VARRAM;

typedef struct _BASERAM
{
	unsigned short selfBase[MAXRX];
	unsigned short tempselfBase[MAXRX];
	unsigned short baseDelta[MAXRX];
	unsigned short mutBase[MAXRX][MAXTX];
	unsigned short tempMutBase[MAXTX];
} BASERAM;

typedef struct _NOISERAM
{
	short timerCount;
	unsigned char noisePresent;
	unsigned char triedFrequencies;
	unsigned char noiseFlag;
} NOISERAM;

//
// decode structures
//
typedef union _UCHARCOORD
{
	unsigned char value[2];
	struct {
		unsigned char x;
		unsigned char y;
	};
} UCHARCOORD;

typedef union _UINTCOORD
{
	unsigned short value[2];
	struct {
		unsigned short x;
		unsigned short y;
	};
} UINTCOORD;

/*! \var TD_TOUCH_STATE
	\brief Enumerated types for the touch state.

	Enum for the states in the Touch ID State Machine:
	\dot
	digraph TouchIDStateMachine {
		node [fontsize=10, fontname=Times];
		ID_Avail [style=bold, color=red, label="ID Available\nTS_ID_AVAILABLE"];
		ID_Alloc [color=red, label="ID Allocated\nNo Packet Sent\nTS_ID_ALLOCATED"];
		ID_TouchDown [color=green, style=bold, label="Touch Down\nPacket Sent\nTS_TOUCH_DOWN"];
		ID_Stream [color=green, style=bold, label = "Stream\nData Packets\nTS_TOUCH_STREAM"];
		ID_Search [color=red, label = "Searching for Touch\nNo Packets Sent\nTS_TOUCH_SEARCH"];
		ID_TouchUp [color=green, style=bold, label = "Touch Up\nPacket Sent\nTS_TOUCH_UP"];

		edge [fontsize=8, fontname=Courier, fontcolor=gray30];
		ID_Avail->ID_Alloc [label="Touch Detected\nPenDownTimer > 0"];
		ID_Avail->ID_TouchDown [style=bold, label="Touch Detected\nPenDownTimer = 0"];
		ID_Alloc->ID_Avail [label="No Touch Detected"];
		ID_Alloc->ID_Alloc [label="Touch Detected\nPenDownTimer-- > 0"];
		ID_Alloc->ID_TouchDown [label="Touch Detected\nPenDownTimer = 0"];
		ID_TouchDown->ID_Stream [style=bold, label="Touch Detected"];
		ID_TouchDown->ID_Search [label="No Touch Detected"];
		ID_Stream->ID_Stream [style=bold, label="Touch Detected"];
		ID_Stream->ID_Search [label="No Touch Detected\nPenUpTimer > 0"];
		ID_Stream->ID_TouchUp [style=bold, label="No Touch Detected\nPenUpTimer = 0"];
		ID_Search->ID_TouchUp [label="No Touch Detected\nPenUpTimer = 0"];
		ID_Search->ID_Search [label="No Touch Detected\nPenUpTimer-- > 0"];
		ID_Search->ID_Stream [label="Touch Detected"];
		ID_TouchUp->ID_Avail [style=bold, label="Complete:\nID Now Available"];
	}
	\enddot
*/
typedef enum {
		TS_ID_AVAILABLE = 0,  	/*!< ID Is currently unused and available to be allocated */
		TS_ID_ALLOCATED = 1,	/*!< ID Has been allocated but not yet transmitted */
		TS_TOUCH_DOWN   = 2,	/*!< Transmit a touch down packet for given ID */
		TS_TOUCH_STREAM = 3, 	/*!< Transmit the data stream for given ID */
		TS_TOUCH_SEARCH = 4, 	/*!< Last frame didn't have a matching touch, try to find one before Pen Up Timer expires */
		TS_TOUCH_UP     = 5 	/*!< Transmit a touch up packet, then free ID (set to \a TS_ID_AVAILABLE) */
	} TD_TOUCH_STATE;

//
// We track two different "things" for ID/Tracking
// Thing one is very basic - Location information for a potential touch
// This doesn't have any concept of "history", or even "Pen Down" or "Pen Up"
// It is purely "There appears to be a touch here".  There ia a potential for
// "Area" or other additional information, but iteration 1 will not include that.
// This structure includes an additional variable used for weighting and matching
// into a Thing two structure.
// Thing two is the "History" and touch identification information.  Thing two
// contains references to(copies of?) a group of Thing Ones, once they have been selected.
// Thing two also tracks the state of the touch.  Each thing two is an individual
// touch that is being tracked on the sensor.
//

//
// TouchFlags values
//
//
#define TF_LARGEACTIVATION			0x01 //!< Potential Large activation detected
#define TF_EXTRALARGEACTIVATION		0x02 //!< Potential Very Large activation detected

// TouchFlags Masks
#define TF_MASK_LARGEACTIVATION		0x03 //!< Mask for all activation types

typedef struct _TOUCHDATA
{
	UCHARCOORD   roughLocation;
    UINTCOORD    fineLocation;
	unsigned short touchStrength;
	unsigned short strengthSum;
	unsigned char  touchFlags; 	//!< Flags for additional touch information.
} TOUCHDATA;

typedef struct _TOUCHIDDATA
{
    UCHARCOORD   roughLocation;
    UINTCOORD    fineLocation;
} TOUCHIDDATA;

//
//! TOUCHSET contains the non-IDed touch locations.  This is the "base" structure
//! for all touch identification.
//
typedef struct _TOUCHSET
{
    unsigned char touchCount;  /*!< Current number of potential touches found */
    TOUCHDATA touchLoc[MAX_TOUCHES]; /*!< Locations of potential touches */
} TOUCHSET;

//
//! Data structure to track a touch and associated ID with history
//
typedef struct _TOUCHID
{
	TD_TOUCH_STATE touchState; /*!< Is this touch ID allocated, what is the state? */
	unsigned char StateTimer; /* variable used for Pen Up/Down timers for this touch */
	UINTCOORD scaleLocation; // Location scaled to 10 bit data.
	unsigned char historySize; // "Valid" history size, used for averaging
	TOUCHIDDATA touchLoc[TOUCH_HISTORY+1]; /*!< once matched, locations copied from TOUCHSET
												Array location 0 is always most recent, older
												locations are "pushed" backwards. */
} TOUCHID;

//
// TOUCHIDSET takes the data from TOUCHSET and associates it with a history of touches
// this then tracks each ID and the status of that ID.
//
typedef struct _TOUCHIDSET
{
	unsigned char weights[MAX_TOUCHES+1][MAX_TOUCHES+1]; // [TOUCHES][IDS]
	unsigned char touchCount;	/*!< Current number of touches */
    TOUCHID tData[MAX_TOUCHES]; /*!< set of all touches */
} TOUCHIDSET;


typedef struct _COLCACHE
{
	unsigned short mutCol[COLCACHESIZE][MAXTX];
	unsigned char  currentLeftEdge;
	unsigned char  currentRightEdge;
} COLCACHE;

typedef struct _GESTURE
{
	unsigned char gestureType;
	unsigned char data;
	unsigned char state;
	unsigned char swipeDirection;
	unsigned short start_x;
	unsigned short start_y;
	unsigned short second_x;
	unsigned short second_y;
	unsigned short timer;
	unsigned short holdVal;
	unsigned short xBoundary;
	unsigned short yBoundary;
} GESTURE;

//
// Headers
//
// main.c headers
//
unsigned char debugFeatures(void);

//
// init.c headers
//
void init(void);

//
// comm.c headers
//
void commReceive(void);
void sendCommandResponse(void);
void sendCommTouchReport(unsigned char whichReport);
void sendCommGestureReport(unsigned char whichID);
void sendCommStatusReport(unsigned char statusValue);

void sendDebugBytes(void);
void sendLongData(unsigned char ID);

//void cmdReadRam(void);
//void cmdWriteRam(void);
void cmdAccessRam(unsigned char cmd);

//
// pc.c headers
//
void initPC(void);
void stutterMaskSetup(void);
unsigned short scanChannels(unsigned char start, unsigned char stop, unsigned char scanMode);
void scanMutual(unsigned char channel, unsigned char TX, unsigned char scanMode);
void setupSelf(unsigned char x);
void setupMutual(unsigned char channel, unsigned char TX);
void mutualCapacitance(unsigned char channel, unsigned char scanMode);
void longSelfScan(void);
void longMutualScan(void);
void getInitialSelfBase(void);
void getInitialMutualBase(void);
unsigned short getNormalizedSelf(unsigned char arrayPosition);
unsigned short getNormalizedMutual(unsigned char x, unsigned char y);
void checkTouch(void);
void updateBaseline(void);
void checkStuckTouches(void);

//
//selfProcessing.c headers
//
void initNoise(void);
unsigned char scanNoise(unsigned char touchLocation);
unsigned char selfCheckNoise(unsigned char channel);
unsigned char mutCheckNoise(unsigned char channel);
void maFilter(unsigned char scanCount);

//
// decode.c headers
//
void decodeInit(void);
unsigned char findTouches(void);
void cancelTouch(unsigned char index);
unsigned char findNextPeak(unsigned short* values, unsigned char numValues, unsigned char startLocation, unsigned char threshold);
unsigned char nudgeLoc(TOUCHDATA *tData);
unsigned char findFineLocation(TOUCHDATA *tData);
unsigned char associateTouches(void);
unsigned short shortDistance(UINTCOORD pointA, UINTCOORD pointB);
unsigned char simpleDistance(UINTCOORD pointA, UINTCOORD pointB);
void extrapolateTouch(TOUCHID* thisTouch, UINTCOORD* next);
unsigned char touchWeight(unsigned char touchID, unsigned char potentialTouch);
void handleTouches(void);

//
// ColCache.c headers
//
unsigned char initColCache(void);
unsigned short queryColCache(unsigned char col, unsigned char row, unsigned char scanCycles); // scan = 0 "normal", 1 = "fast"

//
//gestures.c headers
//
void initGestures(void);
void checkGestureState(void);
unsigned char flipCorrectSwipe(unsigned char baseDir);
void detectSwipe(unsigned char ID);
void checkSwipeHold(unsigned char ID);
void detectTap(unsigned char ID);

//
// USBSerial.c headers
//
#ifdef USB
	void initUSB(void);
	void sendUSBChar(unsigned char *data, BYTE  length);
	void usbReceive(void);
	void sendUSBState(void);
#endif

#endif //ifndef __MAIN_H
