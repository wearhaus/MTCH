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

#ifndef __SENSOR_PIC32MX120F032D_H
#define __SENSOR_PIC32MX120F032D_H


//
// Defines for specific touch sensors
// Select only ONE of the following #define options
// sensors are defined in sensor.h
//
//#define AMTP3002DSBCTMU					// AMT 3.5" 12x9 Displayless Sensor Board CTMU
#define AMTP3002DSBCVD					// AMT 3.5" 12x9 Displayless Sensor Board CVD
//#define TPD_SENSOR			// Sensor for the Touch Pad Demonstrator
//#define PCAPPIC18FDEVBMPK22V6PCB

//
// Projected Constants, pre-defines for dev kit sensors
//

#ifdef AMTP3002DSBCTMU					// AMT 3.5" 12x9 Displayless Sensor Board CTMU
	//#define CHARGEPUMP				// define if using charge pump
	#define CPDELAY 0xFF
	#define USEMUTDIFFERENCE			// if defined take the difference of TX 1 and 0
	#define RXPINMAP {RC2RX, RC1RX, RC0RX, RB3RX, RB2RX, RB1RX, RB0RX, RA1RX, RA0RX, RB15RX, RB14RX, RB13RX}	// Setup the pin mapping for the RX
	#define TXPINMAP {RB4TX, RA8TX, RA3TX, RA2TX, RA4TX, RA9TX, RC3TX, RC4TX, RC5TX}			// Setup the pin mapping for the TX
	#define NUMBEROFRXCHANNELS 12		// Set the number of CTMU channels
	#define NUMBEROFTXCHANNELS 9		// Set the number of TX channels
	#define SELFTOUCHTHRES 37 			// Set the threshold to compare the self measurement, if a bove we have a touch
	#define BASEUPDATETIME 50			// Set the frequency of the base update.  The time is based on loop and will vary based on all these parameters
	#define MUTTOUCHTHRES 45					// threshold to compare mutual measurement to, must be above
	#define FLIPSTATE 3 				// default flip state
	#define SELFSCANTIME 8				// Set the number of self ADC measuements to SUM for 1 measuement
	#define SELFSCANFINETUNE {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // Fine tune individual scan times for the sensor
	#define MUTSCANTIME 30				// Set the number of mutual ADC measuements to SUM for 1 measuement
	#define SELFDELAYTIME 4//7				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in self
	#define MUTDELAYTIME 16				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in mutual
	#ifndef USEMUTDIFFERENCE
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b10010000
		#else
		#define CUSTOMFLAG	0b10010001
		#endif
	#else
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b00010000
		#else
		#define CUSTOMFLAG	0b00010001
		#endif
	#endif
										//	bit	7	6	5	4	3	2	1	0
										//									|	- Only drive half the TX lines during self.  Use with charge pump to not over-drive the sginal
										//								|		- Clear CTMU, Set CVD
										//							|			- Use a delay when using the charge pump if set
										//						|				- NA
										//					|					- Invert Self
										//				|						- Invert Mutual
										//			|							- NA
										//		|								- Use the difference in the mutual measuement
	#define NUMOFAVG 8					// Set the number of positions to average, note depends on the array size
	#define SELFCURRENT 1				// Set CTMU IRNG self: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define MUTCURRENT	1				// Set CTMU IRNG mutual: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define RXDIAGCHANNEL 0			// Channel 0 is default diagnostic channel
	#define TXDIAGCHANNEL 0				// TX channel 0 is default diagnostic channel
	#define SELFSAMPLEFREQ 2			// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define MUTSAMPLEFREQ 2				// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	//#define SLEEPTIME 0					// 0 disabbles, else 1 = 262.144ms, 2 = 524.288ms, etc
	#define MINCUSPDELTA 5
	#define WEIGHTTHRESHOLD 0xff
	#define MINTOUCHDISTANCE 150
	#define PENDOWNTIMER 1
	#define PENUPTIMER 3
	#define STUTTERMULT 1
	#define CPTIMEOUT 18				// Timeout used for chargepump delay, 1 = 256us
	#define SELFNOISETHRESH	25				//threshold for self noise routines to start
	#define MUTNOISETHRESH 	20				//threshold for mutual noise routines to start
	#define FREQUENCYCHANGES 	20//32			//number of frequencies to try before canceling touch
	#define SAMPLESIZE			5				//actual number of samples to check for noise
	#define SELFNOISESCANTIME	6				//number of self scans to take when checking for noise
	#define MUTNOISESCANTIME	8				//number of mutual scans to take when checking for noise
	#define FILTERCOEFF			0				//EMA filter coefficient value (tied to both self and mutual) If zero, filters are turned off
#endif // ifdef AMTP3002DSBCTMU

#ifdef AMTP3002DSBCVD						// AMT 3.5" 12x9 Displayless Sensor Board CVD
	#define SLEEPCONFIG SLEEP_SLEEP
	#define WDTTIMEOUT WDT_PS128
	#define DIAGPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	#define TOUCHPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	#define COMMANDPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	#define GESTUREPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	#define STATUSPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	//#define CHARGEPUMP						// define if using charge pump
	#define CPDELAY 0xFF
	#define USEMUTDIFFERENCE	// if defined take the difference of TX 1 and 0
	//#define RXPINMAP {RB12RX, RB13RX, RB15RX, RA0RX, RA1RX, RB0RX, RB1RX, RB2RX, RB3RX, RC0RX, RC1RX, RC2RX}	// Setup the pin mapping for the RX
	//#define TXPINMAP {RC9TX, RC8TX, RC7TX, RC6TX, RA2TX, RA3TX, RA8TX, RB4TX, RA4TX}			// Setup the pin mapping for the TX
	#define RXPINMAP {RC2RX, RC1RX, RC0RX, RB3RX, RB2RX, RB1RX, RB0RX, RA1RX, RA0RX, RB15RX, RB14RX, RB13RX}	// Setup the pin mapping for the RX
	#define TXPINMAP {RB4TX, RA8TX, RA3TX, RA2TX, RA4TX, RA9TX, RC3TX, RC4TX, RC5TX}			// Setup the pin mapping for the TX
	#define NUMBEROFRXCHANNELS 12		// Set the number of CTMU channels
	#define NUMBEROFTXCHANNELS 9		// Set the number of TX channels
	#define SELFTOUCHTHRES 60 			// Set the threshold to compare the self measurement, if a bove we have a touch
	#define BASEUPDATETIME 50			// Set the frequency of the base update.  The time is based on loop and will vary based on all these parameters
	#define MUTTOUCHTHRES 100					// threshold to compare mutual measurement to, must be above
	#define FLIPSTATE 3 				// default flip state
	#define SELFSCANTIME 8				// Set the number of self ADC measuements to SUM for 1 measuement
	#define SELFSCANFINETUNE {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // Fine tune individual scan times for the sensor
	#define MUTSCANTIME 9				// Set the number of mutual ADC measuements to SUM for 1 measuement
	#define MUTSCANFINETUNE {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // Fine tune individual scan times for the sensor
	#define	SELFDIVIDER 32				// Set the divider used for self raw measerments, left shifted 5, 32 = divide by 1
	#define MUTDIVIDER 32				// Set the divider used for mutual raw measerments, left shifted 5, 32 = divide by 1
	#define SELFDELAYTIME 1//7				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in self
	#define MUTDELAYTIME 1				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in mutual
	#define STUTTERMULT 1
	#ifndef USEMUTDIFFERENCE
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b10011010
		#else
		#define CUSTOMFLAG	0b10011011
		#endif
	#else
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b00011010
		#else
		#define CUSTOMFLAG	0b00011011
		#endif
	#endif
										//	bit	7	6	5	4	3	2	1	0
										//									|	- Only drive half the TX lines during self.  Use with charge pump to not over-drive the sginal
										//								|		- Clear CTMU, Set CVD
										//							|			- Use a delay when using the charge pump if set
										//						|				- Pulse 2 TX
										//					|					- Invert Self
										//				|						- Invert Mutual
										//			|							- Noise Routine enable/disable
										//		|								- Use the difference in the mutual measuement
	#define NUMOFAVG 8					// Set the number of positions to average, note depends on the array size
	#define SELFCURRENT 1				// Set CTMU IRNG self: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define MUTCURRENT	1				// Set CTMU IRNG mutual: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define RXDIAGCHANNEL 0			// Channel 0 is default diagnostic channel
	#define TXDIAGCHANNEL 0				// TX channel 0 is default diagnostic channel
	#define SELFSAMPLEFREQ 2			// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define MUTSAMPLEFREQ 2				// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define SLEEPTIME 0					// 0 disabbles,x else 1 = 262.144ms, 2 = 524.288ms, etc
	#define MINCUSPDELTA 5
	#define WEIGHTTHRESHOLD 0xff
	#define MINTOUCHDISTANCE 150
	#define PENDOWNTIMER 2
	#define PENUPTIMER 3
	#define CPTIMEOUT 18					// Timeout used for chargepump delay, 1 = 256us
	#define STUTTERMULT 1
	#define SELFNOISETHRESH		28			//threshold for self noise routines to start
	#define MUTNOISETHRESH 		20			//threshold for mutual noise routines to start
	#define FREQUENCYCHANGES 	32			//number of frequencies to try before canceling touch
	#define SAMPLESIZE			5			//actual number of samples to check for noise
	#define SELFNOISESCANTIME	7			//number of self scans to take when checking for noise
	#define MUTNOISESCANTIME	12			//number of mutual scans to take when checking for noise
	#define FILTERCOEFF			0			//MA filter coefficient value (tied to both self and mutual) If zero, filters are turned off
	#define SWIPELENGTHX 160			// Minimum length (in x-y pos) required to be detected as a horizontal swipe
	#define SWIPELENGTHY 150			// Minimum length (in x-y pos) required to be detected as a veritcal swipe
	#define HOLDSWIPEBOUNDARY 150		// Distance user can move in any direction after swipe becomes a hold
	#define SWIPETIME 200				// Maximum allowable time from PD for a swipe to occur
	#define SWIPEHOLDTHRESH 70			// movement allowance in opposite direction when holding a swipe
	#define TAPTHRESH 120				// movement allowance for a single tap gesture2
	#define TAPTIME 500					// length of time to hold single tap before gesture data is sent: time(ms) = 4*value
	#define MINSWIPEVELOCITY 3			// minimum velocity that must be sustained to be a swipe (not a hold)
	#define MAXCLICKTIME 350
	#define EDGEKEEPOUTDISTANCE 128
	#define STUCKTHRESHOLD 50   // The maximum distance a "stuck" coordinate can move (final, interpolated, coordinates)
	#define STUCKTIMEOUT 6000   // How long an activation must be "stuck" before a re-baseline occurs
#endif // ifdef AMTP3002DSBCVD

#ifdef TPD_SENSOR						// AMT 3.5" 12x9 Displayless Sensor Board CVD
	#define SLEEPCONFIG SLEEP_SLEEP
	#define WDTTIMEOUT WDT_PS128
	#define DIAGPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	#define TOUCHPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	#define COMMANDPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	#define GESTUREPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	#define STATUSPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	//#define CHARGEPUMP						// define if using charge pump
	#define CPDELAY 0xFF
	#define USEMUTDIFFERENCE	// if defined take the difference of TX 1 and 0
	#define RXPINMAP {RB12RX, RB13RX, RB15RX, RA0RX, RA1RX, RB0RX, RB1RX, RB2RX, RB3RX, RC0RX, RC1RX, RC2RX}	// Setup the pin mapping for the RX
	#define TXPINMAP {RC9TX, RC8TX, RC7TX, RC6TX, RA2TX, RA3TX, RA8TX, RB4TX, RA4TX}			// Setup the pin mapping for the TX
	#define NUMBEROFRXCHANNELS 12		// Set the number of CTMU channels
	#define NUMBEROFTXCHANNELS 9		// Set the number of TX channels
	#define SELFTOUCHTHRES 60 			// Set the threshold to compare the self measurement, if a bove we have a touch
	#define BASEUPDATETIME 50			// Set the frequency of the base update.  The time is based on loop and will vary based on all these parameters
	#define MUTTOUCHTHRES 100					// threshold to compare mutual measurement to, must be above
	#define FLIPSTATE 1 				// default flip state
	#define SELFSCANTIME 8				// Set the number of self ADC measuements to SUM for 1 measuement
	#define SELFSCANFINETUNE {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // Fine tune individual scan times for the sensor
	#define MUTSCANTIME 9				// Set the number of mutual ADC measuements to SUM for 1 measuement
	#define MUTSCANFINETUNE {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // Fine tune individual scan times for the sensor
	#define	SELFDIVIDER 32				// Set the divider used for self raw measerments, left shifted 5, 32 = divide by 1
	#define MUTDIVIDER 32				// Set the divider used for mutual raw measerments, left shifted 5, 32 = divide by 1
	#define SELFDELAYTIME 1//7				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in self
	#define MUTDELAYTIME 1				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in mutual
	#define STUTTERMULT 1
	#ifndef USEMUTDIFFERENCE
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b10011010
		#else
		#define CUSTOMFLAG	0b10011011
		#endif
	#else
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b00011010
		#else
		#define CUSTOMFLAG	0b00011011
		#endif
	#endif
										//	bit	7	6	5	4	3	2	1	0
										//									|	- Only drive half the TX lines during self.  Use with charge pump to not over-drive the sginal
										//								|		- Clear CTMU, Set CVD
										//							|			- Use a delay when using the charge pump if set
										//						|				- Pulse 2 TX
										//					|					- Invert Self
										//				|						- Invert Mutual
										//			|							- Noise Routine enable/disable
										//		|								- Use the difference in the mutual measuement
	#define NUMOFAVG 8					// Set the number of positions to average, note depends on the array size
	#define SELFCURRENT 1				// Set CTMU IRNG self: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define MUTCURRENT	1				// Set CTMU IRNG mutual: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define RXDIAGCHANNEL 0			// Channel 0 is default diagnostic channel
	#define TXDIAGCHANNEL 0				// TX channel 0 is default diagnostic channel
	#define SELFSAMPLEFREQ 2			// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define MUTSAMPLEFREQ 2				// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define SLEEPTIME 0					// 0 disabbles,x else 1 = 262.144ms, 2 = 524.288ms, etc
	#define MINCUSPDELTA 5
	#define WEIGHTTHRESHOLD 0xff
	#define MINTOUCHDISTANCE 150
	#define PENDOWNTIMER 2
	#define PENUPTIMER 3
	#define CPTIMEOUT 18					// Timeout used for chargepump delay, 1 = 256us
	#define STUTTERMULT 1
	#define SELFNOISETHRESH		28			//threshold for self noise routines to start
	#define MUTNOISETHRESH 		20			//threshold for mutual noise routines to start
	#define FREQUENCYCHANGES 	32			//number of frequencies to try before canceling touch
	#define SAMPLESIZE			5			//actual number of samples to check for noise
	#define SELFNOISESCANTIME	7			//number of self scans to take when checking for noise
	#define MUTNOISESCANTIME	12			//number of mutual scans to take when checking for noise
	#define FILTERCOEFF			0			//MA filter coefficient value (tied to both self and mutual) If zero, filters are turned off
	#define SWIPELENGTHX 160			// Minimum length (in x-y pos) required to be detected as a horizontal swipe
	#define SWIPELENGTHY 150			// Minimum length (in x-y pos) required to be detected as a veritcal swipe
	#define HOLDSWIPEBOUNDARY 150		// Distance user can move in any direction after swipe becomes a hold
	#define SWIPETIME 200				// Maximum allowable time from PD for a swipe to occur
	#define SWIPEHOLDTHRESH 70			// movement allowance in opposite direction when holding a swipe
	#define TAPTHRESH 120				// movement allowance for a single tap gesture2
	#define TAPTIME 500					// length of time to hold single tap before gesture data is sent: time(ms) = 4*value
	#define MINSWIPEVELOCITY 3			// minimum velocity that must be sustained to be a swipe (not a hold)
	#define MAXCLICKTIME 350
	#define EDGEKEEPOUTDISTANCE 128
	#define STUCKTHRESHOLD 50   // The maximum distance a "stuck" coordinate can move (final, interpolated, coordinates)
	#define STUCKTIMEOUT 6000   // How long an activation must be "stuck" before a re-baseline occurs
#endif // ifdef AMTP3002DSBCVD

#ifdef PCAPPIC18FDEVBMPK22V6PCB
	#define SLEEPCONFIG SLEEP_SLEEP
	#define WDTTIMEOUT WDT_PS128
	#define DIAGPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	#define TOUCHPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	#define COMMANDPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	#define GESTUREPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	#define STATUSPACKETCFG PACKETCFGENABLE | PACKETCFGPORT0
	//#define CHARGEPUMP						// define if using charge pump
	#define CPDELAY 0xFF
	#define USEMUTDIFFERENCE	// if defined take the difference of TX 1 and 0
	#define RXPINMAP {RC2RX, RC1RX, RC0RX, RB3RX, RB2RX, RB1RX, RB0RX, RA1RX, RA0RX, RB15RX, RB14RX, RB13RX}	// Setup the pin mapping for the RX
	#define TXPINMAP {RB4TX, RA8TX, RA3TX, RA2TX, RA4TX, RC5TX, RC4TX, RC3TX, RA9TX}			// Setup the pin mapping for the TX
	#define NUMBEROFRXCHANNELS 12		// Set the number of CTMU channels
	#define NUMBEROFTXCHANNELS 9		// Set the number of TX channels
	#define SELFTOUCHTHRES 50 			// Set the threshold to compare the self measurement, if a bove we have a touch
	#define BASEUPDATETIME 50			// Set the frequency of the base update.  The time is based on loop and will vary based on all these parameters
	#define MUTTOUCHTHRES 55					// threshold to compare mutual measurement to, must be above
	#define FLIPSTATE 1 				// default flip state
	#define SELFSCANTIME 5				// Set the number of self ADC measuements to SUM for 1 measuement
	#define SELFSCANFINETUNE {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // Fine tune individual scan times for the sensor
	#define MUTSCANTIME 9				// Set the number of mutual ADC measuements to SUM for 1 measuement
	#define MUTSCANFINETUNE {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // Fine tune individual scan times for the sensor
	#define	SELFDIVIDER 32				// Set the divider used for self raw measerments, left shifted 5, 32 = divide by 1
	#define MUTDIVIDER 32				// Set the divider used for mutual raw measerments, left shifted 5, 32 = divide by 1
	#define SELFDELAYTIME 4//7				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in self
	#define MUTDELAYTIME 6
	#ifndef USEMUTDIFFERENCE
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b11011010
		#else
		#define CUSTOMFLAG	0b11011011
		#endif
	#else
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b01011010
		#else
		#define CUSTOMFLAG	0b01011011
		#endif
	#endif
										//	bit	7	6	5	4	3	2	1	0
										//									|	- N/A (unused)
										//								|		- Clear CTMU, Set CVD
										//							|			- Use a delay when using the charge pump if set
										//						|				- NA
										//					|					- Invert Self
										//				|						- Invert Mutual
										//			|							- NA
										//		|								- Use the difference in the mutual measuement
	#define NUMOFAVG 16					// Set the number of positions to average, note depends on the array size
	#define SELFCURRENT 1				// Set CTMU IRNG self: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define MUTCURRENT	2				// Set CTMU IRNG mutual: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define RXDIAGCHANNEL 0				// Channel 0 is default diagnostic channel
	#define TXDIAGCHANNEL 0				// TX channel 0 is default diagnostic channel
	#define SELFSAMPLEFREQ 1			// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define MUTSAMPLEFREQ 1				// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define MUTOVERSAMPLE 8				// Used for digital AC filter, take this many more samples before filtering
	#define SLEEPTIME 0					// 0 disabbles, else 1 = 262.144ms, 2 = 524.288ms, etc
	#define MINCUSPDELTA 5
	#define WEIGHTTHRESHOLD 0xff
	#define MINTOUCHDISTANCE 150
	#define PENDOWNTIMER 3
	#define PENUPTIMER 3
	#define CPTIMEOUT 18				// Timeout used for chargepump delay, 1 = 256us
	#define STUTTERMULT 1
	#define SELFNOISETHRESH		25				//threshold for self noise routines
	#define MUTNOISETHRESH 		23				//threshold for mutual noise routines
	#define FREQUENCYCHANGES 	20//32			//number of frequencies to try before canceling touch
	#define MAXSAMPLESIZE 		20				//maximum number of samples to check for noise
	#define SAMPLESIZE			5				//actual number of samples to check for noise
	#define SAMPLECHANNEL 		4				//TX channel to sample to check for noise
	#define SELFNOISESCANTIME	4				//number of self scans to take when checking for noise
	#define MUTNOISESCANTIME	7				//number of mutual scans to take when checking for noise
	#define FILTERCOEFF			0				//EMA filter coefficient value (tied to both self and mutual) If zero, filters are turned off
	#define SWIPELENGTHX 160			// Minimum length (in x-y pos) required to be detected as a horizontal swipe
	#define SWIPELENGTHY 150			// Minimum length (in x-y pos) required to be detected as a veritcal swipe
	#define HOLDSWIPEBOUNDARY 150		// Distance user can move in any direction after swipe becomes a hold
	#define SWIPETIME 200				// Maximum allowable time from PD for a swipe to occur
	#define SWIPEHOLDTHRESH 70			// movement allowance in opposite direction when holding a swipe
	#define TAPTHRESH 120				// movement allowance for a single tap gesture2
	#define TAPTIME 500					// length of time to hold single tap before gesture data is sent: time(ms) = 4*value
	#define MINSWIPEVELOCITY 3			// minimum velocity that must be sustained to be a swipe (not a hold)
	#define MAXCLICKTIME 350
	#define EDGEKEEPOUTDISTANCE 128
	#define STUCKTHRESHOLD 50   // The maximum distance a "stuck" coordinate can move (final, interpolated, coordinates)
	#define STUCKTIMEOUT 6000   // How long an activation must be "stuck" before a re-baseline occurs
#endif // ifdef PCAPPIC18FDEVBMPK22V6PCB

#endif //ifndef __SENSOR_PIC32MX120F032D_H
