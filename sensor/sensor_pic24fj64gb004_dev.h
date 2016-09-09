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

#ifndef __SENSOR_PIC24FJ64GB004_H
#define __SENSOR_PIC24FJ64GB004_H

//
// Defines for specific touch sensors
// Select only ONE of the following #define options
// sensors are defined in sensor.h
//
//#define AMTP3002DSBCTMU			// AMT 3.5" 12x9 Displayless Sensor Board CTMU
//#define AMTP3002DSBCVD			// AMT 3.5" 12x9 Displayless Sensor Board CVD
#define RDV2					// 12x9 demo board with display


//
// Projected Constants, pre-defines for dev kit sensors
//

#ifdef AMTP3002DSBCTMU						// AMT 3.5" 12x9 Displayless Sensor Board CTMU
	//#define CHARGEPUMP						// define if using charge pump
//	#define CPDELAY 0xFF
	#define USEMUTDIFFERENCE	// if defined take the difference of TX 1 and 0
	#define RXPINMAP {RC3RX, RC2RX, RC1RX, RC0RX, RB3RX, RB2RX, RB1RX, RB0RX, RA1RX, RA0RX, RB15RX, RB14RX}	// Setup the pin mapping for the RX
	#define TXPINMAP {RA2TX, RA3TX, RA8TX, RB4TX, RA4TX, RA9TX, RC4TX, RC5TX, RA7TX}			// Setup the pin mapping for the TX
	#define NUMBEROFRXCHANNELS 12		// Set the number of CTMU channels
	#define NUMBEROFTXCHANNELS 9		// Set the number of TX channels
	#define SELFTOUCHTHRES 50 			// Set the threshold to compare the self measurement, if a bove we have a touch
	#define BASEUPDATETIME 50			// Set the frequency of the base update.  The time is based on loop and will vary based on all these parameters
	#define MUTTOUCHTHRES 100					// threshold to compare mutual measurement to, must be above
	#define FLIPSTATE 3 				// default flip state
	#define SELFSCANTIME 10				// Set the number of self ADC measuements to SUM for 1 measuement
	#define SELFSCANFINETUNE {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // Fine tune individual scan times for the sensor
	#define MUTSCANTIME 16				// Set the number of mutual ADC measuements to SUM for 1 measuement
	#define SELFDELAYTIME 24				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in self
	#define MUTDELAYTIME 1				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in mutual
	#define STUTTERMULT 1     			// Set the stutter multiplier to 1 (off)
	#ifndef USEMUTDIFFERENCE
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b10011000
		#else
		#define CUSTOMFLAG	0b10011101
		#endif
	#else
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b00011000
		#else
		#define CUSTOMFLAG	0b00011101
		#endif
	#endif
										//	bit	7	6	5	4	3	2	1	0
										//									|	- Only drive half the TX lines during self.  Use with charge pump to not over-drive the sginal
										//								|		- Clear CTMU, Set CVD
										//							|			- Use a delay when using the charge pump if set
										//						|				- Pulse 2 TX
										//					|					- Invert Self
										//				|						- Invert Mutual
										//			|							- set to enable noise routines, clear to disable
										//		|								- Use the difference in the mutual measuement
	#define NUMOFAVG 8					// Set the number of positions to average, note depends on the array size
	#define SELFCURRENT 2				// Set CTMU IRNG self: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define MUTCURRENT	3				// Set CTMU IRNG mutual: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define RXDIAGCHANNEL 0				// Channel 0 is default diagnostic channel
	#define TXDIAGCHANNEL 0				// TX channel 0 is default diagnostic channel
	#define SELFSAMPLEFREQ 1			// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define MUTSAMPLEFREQ 1				// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
//	#define SLEEPTIME 0					// 0 disabbles, else 1 = 262.144ms, 2 = 524.288ms, etc
	#define MINCUSPDELTA 5
	#define WEIGHTTHRESHOLD 0xff
	#define MINTOUCHDISTANCE 150
	#define PENDOWNTIMER 1
	#define PENUPTIMER 3
	#define CPTIMEOUT 18				// Timeout used for chargepump delay, 1 = 256us
	#define SELFNOISETHRESH		30				//threshold for self noise routines to start
	#define MUTNOISETHRESH 		24				//threshold for mutual noise routines to start
	#define FREQUENCYCHANGES 	32			//number of frequencies to try before canceling touch
	#define SAMPLESIZE			6				//actual number of samples to check for noise
	#define SELFNOISESCANTIME	10				//number of self scans to take when checking for noise
	#define MUTNOISESCANTIME	16				//number of mutual scans to take when checking for noise
	#define FILTERCOEFF			2				//EMA filter coefficient value (tied to both self and mutual) If zero, filters are turned off
	#define STUCKTHRESHOLD 50   // The maximum distance a "stuck" coordinate can move (final, interpolated, coordinates)
	#define STUCKTIMEOUT 6000   // How long an activation must be "stuck" before a re-baseline occurs
#endif // ifdef AMTP3002DSBCTMU

#ifdef AMTP3002DSBCVD						// AMT 3.5" 12x9 Displayless Sensor Board CVD
	//#define CHARGEPUMP						// define if using charge pump
	//#define CPDELAY 0xFF
	#define USEMUTDIFFERENCE	// if defined take the difference of TX 1 and 0
	#define RXPINMAP {RC3RX, RC2RX, RC1RX, RC0RX, RB3RX, RB2RX, RB1RX, RB0RX, RA1RX, RA0RX, RB15RX, RB14RX}	// Setup the pin mapping for the RX
	#define TXPINMAP {RA2TX, RA3TX, RA8TX, RB4TX, RA4TX, RA9TX, RC4TX, RC5TX, RA7TX}			// Setup the pin mapping for the TX
	#define NUMBEROFRXCHANNELS 12		// Set the number of CTMU channels
	#define NUMBEROFTXCHANNELS 9		// Set the number of TX channels
	#define SELFTOUCHTHRES 60 			// Set the threshold to compare the self measurement, if a bove we have a touch
	#define BASEUPDATETIME 50			// Set the frequency of the base update.  The time is based on loop and will vary based on all these parameters
	#define ACTIVATIONSUMTHRES 80 		// raw axis SUM threshold
	#define MUTTOUCHTHRES 100					// threshold to compare mutual measurement to, must be above
	#define FLIPSTATE 3 				// default flip state
	#define SELFSCANTIME 8				// Set the number of self ADC measuements to SUM for 1 measuement
	#define SELFSCANFINETUNE {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // Fine tune individual scan times for the sensor
	#define MUTSCANTIME 10				// Set the number of mutual ADC measuements to SUM for 1 measuement
	#define SELFDELAYTIME 1				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in self
	#define SHCHARGETIME 2				// Set the delay to allow the FVR to charge up the S/H capacitor on the ADC - CVD only
	#define MUTDELAYTIME 1				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in mutual
	#define STUTTERMULT 1     			// Set the stutter multiplier to 1 (off)
	#ifndef USEMUTDIFFERENCE
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b10011010
		#else
		#define CUSTOMFLAG	0b10011111
		#endif
	#else
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b00011010
		#else
		#define CUSTOMFLAG	0b00011111
		#endif
	#endif
										//	bit	7	6	5	4	3	2	1	0
										//									|	- Only drive half the TX lines during self.  Use with charge pump to not over-drive the sginal
										//								|		- Clear CTMU, Set CVD
										//							|			- Use a delay when using the charge pump if set
										//						|				- Pulse 2 TX
										//					|					- Invert Self
										//				|						- Invert Mutual
										//			|							- set to enable filters, clear to disable
										//		|								- Use the difference in the mutual measuement
	#define NUMOFAVG 8					// Set the number of positions to average, note depends on the array size
	#define SELFCURRENT 3				// Set CTMU IRNG self: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define MUTCURRENT	3				// Set CTMU IRNG mutual: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define RXDIAGCHANNEL 0			// Channel 0 is default diagnostic channel
	#define TXDIAGCHANNEL 0				// TX channel 0 is default diagnostic channel
	#define SELFSAMPLEFREQ 1			// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define MUTSAMPLEFREQ 1				// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define ADCADCS 0					// 2 - TAD = 500nS, 6 - TAD = 1uS
//	#define SLEEPTIME 0					// 0 disabbles, else 1 = 262.144ms, 2 = 524.288ms, etc
	#define MINCUSPDELTA 5
	#define WEIGHTTHRESHOLD 0xff
	#define MINTOUCHDISTANCE 150
	#define PENDOWNTIMER 2
	#define PENUPTIMER 3
	#define CPTIMEOUT 18				// Timeout used for chargepump delay, 1 = 256us
	#define SELFNOISETHRESH	25			//threshold for self noise routines to start
	#define MUTNOISETHRESH 	20			//threshold for mutual noise routines to start
	#define FREQUENCYCHANGES 	20		//number of frequencies to try before canceling touch
	#define SAMPLESIZE			5		//actual number of samples to check for noise
	#define SELFNOISESCANTIME	6		//number of self scans to take when checking for noise
	#define MUTNOISESCANTIME	8		//number of mutual scans to take when checking for noise
	#define FILTERCOEFF			0		//EMA filter coefficient value (tied to both self and mutual) If zero, filters are turned off
	#define STUCKTHRESHOLD 50   // The maximum distance a "stuck" coordinate can move (final, interpolated, coordinates)
	#define STUCKTIMEOUT 6000   // How long an activation must be "stuck" before a re-baseline occurs
#endif // ifdef AMTP3002DSBCVD

#ifdef RDV2						// AMT 3.5" 12x9 Displayless Sensor Board CVD
	//#define CHARGEPUMP						// define if using charge pump
	//#define CPDELAY 0xFF
	#define USEMUTDIFFERENCE	// if defined take the difference of TX 1 and 0
	#define RXPINMAP {RB13RX, RB14RX, RB15RX, RA0RX, RA1RX, RB0RX, RB1RX, RB2RX, RB3RX, RC0RX, RC1RX, RC2RX}	// Setup the pin mapping for the RX
	#define TXPINMAP {RA10TX, RA7TX, RA9TX, RC4TX, RA2TX, RA3TX, RA8TX, RB4TX, RA4TX}			// Setup the pin mapping for the TX
	#define NUMBEROFRXCHANNELS 12		// Set the number of CTMU channels
	#define NUMBEROFTXCHANNELS 9		// Set the number of TX channels
	#define SELFTOUCHTHRES 75 			// Set the threshold to compare the self measurement, if a bove we have a touch
	#define BASEUPDATETIME 10			// Set the frequency of the base update.  The time is based on loop and will vary based on all these parameters
	#define ACTIVATIONSUMTHRES 80 		// raw axis SUM threshold
	#define MUTTOUCHTHRES 75					// threshold to compare mutual measurement to, must be above
	#define FLIPSTATE 1 				// default flip state
	#define SELFSCANTIME 7				// Set the number of self ADC measuements to SUM for 1 measuement
	#define SELFSCANFINETUNE {1, 0, 0, 4, 4, 0, 0, 1, 0, 1, 1, 2} // Fine tune individual scan times for the sensor
	#define MUTSCANTIME 15				// Set the number of mutual ADC measuements to SUM for 1 measuement
	#define SELFDELAYTIME 1				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in self
	#define SHCHARGETIME 2				// Set the delay to allow the FVR to charge up the S/H capacitor on the ADC - CVD only
	#define MUTDELAYTIME 1				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in mutual
	#define STUTTERMULT 1     			// Set the stutter multiplier to 1 (off)
	#ifndef USEMUTDIFFERENCE
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b10011010
		#else
		#define CUSTOMFLAG	0b10011111
		#endif
	#else
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b00011010
		#else
		#define CUSTOMFLAG	0b00011111
		#endif
	#endif
										//	bit	7	6	5	4	3	2	1	0
										//									|	- Only drive half the TX lines during self.  Use with charge pump to not over-drive the sginal
										//								|		- Clear CTMU, Set CVD
										//							|			- Use a delay when using the charge pump if set
										//						|				- Pulse 2 TX
										//					|					- Invert Self
										//				|						- Invert Mutual
										//			|							- set to enable filters, clear to disable
										//		|								- Use the difference in the mutual measuement
	#define NUMOFAVG 8					// Set the number of positions to average, note depends on the array size
	#define SELFCURRENT 3				// Set CTMU IRNG self: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define MUTCURRENT	3				// Set CTMU IRNG mutual: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define RXDIAGCHANNEL 0			// Channel 0 is default diagnostic channel
	#define TXDIAGCHANNEL 0				// TX channel 0 is default diagnostic channel
	#define SELFSAMPLEFREQ 1			// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define MUTSAMPLEFREQ 1				// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define ADCADCS 0					// 2 - TAD = 500nS, 6 - TAD = 1uS
//	#define SLEEPTIME 0					// 0 disables, else 1 = 262.144ms, 2 = 524.288ms, etc
	#define MINCUSPDELTA 5
	#define WEIGHTTHRESHOLD 0xff
	#define MINTOUCHDISTANCE 150
	#define PENDOWNTIMER 1
	#define PENUPTIMER 1
	#define CPTIMEOUT 18				// Timeout used for chargepump delay, 1 = 256us
	#define SELFNOISETHRESH	25			//threshold for self noise routines to start
	#define MUTNOISETHRESH 	20			//threshold for mutual noise routines to start
	#define FREQUENCYCHANGES 	20		//number of frequencies to try before canceling touch
	#define SAMPLESIZE			5		//actual number of samples to check for noise
	#define SELFNOISESCANTIME	6		//number of self scans to take when checking for noise
	#define MUTNOISESCANTIME	8		//number of mutual scans to take when checking for noise
	#define FILTERCOEFF			0		//EMA filter coefficient value (tied to both self and mutual) If zero, filters are turned off
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
#endif // ifdef RDV2

#endif //ifndef __SENSOR_PIC24FJ64GB004_H
