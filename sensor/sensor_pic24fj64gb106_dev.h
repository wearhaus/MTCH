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

#ifndef __SENSOR_PIC24FJ64GB106_H
#define __SENSOR_PIC24FJ64GB106_H

//
// Defines for specific touch sensors
// Select only ONE of the following #define options
// sensors are defined in sensor.h
//
#define AMTP3002DSBCTMU					// AMT 3.5" 12x9 Displayless Sensor Board CTMU
//#define AMTP3002DSBCVD					// AMT 3.5" 12x9 Displayless Sensor Board CVD
//#define AMTP3006DSBCTMU				// AMT 7" 15x24 Displayless Sensor Board CTMU
//#define AMTP3006DSBCVD				// AMT 7" 15x24 Displayless Sensor Board CVD

//
// Projected Constants, pre-defines for dev kit sensors
//

#ifdef AMTP3002DSBCTMU						// AMT 3.5" 12x9 Displayless Sensor Board CTMU
	//#define CHARGEPUMP						// define if using charge pump
	#define CPDELAY 0xFF
	#define USEMUTDIFFERENCE	// if defined take the difference of TX 1 and 0
	#define RXPINMAP {RB5RX, RB4RX, RB3RX, RB2RX, RB6RX, RB7RX, RB8RX, RB9RX, RB10RX, RB11RX, RB15RX, RB14RX}	// Setup the pin mapping for the RX
	#define TXPINMAP {RE7TX, RE6TX, RE5TX, RE4TX, RE3TX, RE2TX, RE1TX, RE0TX, RF1TX}			// Setup the pin mapping for the TX
	#define NUMBEROFRXCHANNELS 12		// Set the number of CTMU channels
	#define NUMBEROFTXCHANNELS 9		// Set the number of TX channels
	#define SELFTOUCHTHRES 65 			// Set the threshold to compare the self measurement, if a bove we have a touch
	#define BASEUPDATETIME 50			// Set the frequency of the base update.  The time is based on loop and will vary based on all these parameters
	#define MUTTOUCHTHRES 110					// threshold to compare mutual measurement to, must be above
	#define FLIPSTATE 3 				// default flip state
	#define SELFSCANTIME 12				// Set the number of self ADC measuements to SUM for 1 measuement
	#define MUTSCANTIME 16				// Set the number of mutual ADC measuements to SUM for 1 measuement
	#define	SELFDIVIDER 32				// Set the divider used for self raw measerments, left shifted 5, 32 = divide by 1
	#define MUTDIVIDER 32				// Set the divider used for mutual raw measerments, left shifted 5, 32 = divide by 1
	#define SELFDELAYTIME 24//7				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in self
	#define MUTDELAYTIME 1//24				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in mutual
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
										//									|	- N/A (unused)
										//								|		- Clear CTMU, Set CVD
										//							|			- Use a delay when using the charge pump if set
										//						|				- NA
										//					|					- Invert Self
										//				|						- Invert Mutual
										//			|							- NA
										//		|								- Use the difference in the mutual measuement
	#define NUMOFAVG 8					// Set the number of positions to average, note depends on the array size
	#define SELFCURRENT 2				// Set CTMU IRNG self: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define MUTCURRENT	3				// Set CTMU IRNG mutual: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define RXDIAGCHANNEL 0			// Channel 0 is default diagnostic channel
	#define TXDIAGCHANNEL 0				// TX channel 0 is default diagnostic channel
	#define SELFSAMPLEFREQ 1			// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define MUTSAMPLEFREQ 1				// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define SLEEPTIME 0					// 0 disabbles, else 1 = 262.144ms, 2 = 524.288ms, etc
	#define MINCUSPDELTA 5
	#define WEIGHTTHRESHOLD 0xff
	#define MINTOUCHDISTANCE 150
	#define PENDOWNTIMER 2
	#define PENUPTIMER 3
	#define CPTIMEOUT 18				// Timeout used for chargepump delay, 1 = 256us
	#define STUTTERMULT  1				// TX STUTTER multiplier
	#define SELFNOISETHRESH		23				//threshold for self noise routines to start
	#define MUTNOISETHRESH 		22				//threshold for mutual noise routines to start
	#define FREQUENCYCHANGES 	32			//number of frequencies to try before canceling touch
	#define MAXSAMPLESIZE 		20				//maximum number of samples to check for noise
	#define SAMPLESIZE			8				//actual number of samples to check for noise
	#define SAMPLECHANNEL 		4				//TX channel to sample to check for noise
	#define SELFNOISESCANTIME	8				//number of self scans to take when checking for noise
	#define MUTNOISESCANTIME	16				//number of mutual scans to take when checking for noise
	#define FILTERCOEFF			0				//EMA filter coefficient value (tied to both self and mutual) If zero, filters are turned off

#endif // ifdef AMTP3002DSBCTMU

#ifdef AMTP3002DSBCVD						// AMT 3.5" 12x9 Displayless Sensor Board CVD
	//#define CHARGEPUMP						// define if using charge pump
	#define CPDELAY 0xFF
	#define USEMUTDIFFERENCE	// if defined take the difference of TX 1 and 0
	#define RXPINMAP {RB5RX, RB4RX, RB3RX, RB2RX, RB6RX, RB7RX, RB8RX, RB9RX, RB10RX, RB11RX, RB15RX, RB14RX}	// Setup the pin mapping for the RX
	#define TXPINMAP {RE7TX, RE6TX, RE5TX, RE4TX, RE3TX, RE2TX, RE1TX, RE0TX, RF1TX}			// Setup the pin mapping for the TX
	#define NUMBEROFRXCHANNELS 12		// Set the number of CTMU channels
	#define NUMBEROFTXCHANNELS 9		// Set the number of TX channels
	#define SELFTOUCHTHRES 65 			// Set the threshold to compare the self measurement, if a bove we have a touch
	#define BASEUPDATETIME 50			// Set the frequency of the base update.  The time is based on loop and will vary based on all these parameters
	#define MUTTOUCHTHRES 110					// threshold to compare mutual measurement to, must be above
	#define FLIPSTATE 3 				// default flip state
	#define SELFSCANTIME 6				// Set the number of self ADC measuements to SUM for 1 measuement
	#define MUTSCANTIME 15				// Set the number of mutual ADC measuements to SUM for 1 measuement
	#define	SELFDIVIDER 32				// Set the divider used for self raw measerments, left shifted 5, 32 = divide by 1
	#define MUTDIVIDER 32				// Set the divider used for mutual raw measerments, left shifted 5, 32 = divide by 1
	#define SELFDELAYTIME 1//7			// Set the delay to wait before ADC measuement after pulsing the TX line(s) in self
	#define MUTDELAYTIME 1//24			// Set the delay to wait before ADC measuement after pulsing the TX line(s) in mutual
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
										//									|	- N/A (unused)
										//								|		- Clear CTMU, Set CVD
										//							|			- Use a delay when using the charge pump if set
										//						|				- NA
										//					|					- Invert Self
										//				|						- Invert Mutual
										//			|							- Turn on/off noise routines
										//		|								- Use the difference in the mutual measuement
	#define NUMOFAVG 8					// Set the number of positions to average, note depends on the array size
	#define SELFCURRENT 2				// Set CTMU IRNG self: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define MUTCURRENT	3				// Set CTMU IRNG mutual: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define RXDIAGCHANNEL 0			// Channel 0 is default diagnostic channel
	#define TXDIAGCHANNEL 0				// TX channel 0 is default diagnostic channel
	#define SELFSAMPLEFREQ 1			// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define MUTSAMPLEFREQ 1				// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define SLEEPTIME 0					// 0 disabbles, else 1 = 262.144ms, 2 = 524.288ms, etc
	#define MINCUSPDELTA 5
	#define WEIGHTTHRESHOLD 0xff
	#define MINTOUCHDISTANCE 150
	#define PENDOWNTIMER 2
	#define PENUPTIMER 3
	#define CPTIMEOUT 18				// Timeout used for chargepump delay, 1 = 256us
	#define STUTTERMULT  1				// TX STUTTER multiplier
	#define SELFNOISETHRESH		32				//threshold for self noise routines to start
	#define MUTNOISETHRESH 		27				//threshold for mutual noise routines to start
	#define FREQUENCYCHANGES 	32			//number of frequencies to try before canceling touch
	#define MAXSAMPLESIZE 		20				//maximum number of samples to check for noise
	#define SAMPLESIZE			5				//actual number of samples to check for noise
	#define SAMPLECHANNEL 		4				//TX channel to sample to check for noise
	#define SELFNOISESCANTIME	6				//number of self scans to take when checking for noise
	#define MUTNOISESCANTIME	14				//number of mutual scans to take when checking for noise
	#define FILTERCOEFF			0				//EMA filter coefficient value (tied to both self and mutual) If zero, filters are turned off
#endif // ifdef AMTP3002DSBCVD

#ifdef AMTP3006DSBCTMU						// AMT 7" 15x24 Displayless Sensor Board CTMU
	#define CHARGEPUMP						// define if using charge pump
	#define CPDELAY 0xFF
	//#define USEMUTDIFFERENCE	// if defined take the difference of TX 1 and 0
	#define RXPINMAP {RB5RX, RB4RX, RB3RX, RB2RX, RB6RX, RB7RX, RB8RX, RB9RX, RB10RX, RB11RX, RB15RX, RB14RX, RB13RX, RB12RX, RB1RX}	// Setup the pin mapping for the RX
	#define TXPINMAP {RD11TX, RD8TX, RF3TX, RF5TX, RF4TX, RE7TX, RE6TX, RE5TX, RE4TX, RE3TX, RE2TX, RE1TX, RE0TX, RF1TX, RF0TX, RD7TX, RD6TX, RD5TX, RD4TX, RD3TX, RD2TX, RD1TX, RC14TX, RC13TX}			// Setup the pin mapping for the TX
	#define NUMBEROFRXCHANNELS 15		// Set the number of CTMU channels
	#define NUMBEROFTXCHANNELS 24		// Set the number of TX channels
	#define SELFTOUCHTHRES 37 			// Set the threshold to compare the self measurement, if a bove we have a touch
	#define BASEUPDATETIME 25			// Set the frequency of the base update.  The time is based on loop and will vary based on all these parameters
	#define MUTTOUCHTHRES 45					// threshold to compare mutual measurement to, must be above
	#define FLIPSTATE 5 				// default flip state
	#define SELFSCANTIME 12				// Set the number of self ADC measuements to SUM for 1 measuement
	#define MUTSCANTIME 20				// Set the number of mutual ADC measuements to SUM for 1 measuement
	#define	SELFDIVIDER 32				// Set the divider used for self raw measerments, left shifted 5, 32 = divide by 1
	#define MUTDIVIDER 32				// Set the divider used for mutual raw measerments, left shifted 5, 32 = divide by 1
	#define SELFDELAYTIME 1//7				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in self
	#define MUTDELAYTIME 18//24				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in mutual
	#ifndef USEMUTDIFFERENCE
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b10010000
		#else
		#define CUSTOMFLAG	0b10010101
		#endif
	#else
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b00010000
		#else
		#define CUSTOMFLAG	0b00010101
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
	#define NUMOFAVG 8					// Set the number of positions to average, note depends on the array size
	#define SELFCURRENT 1				// Set CTMU IRNG self: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define MUTCURRENT	1				// Set CTMU IRNG mutual: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
	#define RXDIAGCHANNEL 0			// Channel 0 is default diagnostic channel
	#define TXDIAGCHANNEL 0				// TX channel 0 is default diagnostic channel
	#define SELFSAMPLEFREQ 1			// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define MUTSAMPLEFREQ 1				// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define SLEEPTIME 0					// 0 disabbles, else 1 = 262.144ms, 2 = 524.288ms, etc
	#define MINCUSPDELTA 5
	#define WEIGHTTHRESHOLD 0xff
	#define MINTOUCHDISTANCE 150
	#define PENDOWNTIMER 1
	#define PENUPTIMER 3
	#define CPTIMEOUT 18				// Timeout used for chargepump delay, 1 = 256us
	#define STUTTERMULT  3				// TX STUTTER multiplier
	#define SELFNOISETHRESH		255//25				//threshold for self noise routines to start
	#define MUTNOISETHRESH 		255//20				//threshold for mutual noise routines to start
	#define FREQUENCYCHANGES 	20//32			//number of frequencies to try before canceling touch
	#define MAXSAMPLESIZE 		20				//maximum number of samples to check for noise
	#define SAMPLESIZE			5				//actual number of samples to check for noise
	#define SAMPLECHANNEL 		4				//TX channel to sample to check for noise
	#define SELFNOISESCANTIME	6				//number of self scans to take when checking for noise
	#define MUTNOISESCANTIME	8				//number of mutual scans to take when checking for noise
	#define FILTERCOEFF			0				//EMA filter coefficient value (tied to both self and mutual) If zero, filters are turned off
#endif // ifdef AMTP3006DSBCTMU

#ifdef AMTP3006DSBCVD						// AMT 7" 15x24 Displayless Sensor Board CTMU
	#define CHARGEPUMP						// define if using charge pump
	#define CPDELAY 0xFF
	#define USEMUTDIFFERENCE	// if defined take the difference of TX 1 and 0
	#define RXPINMAP {RB5RX, RB4RX, RB3RX, RB2RX, RB6RX, RB7RX, RB8RX, RB9RX, RB10RX, RB11RX, RB15RX, RB14RX, RB13RX, RB12RX, RB1RX}	// Setup the pin mapping for the RX
	#define TXPINMAP {RD11TX, RD8TX, RF3TX, RF5TX, RF4TX, RE7TX, RE6TX, RE5TX, RE4TX, RE3TX, RE2TX, RE1TX, RE0TX, RF1TX, RF0TX, RD7TX, RD6TX, RD5TX, RD4TX, RD3TX, RD2TX, RD1TX, RC14TX, RC13TX}			// Setup the pin mapping for the TX
	#define NUMBEROFRXCHANNELS 15		// Set the number of CTMU channels
	#define NUMBEROFTXCHANNELS 24		// Set the number of TX channels
	#define SELFTOUCHTHRES 35 			// Set the threshold to compare the self measurement, if a bove we have a touch
	#define BASEUPDATETIME 25			// Set the frequency of the base update.  The time is based on loop and will vary based on all these parameters
	#define MUTTOUCHTHRES 50					// threshold to compare mutual measurement to, must be above
	#define FLIPSTATE 6 				// default flip state
	#define SELFSCANTIME 3				// Set the number of self ADC measuements to SUM for 1 measuement
	#define MUTSCANTIME 6				// Set the number of mutual ADC measuements to SUM for 1 measuement
	#define	SELFDIVIDER 32				// Set the divider used for self raw measerments, left shifted 5, 32 = divide by 1
	#define MUTDIVIDER 32				// Set the divider used for mutual raw measerments, left shifted 5, 32 = divide by 1
	#define SELFDELAYTIME 1//7				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in self
	#define MUTDELAYTIME 1//24				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in mutual
	#ifndef USEMUTDIFFERENCE
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b10111010
		#else
		#define CUSTOMFLAG	0b10111111
		#endif
	#else
		#ifndef CHARGEPUMP
		#define CUSTOMFLAG	0b00111010
		#else
		#define CUSTOMFLAG	0b00110111
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
	#define RXDIAGCHANNEL 0				// Channel 0 is default diagnostic channel
	#define TXDIAGCHANNEL 0				// TX channel 0 is default diagnostic channel
	#define SELFSAMPLEFREQ 1			// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define MUTSAMPLEFREQ 1				// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
	#define SLEEPTIME 0					// 0 disabbles, else 1 = 262.144ms, 2 = 524.288ms, etc
	#define MINCUSPDELTA 5
	#define WEIGHTTHRESHOLD 0xff
	#define MINTOUCHDISTANCE 150
	#define PENDOWNTIMER 1
	#define PENUPTIMER 3
	#define CPTIMEOUT 18				// Timeout used for chargepump delay, 1 = 256us
	#define STUTTERMULT  3				// TX STUTTER multiplier
	#define SELFNOISETHRESH		30				//threshold for self noise routines to start
	#define MUTNOISETHRESH 		25				//threshold for mutual noise routines to start
	#define FREQUENCYCHANGES 	20//32			//number of frequencies to try before canceling touch
	#define MAXSAMPLESIZE 		20				//maximum number of samples to check for noise
	#define SAMPLESIZE			5				//actual number of samples to check for noise
	#define SAMPLECHANNEL 		4				//TX channel to sample to check for noise
	#define SELFNOISESCANTIME	7				//number of self scans to take when checking for noise
	#define MUTNOISESCANTIME	6				//number of mutual scans to take when checking for noise
	#define FILTERCOEFF			0				//EMA filter coefficient value (tied to both self and mutual) If zero, filters are turned off
#endif // ifdef AMTP3006DSBCVD

#endif //ifndef __SENSOR_PIC24FJ64GB106_H
