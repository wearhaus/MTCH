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

// All sensor files should have the same #define here - to make sure only a single
// sensor file is included
#ifndef __SENSOR_H
#define __SENSOR_H

//
// Projected Constants, pre-defines for dev kit sensors
//

//#define CHARGEPUMP						// define if using charge pump
#define CPDELAY 0xFF
#define USEMUTDIFFERENCE	// if defined take the difference of TX 1 and 0
#define RXPINMAP {RE2RX, RE0RX, RA5RX, RA3RX, RE1RX, RB4RX, RB5RX, RB3RX, RB0RX, RB1RX, RD7RX, RD4RX, RD5RX, RD6RX, RB2RX} // Setup the pin mapping for the RX
#define TXPINMAP {RA1TX, RA2TX, RA0TX} // Setup the pin mapping for the TX
#define NUMBEROFRXCHANNELS 15		// Set the number of CTMU channels
#define NUMBEROFTXCHANNELS 3		// Set the number of TX channels
#define SELFTOUCHTHRES 37 			// Set the threshold to compare the self measurement, if a bove we have a touch
#define BASEUPDATETIME 50			// Set the frequency of the base update.  The time is based on loop and will vary based on all these parameters
#define ACTIVATIONSUMTHRES 80 		// raw axis SUM threshold
#define MUTTHRES 45					// threshold to compare mutual measurement to, must be above
#define FLIPSTATE 1 				// default flip state
#define SELFSCANTIME 50				// Set the number of self ADC measuements to SUM for 1 measuement
#define MUTSCANTIME 15				// Set the number of mutual ADC measuements to SUM for 1 measuement
#define	SELFDIVIDER 32				// Set the divider used for self raw measerments, left shifted 5, 32 = divide by 1
#define MUTDIVIDER 32				// Set the divider used for mutual raw measerments, left shifted 5, 32 = divide by 1
#define SELFDELAYTIME 4//7				// Set the delay to wait before ADC measuement after pulsing the TX line(s) in self
#define SHCHARGETIME 2//5				// Set the delay to allow the FVR to charge up the S/H capacitor on the ADC - CVD only
#define MUTDELAYTIME 6
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
#define MUTCURRENT	2				// Set CTMU IRNG mutual: 1 ~ 0.55uA, 2 ~ 5.5uA, 3 ~ 55uA
#define FVRSELF	0b01100000			// self FVR
#define PVCFGSELF 0					// voltage reference for ADC self
#define FVRMUT	0b11100000			// mutual FVR
#define PVCFGMUT 0					// voltage reference for ADC mutual
#define RXDIAGCHANNEL 0				// Channel 0 is default diagnostic channel
#define TXDIAGCHANNEL 0				// TX channel 0 is default diagnostic channel
#define MAXMINSELFTHRES 40			// Threshold to compare the max-min used to involk AC digital filter if above
#define SELFSAMPLEFREQ 1			// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
#define SELFOVERSAMPLE 8			// Used for digital AC filter, take this many more samples before filtering
#define SELFREPLACETHRES 100		// If above involk digital filter also replace values abobe this with the averge of all samples
#define SELFLPFILTERVALUE 192		// The digital filter coeficient, 192/256 = 0.75.  Sets a value between 0 and 1
#define SELFMAXDELTA 15				// If the maximum delta in the AC digital filter is above set all values to the base ADC, no Touch
#define MAXMINMUTTHRES 40			// Threshold to compare the max-min used to involk AC digital filter if above
#define MUTSAMPLEFREQ 1				// Sets a delay between self ADC measuements, currently TMR6 is set 1 = 250nS
#define MUTOVERSAMPLE 8				// Used for digital AC filter, take this many more samples before filtering
#define MUTREPLACETHRES 100			// If above involk digital filter also replace values abobe this with the averge of all samples
#define MUTLPFILTERVALUE 192		// The digital filter coeficient, 192/256 = 0.75.  Sets a value between 0 and 1
#define MUTMAXDELTA 15				// If the maximum delta in the AC digital filter is above set all values to the base ADC, no Touch
#define ADCADCS 2					// 2 - TAD = 500nS, 6 - TAD = 1uS
#define SLEEPTIME 0					// 0 disabbles, else 1 = 262.144ms, 2 = 524.288ms, etc
#define MINCUSPDELTA 5
#define WEIGHTTHRESHOLD 0xff
#define MINTOUCHDISTANCE 150
#define PENDOWNTIMER 1
#define PENUPTIMER 3
#define CPTIMEOUT 18				// Timeout used for chargepump delay, 1 = 256us

#else
#error Multiple sensor files included, only one sensor file may be included at a time.
#endif //ifndef __SENSOR_H
