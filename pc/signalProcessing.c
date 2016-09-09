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
* AV                  03/06/12    ...
******************************************************************************/
/*
	TSCG			  03/06/12


*/

/*! \file signalProcessing.c
	\brief General Purpose communications functions.

	This file contains all functions and algorithms relating to 
	digital signal processing and noise detection/correction
*/

//
//includes
//
#include "main\main.h"

//
//globals and externs
//
extern USERRAM userRam;
extern RAWRAM rawRam;
extern BASERAM baseRam;
extern VARRAM varRam;
#ifdef ENABLE_DEBUG
extern DEBUGRAM debugRam;
#endif
extern NOISERAM selfNoise;
extern NOISERAM mutNoise;

extern void (*selfScan)(unsigned char);
extern void (*mutualScan)(unsigned char);
/******************************************************************************
* Function:        	void initNoise(void)
*
* PreCondition:    	none
*
*
* Input:            none
*
* Output:          	none
*
* Side Effects:    	None
*
* Overview:     	initialize variables and structures used in noise routines
*
* Note:            	None
*			
*****************************************************************************/
void initNoise(void)
{
	selfNoise.noisePresent = 0;
	selfNoise.timerCount = -1;
	selfNoise.triedFrequencies = 0;

	mutNoise.noisePresent = 0;
	mutNoise.timerCount = -1;
	mutNoise.triedFrequencies = 0;
}

/******************************************************************************
* Function:        	void scanNoise(unsigned char touchLocation)
*
* PreCondition:    	none
*
*
* Input:            RX channel to be scanned for noise
*
* Output:          	0 - we should cancel touch reporting due to noise
*					1 - noise has been resolved
*
* Side Effects:    	None
*
* Overview:     	wrapper function to run noise routines
*
* Note:            	must be run after performing self measurements and locating 
*					an active RX channel
*			
*****************************************************************************/
unsigned char scanNoise(unsigned char touchLocation)
{
	do{
		mutNoise.noiseFlag = 0;									//clear mutual noise flag
		if(mutCheckNoise(touchLocation))						//scan a node of activated RX channel for mutual noise
		{
			mutNoise.noiseFlag = 1;								//set noise mutual flag if noise is present
		}
		if(mutNoise.triedFrequencies > userRam.frequencyChanges)//if true, return and cancel touch 	
		{
			return 1;											
		}
	}while(mutNoise.noiseFlag==1);								//repeat until (mutual) noise is resolved
	selfCheckNoise(touchLocation);								//scan current, activated RX channel for self noise
	return 0;
}

/******************************************************************************
* Function:        	void selfCheckNoise(void)
*
* PreCondition:    	none
*
*
* Input:            none
*
* Output:          	none
*
* Side Effects:    	None
*
* Overview:     	determines if there is noise on the system and changes the
*					sample frequency to sample around it
*
* Note:            	None
*			
*****************************************************************************/
unsigned char selfCheckNoise(unsigned char channel)
{
	unsigned char x;
	unsigned char deviation = 0;
	unsigned short val;
	short meanArray[20];

	for(x=0;x<userRam.sampleSize;x++)																				
	{
		val = scanChannels(channel, channel+1, NOISE_SCAN);
		meanArray[x] = (val / userRam.selfNoiseScanTime);
	
		if(x > 0)																						
		{
			if(meanArray[x-1] > meanArray[x])
			{
				deviation += (meanArray[x-1] - meanArray[x]);
			}
			else
			{
				deviation += (meanArray[x] - meanArray[x-1]);
			}

			//no reason to continue scanning for noise if we are laready over the threshold
			if(deviation > userRam.selfNoiseThresh)														
			{
				break;
			}                                           
		}
	}																									
#ifdef ENABLE_DEBUG
	debugRam.size = 1;
	debugRam.ID = SELFNOISEDEV;
	debugRam.buffer[0] = deviation;
	sendDebugBytes();
#endif

	if(deviation > userRam.selfNoiseThresh)											//we are in a noisy state
	{
		selfNoise.timerCount = -1;													//disable timer counter
		if((selfNoise.noisePresent == 0) && (userRam.selfSampleFreq != 1))			//if entering from clean state...
		{
			userRam.selfSampleFreq = NOISE_INC_VALUE;												//reset sampling frequency to original value
		}
		else
		{
			userRam.selfSampleFreq += NOISE_INC_VALUE;							//modify sampling frequency											//mask to cycle from 0-31
			userRam.selfSampleFreq &= 0x1F;											//mask sampling frequency to loop 1-31
		}	
#ifdef ENABLE_DEBUG
		debugRam.size = 1;
		debugRam.ID = SELFNOISEFREQ;
		debugRam.buffer[0] = userRam.selfSampleFreq;
		sendDebugBytes();
#endif
		selfNoise.noisePresent = 1;													//set flag that we are in noise
		return 1;
	}
	else									
	{
		if((selfNoise.timerCount < 0) && (selfNoise.noisePresent))					//if timer isn't already running and are in noise
		{
			selfNoise.timerCount = 0;												//enable timer counter
		}
		return 0;
	}
}

/******************************************************************************
* Function:        	void mutCheckNoise(void)
*
* PreCondition:    	none
*
* Input:           	none
*
* Output:          	none
*
* Side Effects:    	None
*
* Overview:        	checks for noise on the system by comparing the summed
*					mean deviation of previous samples to a pre-determined 
*					threshold and adjusts the sampling frequency to try to 
*					sample around the noise
*
* Note:            	None
*			
*****************************************************************************/
unsigned char mutCheckNoise(unsigned char channel)
{
	unsigned char x, temp;
	unsigned char deviation = 0;
	short meanArray[20];

	//get channel to sample
	temp = (userRam.numberOfTXChannels >> 1);		//use channel in middle of sensor			

	for(x=0;x<userRam.sampleSize;x++)																
	{
		//scan for noise
		scanMutual(channel, temp, NOISE_SCAN);
		meanArray[x] = rawRam.rawMut[temp] / userRam.mutNoiseScanTime;

		if(x > 0)																						
		{
			if(meanArray[x-1] > meanArray[x])
			{
				deviation += (meanArray[x-1] - meanArray[x]);
			}
			else
			{
				deviation += (meanArray[x] - meanArray[x-1]);		
			}

			//no need to keep scanning for noise if we already exceed the noise threshold
			if(deviation > userRam.mutNoiseThresh)														
			{
				break;																					
			}	
		}
	}

 #ifdef ENABLE_DEBUG	
	debugRam.size = 1;
	debugRam.ID = MUTNOISEDEV;
	debugRam.buffer[0] = deviation;
	sendDebugBytes();	
 #endif

	if(deviation > userRam.mutNoiseThresh)											//we are in a noisy state
	{
		mutNoise.timerCount = -1;													//disable mutual timer
		if((mutNoise.noisePresent == 0) && (userRam.mutSampleFreq != 1))			//if entering noise from clean state
		{
			userRam.mutSampleFreq = NOISE_INC_VALUE;												//reset sampling frequency to original value
			mutNoise.triedFrequencies = 1;											//reset number of tried frequencies
		}
		else																		//if continuing to be in a noisy state
		{
			if(mutNoise.triedFrequencies <= userRam.frequencyChanges)				//set flag to stop sending touch data if the number of
			{																		//frequencies tried exceeds threshold	

				mutNoise.triedFrequencies++;										//increment number of frequencies tried
			}
			userRam.mutSampleFreq += NOISE_INC_VALUE;												//modify sampling frequency
			userRam.mutSampleFreq &= 0x1F;											//mask sampling frequency to loop 1-31
#ifdef ENABLE_DEBUG
			debugRam.size = 1;
			debugRam.ID = MUTNOISEFREQ;
			debugRam.buffer[0] = userRam.mutSampleFreq;
			sendDebugBytes();
#endif
		}
		mutNoise.noisePresent = 1;													//set noise indentifier
		return 1;
	}
	else																			//we are in a clean state
	{
		if((mutNoise.timerCount < 0) && (mutNoise.noisePresent))					//start noise timer if not already running
		{
			mutNoise.timerCount = 0;												
		}
		return 0;
	}
}

/******************************************************************************
* Function:        	void maFilter(unsigned char scanCount)
*
* PreCondition:    	varRam.guint3 must contain sum of current sample set 
*					rawRam.rawADC must contain each sample
*
* Input:           	none
*
* Output:          	none
*
* Side Effects:    	None
*
* Overview:        	runs a moving average filter on the current sample set
*
*					Algorithm: Y[n] = (Y[n-1]*(a-1) + Y[n]) / a
*							   where a = filter coefficient
*
* Note:            	None
*			
*****************************************************************************/
void maFilter(unsigned char scanCount)
{
	unsigned short seed;
	unsigned short *rawPtr;
	unsigned char y, coeff;

	coeff = userRam.filterCoeff;						//assign filter weight coefficient
	rawPtr = &rawRam.rawADC[0];							//get starting address for values in sample set

	seed = varRam.guint3 / scanCount;					//user sample set mean as the initial seed for the filter	
	*rawPtr = ((seed<<coeff) - seed) + *rawPtr;			//run the filter for the first sample value using the initial seed
	*rawPtr >>= coeff;
	seed = *rawPtr;										//use the filtered value as the new seed
	varRam.guint3 = seed;								//begin to re-calculate the sum
	for(y=1;y<scanCount;y++)							//loop through and filter the rest of the values in the sample set
	{
		rawPtr++;
		*rawPtr = (((seed<<coeff) - seed) + *rawPtr);
		*rawPtr >>= coeff;
		seed = *rawPtr;
		varRam.guint3 += seed;
	}
}	
