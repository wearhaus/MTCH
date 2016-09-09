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
* name                  01/26/10    ...
******************************************************************************/
/*
	TSCG				12/05/11


*/

/*! \file decode.c
	\brief Contains functions to process data from the sensor.

	This file contains the functions to process the raw data from the sensor
	and do the following:

	\li Identify potential touch locations
	\li interpolate raw data to full resolution
	\li Track locations from "frame" to "frame" to maintain IDs.
*/

#include "main\main.h"

//
// Globals
//
extern GESTURE gesture[MAX_TOUCHES];
extern NOISERAM mutNoise;
extern USERRAM userRam;
extern COMMRAM commRam;
extern RAWRAM rawRam;
extern VARRAM varRam;
#ifdef ENABLE_DEBUG
	extern DEBUGRAM debugRam;
#endif // ENABLE_DEBUG
extern TOUCHSET touchSet;
extern TOUCHIDSET touchIDSet;

/*************************************************************************//**
* Function:        	unsigned char decodeInit(void)
*
* \pre		    	None
*
* \result          	touchSet and touchIDSet will be initialized to default
*					values.
*
* \post		    	None
*
* \brief        	Initializes all decoding rated data structures.
*
* \note           	None
*
*****************************************************************************/
void decodeInit(void)
{
	unsigned char counter;
	unsigned char counter2;
	touchSet.touchCount = 0;
	touchIDSet.touchCount = 0;
	for (counter = 0; counter < MAX_TOUCHES; counter++)
	{
		touchSet.touchLoc[counter].roughLocation.x = 0;
		touchSet.touchLoc[counter].roughLocation.y = 0;
		touchSet.touchLoc[counter].fineLocation.x = 0;
		touchSet.touchLoc[counter].fineLocation.y = 0;
		touchSet.touchLoc[counter].touchFlags = 0;
		touchIDSet.tData[counter].touchState = TS_ID_AVAILABLE;
		touchIDSet.tData[counter].StateTimer = 0;
		touchIDSet.tData[counter].historySize = 0;
		for (counter2 = 0; counter2 < TOUCH_HISTORY+1; counter2++)
		{
			touchIDSet.tData[counter].touchLoc[counter2].roughLocation.x = 0;
			touchIDSet.tData[counter].touchLoc[counter2].roughLocation.y = 0;
			touchIDSet.tData[counter].touchLoc[counter2].fineLocation.x = 0;
			touchIDSet.tData[counter].touchLoc[counter2].fineLocation.y = 0;
		}
	}
}

/*************************************************************************//**
* Function:        	unsigned char findTouches(void)
*
* \pre		    	None
*
* \result          	touchSet will contain all potential touch locations
*
* \post		    	None
*
* \brief        	drives the touch decoding algorithm.  Loads potential touch
*					locations into the touchSet data structure.
*
* \note           	None
*
*****************************************************************************/
unsigned char findTouches(void)
{
    unsigned char counter;
	unsigned short maxTouchStrength = 0; //jgh unsigned char maxTouchStrength = 0;
    // If there are no touches, then do a full self scan

#ifdef ENABLE_DEBUG
	if (userRam.flag1&CONTROLLERDIAGNOSTICS)
	{
	    debugRam.ID = DIAGIMAGESTART;
	    debugRam.size = 0;
	    sendDebugBytes();
	}
#endif //ENABLE_DEBUG
	initColCache();
	touchSet.touchCount = 0;
    //if (touchSet.touchCount == 0)
    {   //
        unsigned char touchLocation = 0;
        unsigned char mutTouchLocation = 0;
        unsigned short axisValues[MAXRX];
        unsigned short mutValues[MAXTX];

#ifdef ENABLE_DEBUG
		if (userRam.flag1&CONTROLLERDIAGNOSTICS)
		{
			debugRam.ID = DIAGSELFDATA;
    	    debugRam.size = (userRam.numberOfRXChannels<<1)+1;
        	debugRam.buffer[0] = 0; // Starting self channel
		}
#endif //ENABLE_DEBUG
         for (counter = 0; counter < userRam.numberOfRXChannels; counter++)
        {
            axisValues[counter] = getNormalizedSelf(counter);
#ifdef ENABLE_DEBUG
         	if (userRam.flag1&CONTROLLERDIAGNOSTICS)
			{
				debugRam.buffer[(counter<<1)+1] = axisValues[counter]&0xff;
            	debugRam.buffer[(counter<<1)+2] = (axisValues[counter]>>8)&0xFF;
			}
#endif //ENABLE_DEBUG
        }
#ifdef ENABLE_DEBUG
		if (userRam.flag1&CONTROLLERDIAGNOSTICS)
		{
        	sendDebugBytes(); // this sends out all of the self measurements.
		}
#endif //ENABLE_DEBUG

		touchSet.touchLoc[touchSet.touchCount].strengthSum = 0;
        while (touchLocation != 0xff)
        {
            mutTouchLocation = 0;
            touchLocation = findNextPeak(axisValues, userRam.numberOfRXChannels, touchLocation, userRam.selfTouchThres);
            if (touchLocation != 0xff)
            {
				//scan for noise
				if((userRam.customFlag&RUNNOISEROUTINES) != 0x00)
				{
					if(scanNoise(touchLocation))
					{
						return 0;
					}
				}
   	         	for (counter = 0; counter < userRam.numberOfTXChannels; counter++)
                {
                    mutValues[counter] = queryColCache(touchLocation,counter,1);
            	}

				//clear any signal on channel 0 if we are pulsing 2 TX channels
            	//because it will be the same as channel 1
				if((userRam.customFlag&PULSE2TX) != 0x00)
				{
					if(mutValues[1] > mutValues[2])
					{
						mutValues[0] = (mutValues[1] - mutValues[2]);
					}
					else
					 {
					 	mutValues[0] = 0;
					 }
				}
				#ifdef ENABLE_DEBUG
   	      		for (counter = 0; counter < userRam.numberOfTXChannels; counter++)
           		{
					if (userRam.flag1&CONTROLLERDIAGNOSTICS)
					{
            		    debugRam.buffer[2+(counter<<1)] = mutValues[counter]&0xff;
            		    debugRam.buffer[3+(counter<<1)] = (mutValues[counter]>>8)&0xff;
					}
				}
				#endif //ENABLE_DEBUG

#ifdef ENABLE_DEBUG
				if (userRam.flag1&CONTROLLERDIAGNOSTICS)
				{
                	for (counter = 0; counter < userRam.numberOfTXChannels; counter++)
                	{ // This loop can NOT be combined with the one above, as queryColCache uses debugRam as well
	                    debugRam.buffer[2+(counter<<1)] = mutValues[counter]&0xff;
	                    debugRam.buffer[3+(counter<<1)] = (mutValues[counter]>>8)&0xff;
                	}
	                debugRam.ID = DIAGMUTDATA;
	                debugRam.size = 2 + (userRam.numberOfTXChannels<<1);
	                debugRam.buffer[0] = touchLocation; // X location
	                debugRam.buffer[1] = 0; // starting Y location
    	            sendDebugBytes();
				}
#endif //ENABLE_DEBUG

                while (mutTouchLocation != 0xff && touchSet.touchCount < MAX_TOUCHES)
                {
                    mutTouchLocation = findNextPeak(mutValues, userRam.numberOfTXChannels, mutTouchLocation, userRam.mutTouchThres);
                    if (mutTouchLocation != 0xff)
                    {
						unsigned char duplicateLocation = 0;
                        touchSet.touchLoc[touchSet.touchCount].roughLocation.x = touchLocation;
                        touchSet.touchLoc[touchSet.touchCount].roughLocation.y = mutTouchLocation;
						touchSet.touchLoc[touchSet.touchCount].fineLocation.x = 0;
						touchSet.touchLoc[touchSet.touchCount].fineLocation.y = 0;
						touchSet.touchLoc[touchSet.touchCount].touchFlags = 0;
						touchSet.touchLoc[touchSet.touchCount].strengthSum = 0;
						nudgeLoc(&touchSet.touchLoc[touchSet.touchCount]);

						// now that it is nudged, check to see if it overlaps any previous nodes
						// if it does, ignore/eliminate it
						for (counter = 0; counter < touchSet.touchCount; counter++)
						{
							if (touchSet.touchLoc[counter].roughLocation.x == touchSet.touchLoc[touchSet.touchCount].roughLocation.x
								&& touchSet.touchLoc[counter].roughLocation.y == touchSet.touchLoc[touchSet.touchCount].roughLocation.y)
							{ // rough locations match
								duplicateLocation = 1;
								touchSet.touchLoc[counter].strengthSum += touchSet.touchLoc[touchSet.touchCount].strengthSum; // add signal level to strength sum of point nudged into
								// Check to see how large the touch is (based upon strengthSum)
								if (touchSet.touchLoc[counter].strengthSum > ((touchSet.touchLoc[counter].touchStrength) + (touchSet.touchLoc[counter].touchStrength>>1)))
								{
									touchSet.touchLoc[counter].touchFlags |= TF_LARGEACTIVATION;
								}
								if (touchSet.touchLoc[counter].strengthSum > ((touchSet.touchLoc[counter].touchStrength<<1) + (touchSet.touchLoc[counter].touchStrength>>1)))
								{
									touchSet.touchLoc[counter].touchFlags |= TF_EXTRALARGEACTIVATION;
								}
							}
						}

						if (!duplicateLocation)
						{
	                        findFineLocation(&touchSet.touchLoc[touchSet.touchCount]);
							if (touchSet.touchLoc[touchSet.touchCount].touchStrength > maxTouchStrength)
							{
								maxTouchStrength = touchSet.touchLoc[touchSet.touchCount].touchStrength;
							}
	                        //
	                    	// send out diagnostics
	                    	//
#ifdef ENABLE_DEBUG
							if (userRam.flag1&CONTROLLERDIAGNOSTICS)
							{
	                            //unsigned char x0;
	                    		// (Total 8 bytes each touch)
	                    		// flag FINDTOUCHES
	                            // Touch Number
	                            // Rough Location 0
	                            // Rough Location 1
	                            // Fine Location [0] low
	                            // Fine Location [0] High
	                            // Fine Location [1] low
	                            // Fine Location [1] High
	                    		//
	                            debugRam.ID = FINDTOUCHES;
	                            debugRam.size = 7;
	                            debugRam.buffer[0] = touchSet.touchCount;
	                            debugRam.buffer[1] = touchSet.touchLoc[touchSet.touchCount].roughLocation.x;
	                            debugRam.buffer[2] = touchSet.touchLoc[touchSet.touchCount].roughLocation.y;
	                            debugRam.buffer[3] = touchSet.touchLoc[touchSet.touchCount].fineLocation.x&0xff;
	                            debugRam.buffer[4] = (touchSet.touchLoc[touchSet.touchCount].fineLocation.x>>8)&0xff;
	                            debugRam.buffer[5] = touchSet.touchLoc[touchSet.touchCount].fineLocation.y&0xff;
	                            debugRam.buffer[6] = (touchSet.touchLoc[touchSet.touchCount].fineLocation.y>>8)&0xff;
	                            sendDebugBytes();
	                    	}
#endif //ENABLE_DEBUG
	                        touchSet.touchCount++;
						}
	                    mutTouchLocation++;
                    }
                }
                touchLocation++;
            }
        }
     //   touchSet.touchCount = 0;
    }
    /*else
    {   // first, we want to scan where there are touches,
        // then check for new touches if we can handle them.
    }*/

	// remove any touches that are below 50% of the maximum touch
	maxTouchStrength >>= 1;
	for (counter = 0; counter < touchSet.touchCount; counter++)
	{
		if (touchSet.touchLoc[counter].touchStrength < maxTouchStrength)
		{ // bad touch, remove it
			cancelTouch(counter);
		}
	}

	// if we have more than one activation check to see if any of them are large activations
	// if they are, find the distance between them and the other activations and cancel them
	// if they are too close.  They probably appeared from the same large activation.
	// Note that there are two levels of large activatons and thus two distance thresholds.
	// Two levels of large activations helepd to avoid canceling the two close activations scenario.
	if (touchSet.touchCount > 1)
	{
		for (counter = 0; counter < touchSet.touchCount; counter++)
		{
			if (touchSet.touchLoc[counter].touchFlags&TF_MASK_LARGEACTIVATION)
			{
				unsigned char i;
				for (i = 0; i < touchSet.touchCount; i++)
				{
					if (i != counter)
					{
						unsigned short tDistance;
						tDistance = shortDistance(touchSet.touchLoc[counter].fineLocation,touchSet.touchLoc[i].fineLocation);
						if (touchSet.touchLoc[counter].touchFlags&TF_EXTRALARGEACTIVATION)
						{
							if (tDistance < userRam.largeActThres)
							{	// this activation is too close to the other activation
								cancelTouch(i);
							}
						}
						else
						{
							if (tDistance < userRam.largeActThres>>1)
							{
								cancelTouch(i);
							}
						}
					}
				}
			}
		}
	}

#ifdef ENABLE_DEBUG
	if (userRam.flag1&CONTROLLERDIAGNOSTICS)
	{
	    debugRam.ID = DIAGIMAGEEND;
	    debugRam.size = 0;
	    sendDebugBytes();
	}
#endif //ENABLE_DEBUG
	return touchSet.touchCount;
}

/*************************************************************************//**
* Function:        	void cancelTouch(unsigned char index)
*
* \pre		    	index = the index of the touch to be canceled
*
* \result          	none
*
* \post		    	None
*
* \brief        	Shifts the touchSet.touchLoc array up to cancel out the
*					touch at the specified index
*
* \note           	none
*
*****************************************************************************/
void cancelTouch(unsigned char index)
{
	unsigned char x;
	for(x=index;x<touchSet.touchCount;x++)
	{
		touchSet.touchLoc[x] = touchSet.touchLoc[x+1];
	}
	touchSet.touchCount--;
}

/*************************************************************************//**
* Function:        	unsigned char findNextPeak(unsigned int* values, unsigned char numValues, unsigned char startLocation, unsigned char threshold)
*
* \pre		    	values contains an array of numValues values to analyze,
*					starting with startLocation.
*
* \result          	returns next potential "peak" in the values array.  If
*					no further peak is found, returns 0xff.
*
* \post		    	None
*
* \brief        	Analyzes the data in 'values' and attempts to identify
*					peaks or 'cusps' in the data.  Will flag significant
*					changes in slope or a change from a positive to a negative
*					slope.
*
* \note           	Initially only flagged positive to negative slope changes,
*					but several scenarios with the self measurements were
*					identified where that would only find a single touch when
*					two touches were actually present on the sensor.
*
*****************************************************************************/
unsigned char findNextPeak(unsigned short* values, unsigned char numValues, unsigned char startLocation, unsigned char threshold)
{
	// new implementation, looking more closely at slope...
	// need to track "left" and "right" slope.
	// compare the ratio of the slopes.   We're looking for a "greater" slope, followed by a "lesser" slope

	unsigned char currentValue;
	unsigned char leftSlopeSign = 1; // 1 = positive, 0 = negative
	unsigned char leftSlope = 0;
	unsigned char rightSlopeSign = 1; // 1 = positive, 0 = negative
	unsigned char rightSlope = 0;


	if (startLocation > numValues)
	{ // this is an error condition, just return "no peak found"
		return 0xff;
	}

	if (startLocation > 0)
	{
		if (values[startLocation] > values[startLocation-1])
		{ // left slope is positive
			leftSlopeSign = 1;
			leftSlope = values[startLocation] - values[startLocation-1];
		}
		else
		{ // left slope is negative
			leftSlopeSign = 0;
			leftSlope = values[startLocation-1] - values[startLocation];
		}
	}

	for (currentValue = startLocation; currentValue < numValues; currentValue++)
	{
		if (currentValue < numValues)
		{
			if (values[currentValue+1] > values[currentValue])
			{ // right slope is positive
				rightSlopeSign = 1;
				rightSlope = values[currentValue+1] - values[currentValue];
			}
			else
			{ // right slope is negative
				rightSlopeSign = 0;
				rightSlope = values[currentValue] - values[currentValue+1];
			}
		}
		else
		{
			rightSlope = 0;
		}

		if (values[currentValue] > threshold)
		{ // Value is over our theshold, examine it further for a "peak"
			if (0 == rightSlopeSign || leftSlopeSign == rightSlopeSign)
			{ // We don't want a negative to positive cusp (left negative, right positive),
			  // but we do want to look at negative-negative, positive-negative, and
			  // positive-positive relationships.
				if (rightSlopeSign)
				{ // positive right slope
					if ((leftSlope<<1) > rightSlope)
					{ // we have a significant enough slope change, flag it.
						return currentValue;
					}
				}
				else if (leftSlopeSign)
				{ // we have a "cusp"
					if (leftSlope > userRam.minCuspDelta || rightSlope > userRam.minCuspDelta)
					{ // require a minimum threshold for the slopes.
						return currentValue;
					}
				}
				else
				{
					if (rightSlope<<1 > leftSlope)
					{ // we have a signifiant enough slope change, flag it.
						return currentValue;
					}
				}
			}
		}


		// we're shifting to the right one, so the left slope becomes the right slope
		// we'll calculate a new right slope at the beginning of the loop.
		leftSlopeSign = rightSlopeSign;
		leftSlope = rightSlope;
	}
	return 0xff;
}

/*************************************************************************//**
* Function:        	unsigned char maxValue(unsigned short Values[], unsigned char numValues)
*
* \pre		    	Values[] contains more than one value to examine
*					numValues contains the number of values in Values[]
*
* \result          	Returns which index of Values has the largest values.
*
* \post		    	None
*
* \brief        	Returns the largest value within the given array.
*
* \note
*
*****************************************************************************/
unsigned char maxValue(unsigned short Values[5], unsigned char numValues)
{
	unsigned char i = 0;
	unsigned char maxLoc = 0;
	for (i = 1; i < numValues; i++)
	{
		if (Values[i] > Values[maxLoc])
		{
			maxLoc = i;
		}
	}
	return maxLoc;
}

/*************************************************************************//**
* Function:        	unsigned char nudgeLoc(TOUCHDATA *tData)
*
* \pre		    	tData points to a TOUCHDATA structure that findNextPeak
*					has identified as a potential touch location.
*
* \result          	Modifies tData node location to be the local maximum.
*
* \post		    	None
*
* \brief        	Analyzes selected node and adjacent nodes to identify
*					where the "true" peak is.  If current node is not the
*					highest point, it "nudges" that point to the higher
*					location, then re-analyzes.
*
* \note
*
*****************************************************************************/
unsigned char nudgeLoc(TOUCHDATA *tData)
{
	unsigned char wasNudged = 0;
	unsigned char i = 0;
	unsigned char needNudge = 1;
	UCHARCOORD tempLoc;
	UCHARCOORD startLoc; // Primarily used for diagnostic output
	unsigned short startValue; // Primarily used for diagnostic output
	unsigned short valueSet[3];
	unsigned char nudgeSet = 0;

	// set up the nudge pairs (sets)
	//
	//       5
	//     3 1 4
	//   6 2 0 2 6
	//     3 1 4
	//       5
	//
	// We look at the center, and a pair of other nodes
	// if one of those nodes is higher, mover there and re-start
	// the algorithm.
	// if values are "close enough" to the center, then explore
	// further out.
	//
	// Data structure is laid out as follows:
	// NUDGESETS (6) - this is the sets of nodes to examine, in order
	// NODES (3) - Center, then other two nodes to examine.  Center always first.
	// 4 values:
	//		[0] - X shift (from center)
	//		[1] - Y shift (from center)
	//		[2] - "permission" required in "valueGood" data structure
	//				 - If valueGood[] is 1, perform the scan, otherwise result is "0"

	signed char nudgeSets[6][3][3] = 	{{{0,0,0},{0,-1,0},{0,1,0}},
										 {{0,0,0},{-1,0,0},{1,0,0}},
										 {{0,0,0},{-1,-1,8},{-1,1,5}},
										 {{0,0,0},{1,-1,7},{1,1,6}},
										 {{0,0,0},{0,-2,2},{0,2,1}},
										 {{0,0,0},{-2,0,3},{2,0,4}}};

	// Is the value greater than 75% of the center?
	//
	//   A B C         Index: 5 1 6
	//   D o E                3 0 4
	//   F G H                8 2 7

	// valueGood[0] will be "always yes" for the center
	// valueGood[1] will be location "B" > 75% of center
	// valueGood[2] will be location "G" > 75% of center
 	// valueGood[3] will be location "D" > 75% of center
	// valueGood[4] will be location "E" > 75% of center
	// valueGood[5] will be locations "D" AND "B" > 75% of center
	// valueGood[6] will be locations "B" AND "E" > 75% of center
	// valueGood[7] will be locations "E" AND "G" > 75% of center
	// valueGood[8] will be locations "G" AND "D" > 75% of center
	unsigned char valueGood[9];

	startLoc = tData->roughLocation;
	startValue = queryColCache(startLoc.x,startLoc.y,1);
	tData->strengthSum = startValue;

	while (needNudge)
	{ // keep repeating until we don't nudge anymore.

		// clear the "valueGood" data structure
		for (i = 1; i < 9; ++i)
		{
			valueGood[i] = 0;
		}
		valueGood[0] = 1;
		nudgeSet = 0;
		needNudge = 0;
		while (needNudge == 0 && nudgeSet < 6)
		{
			// First check for the "local" up and down nudges (NudgeSet 1)
			// now check for the "local" left and right nudges (NudgeSet 2)
			// if still not nudging, check the diagonals (NudgeSet 3)
			// if still not nudged and the direct adjacent values
			// are big enough (75%), then check two away. (NudgeSet 4)
			if (nudgeSet == 2)
			{ // set up the rest of the "valueGood" array
				valueGood[5] = valueGood[3]&valueGood[1];
				valueGood[6] = valueGood[1]&valueGood[4];
				valueGood[7] = valueGood[4]&valueGood[2];
				valueGood[8] = valueGood[2]&valueGood[3];
			}
			for (i = 0; i < 3; i++)
			{
				tempLoc = tData->roughLocation;
				tempLoc.x += nudgeSets[nudgeSet][i][0];
				tempLoc.y += nudgeSets[nudgeSet][i][1];

				// We will use the "trick" that tempLoc.x and tempLoc.y are unsigned
				// thus a number "less than" 0 will wrap around.  The check to make sure
				// that the values are <= max channels will check both the edges.
				valueSet[i] = 0;
				if (valueGood[nudgeSets[nudgeSet][i][2]])
				{
					if (tempLoc.x < userRam.numberOfRXChannels && tempLoc.y < userRam.numberOfTXChannels)
					{
						valueSet[i] = queryColCache(tempLoc.x,tempLoc.y,1);
					}
				}
				if (nudgeSet < 2 && i > 0)
				{
					unsigned short tempValue = valueSet[0];
					tempValue -= tempValue>>2;
					if (valueSet[i] > tempValue)
					{
						valueGood[(nudgeSet<<1)+i] = 1;
					}
				}
			}
			needNudge = maxValue(valueSet,3);
			if (needNudge > 0)
			{
				tData->roughLocation.x += nudgeSets[nudgeSet][needNudge][0];
				tData->roughLocation.y += nudgeSets[nudgeSet][needNudge][1];
				tData->touchStrength = queryColCache(tData->roughLocation.x,tData->roughLocation.y,1);
				wasNudged = 1;
#ifdef ENABLE_DEBUG
#ifdef DIAGNUDGE_ENABLE
				if (userRam.flag1&CONTROLLERDIAGNOSTICS)
				{
				    debugRam.ID = DIAGNUDGE;
				    debugRam.size = 8;
				    debugRam.buffer[0] = startLoc.x;
				    debugRam.buffer[1] = startLoc.y;
					debugRam.buffer[2] = (startValue>>8)&0xff;
					debugRam.buffer[3] = startValue&0xff;
				    debugRam.buffer[4] = tData->roughLocation.x;
				    debugRam.buffer[5] = tData->roughLocation.y;
					debugRam.buffer[6] = (tData->touchStrength>>8)&0xff;
					debugRam.buffer[7] = tData->touchStrength&0xff;
				    sendDebugBytes();
				}
#endif
#endif //ENABLE_DEBUG
			}
			nudgeSet++;
		}
	}
	tData->touchStrength = queryColCache(tData->roughLocation.x,tData->roughLocation.y,1);
	return wasNudged;
}

/*************************************************************************//**
* Function:        	unsigned char findFineLocation(TOUCHDATA *tData)
*
* \pre		    	tData must point to a valid TOUCHDATA structure that
*					defines a peak.
*
* \result          	populates the "fineLocation" members of the tData data
*					structure.
*
* \post		    	None
*
* \brief        	Analyzes the mutual measurements of the points adjacent
*					to the peak point to interpolate a higher resolution
*					location.
*
*					Equuation used: Position = CenterPosition +/- 64-((64*(center - max(left, right)))/(center-min(left, right)))
*
* \note
*
*****************************************************************************/
 unsigned char findFineLocation(TOUCHDATA *tData)
{
    UCHARCOORD location;
    unsigned char xDistance, yDistance;
    unsigned short fineMeasurements[5];		//[0] = center
											//[1] = left
											//[2] = top
											//[3] = right
											//[4] = bottom

	//clear all node measurements
	fineMeasurements[0] = 0;
	fineMeasurements[1] = 0;
	fineMeasurements[2] = 0;
	fineMeasurements[3] = 0;
	fineMeasurements[4] = 0;

	//get current activated node
    location.x = tData->roughLocation.x;
    location.y = tData->roughLocation.y;

    //get signal strength of current activated node
	fineMeasurements[0] = queryColCache(location.x,location.y,0);
	tData->touchStrength = fineMeasurements[0];

	//get signal strength of node to the right
	if(location.x != 0)
	{
		fineMeasurements[3] = queryColCache(location.x-1, location.y, 0);
	}

	//get signal strength of node to the left
	if(location.x != (userRam.numberOfRXChannels-1))
	{
		fineMeasurements[1] = queryColCache(location.x+1, location.y, 0);
	}


	//if pulsing 2 TX, we need to handle channels 0 and 1 differently
	if(((userRam.customFlag&PULSE2TX) != 0x00) && (location.y < 2))
	{
		//if channel 0, node above should be channel 2
		if(location.y == 0)
		{
			fineMeasurements[2] = queryColCache(location.x, location.y+2, 0);
		}
		else
		{
			fineMeasurements[2] = queryColCache(location.x, location.y+1, 0);
		}

		//node below is always (center - above) or zero
		if(fineMeasurements[0] > fineMeasurements[2])
		{
			fineMeasurements[4] = fineMeasurements[0] - fineMeasurements[2];
		}
		else
		{
			fineMeasurements[4] = 0;
		}
	}
	else
	{
		//get signal strength of node below
		if(location.y != 0)
		{
			fineMeasurements[4] = queryColCache(location.x, location.y-1, 0);
		}

		//get signal strength of node above
		if(location.y != (userRam.numberOfTXChannels-1))
		{
			fineMeasurements[2] = queryColCache(location.x, location.y+1, 0);
		}
	}

	if(location.x == 0)
		{
			fineMeasurements[3] = 0;
			if (fineMeasurements[0] > fineMeasurements[1])
			{
				fineMeasurements[3] = fineMeasurements[0] - fineMeasurements[1];
			}
		}

	if(location.x == (userRam.numberOfRXChannels-1))
		{
			fineMeasurements[1] = 0;
			if (fineMeasurements[0] > fineMeasurements[3])
			{
				fineMeasurements[1] = fineMeasurements[0] - fineMeasurements[3];
			}
		}

	if(location.y == 0)
		{
			fineMeasurements[4] = 0;
			if (fineMeasurements[0] > fineMeasurements[2])
			{
				fineMeasurements[4] = fineMeasurements[0] - fineMeasurements[2];
			}
		}

	if(location.y == (userRam.numberOfTXChannels-1))
		{
			fineMeasurements[2] = 0;
			if (fineMeasurements[0] > fineMeasurements[4])
			{
				fineMeasurements[2] = fineMeasurements[0] - fineMeasurements[4];
			}
		}


	//check for div 0 situation, correct by adding small offset to center. 
	//there is one assumption being made when this function is called - that the "peak" (fineMeasurements[0]) is in 
	//fact the LARGEST of the 3 electrodes on each axis. if this is NOT true, there is still the chance for a div/0. 	
	if(fineMeasurements[2] == fineMeasurements[4])
	{
		if(fineMeasurements[2] == fineMeasurements[0])
		{
			fineMeasurements[0]++;
		}
	}
	if(fineMeasurements[1] == fineMeasurements[3])
	{
		if(fineMeasurements[3] == fineMeasurements[0])
		{
			fineMeasurements[0]++;
		}
	}





	//transform rough x-location to 10-bit resolution
	tData->fineLocation.value[0] = tData->roughLocation.value[0];
	tData->fineLocation.value[0] <<= 7;
	tData->fineLocation.value[0] += 64;

	//if pulsing 2 TX and active node is at TX0 or TX1, set initial y-location to halfway between TX0 and TX1
	if(((userRam.customFlag&PULSE2TX) != 0x00) && (location.y < 2))
	{
		tData->fineLocation.value[1] = 128;
	}
	else
	{
		//resolve initial y-location to 10-bit data
		tData->fineLocation.value[1] = tData->roughLocation.value[1];
		tData->fineLocation.value[1] <<= 7;
		tData->fineLocation.value[1] += 64;
	}

	//left of center
	if(fineMeasurements[1] >= fineMeasurements[3])
	{
		//above center
		if(fineMeasurements[2] >= fineMeasurements[4])
		{
			xDistance = 64 - (((fineMeasurements[0]-fineMeasurements[1])<<6)/(fineMeasurements[0]-fineMeasurements[3]));
			tData->fineLocation.value[0] += xDistance;
			//interpolate with margain of 128 if pulsing 2 TX and at TX0 or TX1
			if(((userRam.customFlag&PULSE2TX) != 0x00) && (location.y < 2))
			{
				yDistance = 128 - (((fineMeasurements[0]-fineMeasurements[2])<<7)/(fineMeasurements[0]-fineMeasurements[4]));
			}
			else
			{
				yDistance = 64 - (((fineMeasurements[0]-fineMeasurements[2])<<6)/(fineMeasurements[0]-fineMeasurements[4]));
			}
			tData->fineLocation.value[1] += yDistance;
		}
		//below center
		else
		{
			xDistance = 64 - (((fineMeasurements[0]-fineMeasurements[1])<<6)/(fineMeasurements[0]-fineMeasurements[3]));
			tData->fineLocation.value[0] += xDistance;
			if(((userRam.customFlag&PULSE2TX) != 0x00) && (location.y < 2))
			{
				yDistance = 128 - (((fineMeasurements[0]-fineMeasurements[4])<<7)/(fineMeasurements[0]-fineMeasurements[2]));
			}
			else
			{
				yDistance = 64 - (((fineMeasurements[0]-fineMeasurements[4])<<6)/(fineMeasurements[0]-fineMeasurements[2]));
			}
			if(yDistance < tData->fineLocation.value[1])
			{
				tData->fineLocation.value[1] -= yDistance;
			}
			else
			{
				tData->fineLocation.value[1] = 0;
			}
		}
	}
	//right of center
	else
	{
		//above center
		if(fineMeasurements[2] >= fineMeasurements[4])
		{
			xDistance = 64 - (((fineMeasurements[0]-fineMeasurements[3])<<6)/(fineMeasurements[0]-fineMeasurements[1]));
			if (xDistance < tData->fineLocation.value[0])
			{
				tData->fineLocation.value[0] -= xDistance;
			}
			else
			{
				tData->fineLocation.value[0] = 0;
			}
			if(((userRam.customFlag&PULSE2TX) != 0x00) && (location.y < 2))
			{
				yDistance = 128 - (((fineMeasurements[0]-fineMeasurements[2])<<7)/(fineMeasurements[0]-fineMeasurements[4]));
			}
			else
			{
				yDistance = 64 - (((fineMeasurements[0]-fineMeasurements[2])<<6)/(fineMeasurements[0]-fineMeasurements[4]));
			}
			tData->fineLocation.value[1] += yDistance;
		}
		//below center
		else
		{
			xDistance = 64 - (((fineMeasurements[0]-fineMeasurements[3])<<6)/(fineMeasurements[0]-fineMeasurements[1]));
			if (xDistance < tData->fineLocation.value[0])
			{
				tData->fineLocation.value[0] -= xDistance;
			}
			else
			{
				tData->fineLocation.value[0] = 0;
			}
			if(((userRam.customFlag&PULSE2TX) != 0x00) && (location.y < 2))
			{
				yDistance = 128 - (((fineMeasurements[0]-fineMeasurements[4])<<7)/(fineMeasurements[0]-fineMeasurements[2]));
			}
			else
			{
				yDistance = 64 - (((fineMeasurements[0]-fineMeasurements[4])<<6)/(fineMeasurements[0]-fineMeasurements[2]));
			}
			if(yDistance < tData->fineLocation.value[1])
			{
				tData->fineLocation.value[1] -= yDistance;
			}
			else
			{
				tData->fineLocation.value[1] = 0;
			}
		}
	}

#ifdef ENABLE_DEBUG
        if (userRam.flag1&CONTROLLERDIAGNOSTICS)
		{
			debugRam.ID = DIAGFINEX;
	        debugRam.size = 6;
	        debugRam.buffer[0] = (fineMeasurements[1]>>8)&0xff;
	        debugRam.buffer[1] = fineMeasurements[1]&0xff;
	        debugRam.buffer[2] = (fineMeasurements[0]>>8)&0xff;
	        debugRam.buffer[3] = fineMeasurements[0]&0xff;
	        debugRam.buffer[4] = (fineMeasurements[3]>>8)&0xff;
	        debugRam.buffer[5] = fineMeasurements[3]&0xff;
	        sendDebugBytes();
			debugRam.ID = DIAGFINEY;
	        debugRam.size = 6;
	        debugRam.buffer[0] = (fineMeasurements[2]>>8)&0xff;
	        debugRam.buffer[1] = fineMeasurements[2]&0xff;
	        debugRam.buffer[2] = (fineMeasurements[0]>>8)&0xff;
	        debugRam.buffer[3] = fineMeasurements[0]&0xff;
	        debugRam.buffer[4] = (fineMeasurements[4]>>8)&0xff;
	        debugRam.buffer[5] = fineMeasurements[4]&0xff;
	        sendDebugBytes();
		}
#endif //ENABLE_DEBUG
    return 0;
}


/*************************************************************************//**
* Function:        	unsigned char associateTouches(void)
*
* \pre		    	touchSet, touchIDSet have been initialized by decodeInit,
*					for full functionality touchSet should have valid touch data
*
* \result          	Associates touches from touchSet with previous touches from
*					touchIDSet.
*
* \post		    	None
*
* \brief        	Associates touches in touchSet with touch IDs in touchIDset.
*					This is done by looking at a "matrix" of touch weights,
*					which are determined by using the 'touchWeight' function.
*					The "best" match for all touches will attempt to be made,
*					including using a slightly worse weight for one touch to
*					allow for a better weight for another touch.
*
* \note
*
*****************************************************************************/
unsigned char associateTouches(void)
{
	/* Here is the order of operations:

1.	Find the weights for every touch in relationship to every ID, store in a 2D array (ROWS = Touches, COLS = IDS)
2.  Find the minimum weight for every TOUCH (row)
3.  Subtract minimum weight from all weights for each TOUCH (should have at least one zero per ROW)
4.  Count number of ZEROs per Column.  If every column has one or fewer ZEROs, those are the right touches
	to match to the IDs. DONE.
5.	If more than one ZERO in a COLUMN, find ROW that has next lowest number.  Change ZERO in that row to 255.
6.	GOTO Step 2.
*/
	unsigned char currentID, currentTouch, counter;
	unsigned char maxIterations = 10;
	unsigned char badColumns = MAX_TOUCHES;

	for (currentID = 0; currentID < MAX_TOUCHES+1; currentID++)
	{
		for (currentTouch = 0; currentTouch < MAX_TOUCHES+1; currentTouch++)
		{
			touchIDSet.weights[currentTouch][currentID] = 0xff;
		}
		touchIDSet.weights[MAX_TOUCHES][currentID] = 0; // zero touch count
	}

#ifdef ENABLE_DEBUG
#ifdef DIAGASSOCIATETOUCH_ENABLE
	if (userRam.flag1&CONTROLLERDIAGNOSTICS)
	{
		if ((touchSet.touchCount > 0) || (touchIDSet.touchCount > 0))
		{
			debugRam.ID = DIAGASSOCIATETOUCH;
		    debugRam.size = 3;
		    debugRam.buffer[0] = DIAGAT_INTRO;
			debugRam.buffer[1] = touchSet.touchCount;
			debugRam.buffer[2] = touchIDSet.touchCount;
		    sendDebugBytes();


			for (counter = 0; counter < touchSet.touchCount; counter++)
			{
				debugRam.ID = DIAGASSOCIATETOUCH;
			    debugRam.size = 9;
			    debugRam.buffer[0] = DIAGAT_TOUCHSETINFO;
				debugRam.buffer[1] = counter;
				debugRam.buffer[2] = touchSet.touchLoc[counter].roughLocation.x;
				debugRam.buffer[3] = touchSet.touchLoc[counter].roughLocation.y;
				debugRam.buffer[4] = touchSet.touchLoc[counter].fineLocation.x&0xff;
				debugRam.buffer[5] = (touchSet.touchLoc[counter].fineLocation.x>>8)&0xff;
				debugRam.buffer[6] = touchSet.touchLoc[counter].fineLocation.y&0xff;
				debugRam.buffer[7] = (touchSet.touchLoc[counter].fineLocation.y>>8)&0xff;
				debugRam.buffer[8] = touchSet.touchLoc[counter].touchStrength;
			    sendDebugBytes();
			}

			for (counter = 0; counter < MAX_TOUCHES; counter++)
			{
				if (touchIDSet.tData[counter].touchState != TS_ID_AVAILABLE)
				{
					debugRam.ID = DIAGASSOCIATETOUCH;
				    debugRam.size = 10;
				    debugRam.buffer[0] = DIAGAT_TOUCHIDSETINFO;
					debugRam.buffer[1] = counter;
					debugRam.buffer[2] = touchIDSet.tData[counter].touchState;
					debugRam.buffer[3] = touchIDSet.tData[counter].StateTimer;
					debugRam.buffer[4] = touchIDSet.tData[counter].touchLoc[0].roughLocation.x;
					debugRam.buffer[5] = touchIDSet.tData[counter].touchLoc[0].roughLocation.y;
					debugRam.buffer[6] = touchIDSet.tData[counter].touchLoc[0].fineLocation.x&0xff;
					debugRam.buffer[7] = (touchIDSet.tData[counter].touchLoc[0].fineLocation.x>>8)&0xff;
					debugRam.buffer[8] = touchIDSet.tData[counter].touchLoc[0].fineLocation.y&0xff;
					debugRam.buffer[9] = (touchIDSet.tData[counter].touchLoc[0].fineLocation.y>>8)&0xff;
				    sendDebugBytes();
				}
			}
		}
	}

#endif
#endif //ENABLE_DEBUG

	// Step 1, find the weights for every touch in relationship to every ID.
	for (currentID = 0; currentID < MAX_TOUCHES; currentID++)
	{
		if (touchIDSet.tData[currentID].touchState != TS_ID_AVAILABLE
 			&& touchIDSet.tData[currentID].touchState != TS_TOUCH_UP)
		{ // This is an ID we want to look at.
			for (currentTouch = 0; currentTouch < touchSet.touchCount; currentTouch++)
			{
				touchIDSet.weights[currentTouch][currentID] = touchWeight(currentID, currentTouch);


#ifdef ENABLE_DEBUG
#ifdef DIAGASSOCIATETOUCH_ENABLE
				if (userRam.flag1&CONTROLLERDIAGNOSTICS)
				{
					debugRam.ID = DIAGASSOCIATETOUCH;
				    debugRam.size = 5;
				    debugRam.buffer[0] = DIAGAT_TOUCHWEIGHT;
					debugRam.buffer[1] = currentID; // Which ID we are looking at
					debugRam.buffer[2] = touchIDSet.tData[currentID].touchState;
					debugRam.buffer[3] = currentTouch; // Which touch we are trying to match to an ID
					debugRam.buffer[4] = touchIDSet.weights[currentTouch][currentID];
			    	sendDebugBytes();
				}
#endif
#endif //ENABLE_DEBUG


				if (touchIDSet.weights[currentTouch][currentID] < touchIDSet.weights[currentTouch][MAX_TOUCHES])
				{// Step 2, find the minimum weight for every Touch, store them in touchIDSet.weights[currentTouch][MAX_TOUCHES];
					touchIDSet.weights[currentTouch][MAX_TOUCHES] = touchIDSet.weights[currentTouch][currentID];
				}
			}
		}
	}

	if (/*touchSet.touchCount >=3 || */touchIDSet.touchCount >= 3)
	{
		touchIDSet.weights[MAX_TOUCHES][MAX_TOUCHES] = 0;
	}

	// Step 3 - subtract minimums
	for (currentID = 0; currentID < MAX_TOUCHES; currentID++)
	{
		if (touchIDSet.tData[currentID].touchState != TS_ID_AVAILABLE
 			&& touchIDSet.tData[currentID].touchState != TS_TOUCH_UP)
		{ // This is an ID we want to look at.
			for (currentTouch = 0; currentTouch < touchSet.touchCount; currentTouch++)
			{// Step 3, Subtract minimum weight from all weights for each touch
				if (touchIDSet.weights[currentTouch][currentID] < userRam.weightThreshold)//0xff) // parameterize this!
				{
					touchIDSet.weights[currentTouch][currentID] -= touchIDSet.weights[currentTouch][MAX_TOUCHES];
				}
			}
		}
	}

	while ((badColumns > 0) && (maxIterations-- > 0))
	{

#ifdef ENABLE_DEBUG
#ifdef DIAGASSOCIATETOUCH_ENABLE
		if (userRam.flag1&CONTROLLERDIAGNOSTICS)
		{
			debugRam.ID = DIAGASSOCIATETOUCH;
		    debugRam.size = 3;
		    debugRam.buffer[0] = DIAGAT_TOUCHMATRIX;
			debugRam.buffer[1] = touchSet.touchCount;
			debugRam.buffer[2] = touchIDSet.touchCount;
			counter = 3;
			if (touchSet.touchCount > 0 || touchIDSet.touchCount > 0)
			{
				for (currentID = 0; currentID < MAX_TOUCHES; currentID++)
				{
					if (touchIDSet.tData[currentID].touchState != TS_ID_AVAILABLE
			 			&& touchIDSet.tData[currentID].touchState != TS_TOUCH_UP)
					{
						debugRam.size += touchSet.touchCount+2;
						debugRam.buffer[counter++] = currentID;
						for (currentTouch = 0; currentTouch < touchSet.touchCount; currentTouch++)
						{
							debugRam.buffer[counter++] = touchIDSet.weights[currentTouch][currentID];
						}
						debugRam.buffer[counter++] = touchIDSet.weights[MAX_TOUCHES][currentID];
					}
				}
				debugRam.size += touchSet.touchCount+1;
				for (currentTouch = 0; currentTouch < touchSet.touchCount; currentTouch++)
				{
					debugRam.buffer[counter++] = touchIDSet.weights[currentTouch][MAX_TOUCHES];
				}
				debugRam.buffer[counter++] = 0xff;
			   	sendDebugBytes();
			}
		}
#endif
#endif //ENABLE_DEBUG

		// zero out minimum weight column. and weights row
		for (counter = 0; counter < MAX_TOUCHES; counter++)
		{
			touchIDSet.weights[MAX_TOUCHES][counter] = 0;
		}

		// Step 4, count the ZEROs per column
		for (currentID = 0; currentID < MAX_TOUCHES; currentID++)
		{
			if (touchIDSet.tData[currentID].touchState != TS_ID_AVAILABLE
	 			&& touchIDSet.tData[currentID].touchState != TS_TOUCH_UP)
			{ // This is an ID we want to look at.
				for (currentTouch = 0; currentTouch < touchSet.touchCount; currentTouch++)
				{
					if (touchIDSet.weights[currentTouch][currentID] == 0)
					{
						if (touchIDSet.weights[MAX_TOUCHES][currentID] > 0)
						{
							if (touchSet.touchCount > touchIDSet.touchCount)
							{ // we have more touches than IDs
								// find previous zero, compare values.  "Nudge" up the
								// "worse" of the two values
								for (counter = 0; counter < currentTouch; counter++)
								{
									if (touchIDSet.weights[counter][currentID] == 0)
									{
										if (touchIDSet.weights[currentTouch][MAX_TOUCHES] > touchIDSet.weights[counter][MAX_TOUCHES])
										{
											touchIDSet.weights[currentTouch][currentID]++; // no longer a zero
										}
										else
										{
											touchIDSet.weights[counter ][currentID]++; // no longer a zero
										}
									}
								}
								touchIDSet.weights[MAX_TOUCHES][currentID]--;
							}
						}
						touchIDSet.weights[MAX_TOUCHES][currentID]++;
					}
				}
			}
		}

		// zero out minimum weight column. and weights row
		for (counter = 0; counter < MAX_TOUCHES; counter++)
		{
			touchIDSet.weights[counter][MAX_TOUCHES] = 0xff;
		}

		// Step 5, examine and look for multi-match
		badColumns = 0;
		for (currentID = 0; currentID < MAX_TOUCHES; currentID++)
		{

			if (touchIDSet.tData[currentID].touchState != TS_ID_AVAILABLE
	 			&& touchIDSet.tData[currentID].touchState != TS_TOUCH_UP)
			{ // This is an ID we want to look at.
				if (touchIDSet.weights[MAX_TOUCHES][currentID] > 1)
				{ // we have a multi-match  Find lowest non-zero value from each row...
				  // Check to see if there are more touches than IDs - if there are, then it is OK
				  // to have multi-match.  Flag one to create a new ID by changing the weight to 1.
					badColumns++;
					for (currentTouch = 0; currentTouch < touchSet.touchCount; currentTouch++)
					{
						if (touchIDSet.weights[currentTouch][currentID] == 0)
						{ // We found a touch with a weight of zero, now find next lowest value
							for (counter = 0; counter < MAX_TOUCHES; counter++)
							{
								if (touchIDSet.tData[counter].touchState != TS_ID_AVAILABLE
						 			&& touchIDSet.tData[counter].touchState != TS_TOUCH_UP)
								{ // This is an ID we want to look at.
									if (counter != currentID && touchIDSet.weights[currentTouch][counter] < touchIDSet.weights[currentTouch][MAX_TOUCHES])
									{
										touchIDSet.weights[currentTouch][MAX_TOUCHES] = touchIDSet.weights[currentTouch][counter];
									}
								}
							}
							// next lowest value found and stored in weights
						}
					}

					// now we have all of the potentials weighted, find the "best" and set the match
					// to 0xff and re-subtract all.
					counter = 0;
					for (currentTouch = 0; currentTouch < touchSet.touchCount; currentTouch++)
					{
						if (touchIDSet.weights[currentTouch][MAX_TOUCHES] < touchIDSet.weights[counter][MAX_TOUCHES])
						{
							counter = currentTouch;
						}
					}
					// counter now has the row we want to modify in it.
					touchIDSet.weights[counter][currentID] = 0xff;
					currentTouch = counter;
					for (counter = 0; counter < MAX_TOUCHES; counter++)
					{// Step 3, Subtract minimum weight from all weights for each touch
						if (touchIDSet.tData[counter].touchState != TS_ID_AVAILABLE
				 			&& touchIDSet.tData[counter].touchState != TS_TOUCH_UP)
						{ // This is an ID we want to look at.
							if (touchIDSet.weights[currentTouch][counter] < userRam.weightThreshold)//0xff) // parameterize this!
							{
								touchIDSet.weights[currentTouch][counter] -= touchIDSet.weights[currentTouch][MAX_TOUCHES];
							}
						}
					}
				}
			}
		}
	}

	// we should now have everything "settled", so let's assign them!
	// we no longer need the "min weight" and "number of matches" values,
	// so we will re-use the touchIDSet.weights[MAX_TOUCHES][currentID] and touchIDSet.weights[currentTouch][MAX_TOUCHES]
	// to track which IDs have touches associated (touchIDSet.weights[MAX_TOUCHES][currentID]) and which
	// touches have IDs associated (touchIDSet.weights[currentTouch][MAX_TOUCHES]), as well as the total
	// number of touches stored in touchIDSet.weights[MAX_TOUCHES][MAX_TOUCHES].

	for (counter = 0; counter < MAX_TOUCHES; counter++)
	{
		touchIDSet.weights[counter][MAX_TOUCHES] = 0xff;
		touchIDSet.weights[MAX_TOUCHES][counter] = 0xff;
	}
	touchIDSet.weights[MAX_TOUCHES][MAX_TOUCHES] = 0; // no touches IDed right now.
	for (currentID = 0; currentID < MAX_TOUCHES; currentID++)
	{
		if (touchIDSet.tData[currentID].touchState != TS_ID_AVAILABLE
 			&& touchIDSet.tData[currentID].touchState != TS_TOUCH_UP)
		{ // This is an ID we want to look at.
			// look at each ID, find the ZERO weight and assign it.
			for (currentTouch = 0; currentTouch < touchSet.touchCount; currentTouch++)
			{
				if (touchIDSet.weights[currentTouch][currentID] == 0)
				{ // this is our match
					touchIDSet.weights[currentTouch][MAX_TOUCHES] = currentID; // This touch is taken
					touchIDSet.weights[MAX_TOUCHES][currentID] = currentTouch; // This ID is taken
					touchIDSet.weights[MAX_TOUCHES][MAX_TOUCHES]++; // one more touch IDed.

					// Push touch history, then add touch to index 0
					for (counter = TOUCH_HISTORY; counter > 0; counter--)
					{
						touchIDSet.tData[currentID].touchLoc[counter] = touchIDSet.tData[currentID].touchLoc[counter-1];
					}
					//touchIDSet.tData[currentID].touchLoc[0] = touchSet.touchLoc[currentTouch];
					touchIDSet.tData[currentID].touchLoc[0].fineLocation = touchSet.touchLoc[currentTouch].fineLocation;
					touchIDSet.tData[currentID].touchLoc[0].roughLocation = touchSet.touchLoc[currentTouch].roughLocation;
					touchIDSet.tData[currentID].historySize++;

					if (touchIDSet.tData[currentID].historySize > TOUCH_HISTORY)
					{
						touchIDSet.tData[currentID].historySize = TOUCH_HISTORY;
					}

#ifdef ENABLE_DEBUG
#ifdef DIAGASSOCIATETOUCH_ENABLE
					if (userRam.flag1&CONTROLLERDIAGNOSTICS)
					{
						debugRam.ID = DIAGASSOCIATETOUCH;
					    debugRam.size = 3;
					    debugRam.buffer[0] = DIAGAT_TOUCHLINK;
						debugRam.buffer[1] = currentTouch;
						debugRam.buffer[2] = currentID;
				    	sendDebugBytes();
					}
#endif
#endif //ENABLE_DEBUG
				}
			}
		}
	}

	// now look for unallocated touches, set them to new IDs.

	for (currentTouch = 0; currentTouch < touchSet.touchCount; currentTouch++)
	{
		if (touchIDSet.weights[currentTouch][MAX_TOUCHES] == 0xff)
		{ // this touch is unallocated
			unsigned short temp;
			currentID = 0;

			// check to make sure that the touch isn't within "range" of an existing touch
			for (counter = 0; counter < MAX_TOUCHES; counter++)
			{
				if (touchIDSet.tData[counter].touchState != TS_ID_AVAILABLE
		 			&& touchIDSet.tData[counter].touchState != TS_TOUCH_UP)
				{
					temp = touchIDSet.tData[counter].touchLoc[0].fineLocation.x;
					if (temp > touchSet.touchLoc[currentTouch].fineLocation.x)
					{
						temp = temp - touchSet.touchLoc[currentTouch].fineLocation.x;
					}
					else
					{
						temp = touchSet.touchLoc[currentTouch].fineLocation.x - temp;
					}
					if (temp < userRam.minTouchDistance)
					{ // too close to an existing touch, eliminate this touch
						temp = touchIDSet.tData[counter].touchLoc[0].fineLocation.y;
						if (temp > touchSet.touchLoc[currentTouch].fineLocation.y)
						{
							temp = temp - touchSet.touchLoc[currentTouch].fineLocation.y;
						}
						else
						{
							temp = touchSet.touchLoc[currentTouch].fineLocation.y - temp;
						}
						if (temp < userRam.minTouchDistance)
						{ // too close to an existing touch, eliminate this touch
							currentID = MAX_TOUCHES;
						}
					}
				}
			}

			while (currentID < MAX_TOUCHES && touchIDSet.tData[currentID].touchState != TS_ID_AVAILABLE)
			{
				currentID++;
			}

			if (currentID < MAX_TOUCHES)
			{

#ifdef ENABLE_DEBUG
#ifdef DIAGASSOCIATETOUCH_ENABLE
				if (userRam.flag1&CONTROLLERDIAGNOSTICS)
				{
					debugRam.ID = DIAGASSOCIATETOUCH;
				    debugRam.size = 3;
				    debugRam.buffer[0] = DIAGAT_NEWTOUCH;
					debugRam.buffer[1] = currentTouch;
					debugRam.buffer[2] = currentID;
			    	sendDebugBytes();
				}
#endif
#endif //ENABLE_DEBUG
				touchIDSet.tData[currentID].touchState = TS_ID_ALLOCATED;//TS_TOUCH_DOWN;
				touchIDSet.tData[currentID].StateTimer = userRam.penDownTimer;//1; // parameterize this!
				touchIDSet.weights[currentTouch][MAX_TOUCHES] = currentID;
				touchIDSet.weights[MAX_TOUCHES][currentID] = currentTouch;
				touchIDSet.weights[MAX_TOUCHES][MAX_TOUCHES]++;
				// push this location onto all of the history
				for (counter = 0; counter < TOUCH_HISTORY; counter++)
				{
					//touchIDSet.tData[currentID].touchLoc[counter] = touchSet.touchLoc[currentTouch];
					touchIDSet.tData[currentID].touchLoc[counter].fineLocation = touchSet.touchLoc[currentTouch].fineLocation;
					touchIDSet.tData[currentID].touchLoc[counter].roughLocation = touchSet.touchLoc[currentTouch].roughLocation;
				}
				touchIDSet.tData[currentID].historySize = 1;
				touchIDSet.touchCount++;
			}
		}
	}

	for (currentID = 0; currentID < MAX_TOUCHES; currentID++)
	{ // drive the touch state machine
		if (touchIDSet.weights[MAX_TOUCHES][currentID] != 0xff)
		{ // We have a touch associated with this ID
			if (touchIDSet.tData[currentID].touchState == TS_TOUCH_SEARCH)
			{
				touchIDSet.tData[currentID].touchState = TS_TOUCH_STREAM;
			}
			else if (touchIDSet.tData[currentID].StateTimer == 0)
			{
				if (touchIDSet.tData[currentID].touchState == TS_ID_ALLOCATED)
				{
					touchIDSet.tData[currentID].touchState = TS_TOUCH_DOWN;
				}
			}
		}
		else
		{ // no touch associated with this ID.
			if (touchIDSet.tData[currentID].touchState == TS_ID_ALLOCATED)
			{
				touchIDSet.tData[currentID].touchState = TS_ID_AVAILABLE;
				touchIDSet.touchCount--;
			}
			else if (touchIDSet.tData[currentID].touchState == TS_TOUCH_STREAM)
			{
				touchIDSet.tData[currentID].touchState = TS_TOUCH_SEARCH;
				touchIDSet.tData[currentID].StateTimer = userRam.penUpTimer;//3; // Initi Pen-up timer. Parameterize this
			}
			else if (touchIDSet.tData[currentID].touchState == TS_TOUCH_SEARCH)
			{
				if (touchIDSet.tData[currentID].StateTimer == 0)
				{
					touchIDSet.tData[currentID].touchState = TS_TOUCH_UP;
				}
			}
		}
		if (touchIDSet.tData[currentID].StateTimer > 0)
		{
			touchIDSet.tData[currentID].StateTimer--;
		}
	}
	return 0;
}

/*************************************************************************//**
* Function:        	unsigned char touchWeight(unsigned char touchID, unsigned char potentialTouch)
*
* \pre		    	touchIDSet, touchSet contain valid data
*
* \result          	Provides a weighting value between the given touchID and
*					a potential touch.
*
* \post		    	None
*
* \brief        	Generates a "weight" for a given touch point with regard to
*					an existing touch ID.  The lower the return value, the
*					better the match.
*
* \note
*
*****************************************************************************/
unsigned char touchWeight(unsigned char touchID, unsigned char potentialTouch)
{
	unsigned short currentWeight = 0xffff;


	// first check the state of the touchID
	if (touchID < MAX_TOUCHES
						 	&& touchIDSet.tData[touchID].touchState != TS_ID_AVAILABLE
							&& touchIDSet.tData[touchID].touchState != TS_TOUCH_UP)
	{
		currentWeight = simpleDistance(touchIDSet.tData[touchID].touchLoc[0].fineLocation, touchSet.touchLoc[potentialTouch].fineLocation);
	}
	if (currentWeight > 0xff)
	{
		currentWeight = 0xff;
	}
	return (currentWeight&0xff);
}

/*************************************************************************//**
* Function:        	unsigned short shortDistance(UINTCOORD pointA, UINTCOORD pointB)
*
* \pre
*
* \result          	Returns the larger distance (either in X or Y) between the points.
*
* \post		    	None
*
* \brief        	Provides a simple distance between the two given points.
*
* \note
*
*****************************************************************************/
unsigned short shortDistance(UINTCOORD pointA, UINTCOORD pointB)
{
	UINTCOORD testDeltas;
	if (pointA.x > pointB.x)
	{
		testDeltas.x = pointA.x - pointB.x;
	}
	else
	{
		testDeltas.x = pointB.x - pointA.x;
	}

	if (pointA.y > pointB.y)
	{
		testDeltas.y = pointA.y - pointB.y;
	}
	else
	{
		testDeltas.y = pointB.y - pointA.y;
	}

	if (testDeltas.y > testDeltas.x)
	{
		testDeltas.x = testDeltas.y;
	}

	return testDeltas.x;
}

/*************************************************************************//**
* Function:        	unsigned char simpleDistance(UINTCOORD pointA, UINTCOORD pointB)
*
* \pre
*
* \result          	The "manhattan" distance between the two given points.
*
* \post		    	None
*
* \brief        	Provides a simple distance between the two given points.
*					avoids all "high" math - divide, square, square root, etc.
*					So generates a "manhattan" distance - that is, the
*					distance on the X axis, plus the distance on the Y axis.
*
* \note
*
*****************************************************************************/
unsigned char simpleDistance(UINTCOORD pointA, UINTCOORD pointB)
{
	UINTCOORD testDeltas;
	if (pointA.x > pointB.x)
	{
		testDeltas.x = pointA.x - pointB.x;
	}
	else
	{
		testDeltas.x = pointB.x - pointA.x;
	}

	if (pointA.y > pointB.y)
	{
		testDeltas.y = pointA.y - pointB.y;
	}
	else
	{
		testDeltas.y = pointB.y - pointA.y;
	}

	testDeltas.x += testDeltas.y;

	testDeltas.x >>= 1;

	if (testDeltas.x > 0xff)
	{
		testDeltas.x = 0xff;
	}
	return (testDeltas.x&0xff);
}

/*************************************************************************//**
* Function:        	void extrapolateTouch(TOUCHID* thisTouch, UINTCOORD* next)
*
* \pre
*
* \result          	'next' will be populated with the extrapolated touch
*					position, given the history in 'thisTouch'.
*
* \post		    	None
*
* \brief        	Linear extrapolation of where the next touch location
*					will be, given the history of the associated touches with
*					the ID.
*
* \note
*
*****************************************************************************/
void extrapolateTouch(TOUCHID* thisTouch, UINTCOORD* next)
{
	UINTCOORD to = thisTouch->touchLoc[0].fineLocation;
	UINTCOORD from = thisTouch->touchLoc[1].fineLocation;
	next->x = to.x + (to.x-from.x);
	next->y = to.y + (to.y-from.y);
}


/*************************************************************************//**
* Function:        	unsigned char handleTouches(void)
*
* \pre		    	touchIDSet is initialized
*
* \result          	Scales and flips coordinates for valid Touches, calls
*					sendCommTouchReport for each touch that should be
*					transmitted.
*
* \post		    	None
*
* \brief        	Looks through every touch ID, process and transmits those
*					that are in a state that should be transmitted -
*					TOUCH_DOWN, TOUCH_STREAM, and TOUCH_UP are the only states
*					that should be transmitted.
*
* \note
*
*****************************************************************************/
void handleTouches(void)
{
	unsigned char currentTouch;
	for (currentTouch = 0; currentTouch < MAX_TOUCHES; currentTouch++)
	{
		if (touchIDSet.tData[currentTouch].touchState == TS_TOUCH_DOWN ||
			touchIDSet.tData[currentTouch].touchState == TS_TOUCH_STREAM ||
			touchIDSet.tData[currentTouch].touchState == TS_TOUCH_UP)
		{ // This touch is somewhere in the state machine (not in a pen down/up timer)
			unsigned char avgCount = userRam.numOfAvg;
			unsigned char currentHistory;
			PCAPULONG xVal, yVal;
			// average the data
			xVal = 0;
			yVal = 0;
			if (touchIDSet.tData[currentTouch].historySize < avgCount)
			{
				avgCount = touchIDSet.tData[currentTouch].historySize;
			}
			for (currentHistory = 0; currentHistory < avgCount; currentHistory++)
			{
				xVal += touchIDSet.tData[currentTouch].touchLoc[currentHistory].fineLocation.x;
				yVal += touchIDSet.tData[currentTouch].touchLoc[currentHistory].fineLocation.y;
			}


			xVal = xVal/avgCount;
			yVal = yVal/avgCount;
		
			// Scale to 14 bit data
			xVal = xVal*userRam.xmul;
			xVal >>= 11;
			yVal = yVal*userRam.ymul;
			yVal >>= 11;

			if (xVal > 0x3FFF)
			{
				xVal = 0;
			}
			if (xVal > 0xFFF)
			{
				xVal = 0xFFF;
			}
			if (yVal > 0x3FFF)
			{
				yVal = 0;
			}
			if (yVal > 0xFFF)
			{
				yVal = 0xFFF;
			}

			//
			// apply the flipState
			//
			switch(userRam.flipState&0x03) // apply flips first
			{
			case 1: // X flipped
				xVal = 0x0FFF - xVal;
				break;
			case 2: // Y flipped
				yVal = 0x0FFF - yVal;
				break;
			case 3: // X flipped and Y flipped
				xVal = 0x0FFF - xVal;
				yVal = 0x0FFF - yVal;
				break;
			}
		
			//
			// check if X&Y exchanged
			//
			if ((userRam.flipState&0x04) == 0x04)
			{
				//unsigned long temp = xVal;
				PCAPULONG temp = xVal;
				xVal = yVal;
				yVal = temp;
			}

			touchIDSet.tData[currentTouch].scaleLocation.x = xVal;
			touchIDSet.tData[currentTouch].scaleLocation.y = yVal;
			sendCommTouchReport(currentTouch);
			if (touchIDSet.tData[currentTouch].touchState == TS_TOUCH_UP)
			{
				touchIDSet.tData[currentTouch].touchState = TS_ID_AVAILABLE;
		    	touchIDSet.touchCount--;
			}
			if (touchIDSet.tData[currentTouch].touchState == TS_TOUCH_DOWN)
			{
				touchIDSet.tData[currentTouch].touchState = TS_TOUCH_STREAM;
			}
		}
		if (gesture[currentTouch].data != GT_NONE)
		{
			sendCommGestureReport(currentTouch);
		}
	}
	if((varRam.flag&TOUCH) == TOUCH)
	{
			varRam.flag |= LASTTOUCH;
	}
	else
	{
			varRam.flag &= NOTLASTTOUCH;
	}
}
