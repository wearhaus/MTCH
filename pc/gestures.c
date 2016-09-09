/*****************************************************************************
* CODE OWNERSHIP AND DISCLAIMER OF LIABILITY
*
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in
* all derivatives hereto.  You may use this code, and any derivatives created
* by any person or entity by or on your behalf, exclusively with Microchip
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
* WHETHER IN CONTRACT, WARRANTY, TO12RT (INCLUDING NEGLIGENCE OR BREACH OF
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
* name                  05/07/12    ...
******************************************************************************/
/*
	TSCG				05/07/12


*/
#include "main\main.h"

//
// Globals
//
extern GESTURE gesture[MAX_TOUCHES];
extern USERRAM userRam;
extern TOUCHIDSET touchIDSet;
extern DEBUGRAM debugRam;

//TOUCH POSITIONS//
#define TOUCH_XPOS 				touchIDSet.tData[ID].touchLoc[0].fineLocation.x
#define TOUCH_YPOS 				touchIDSet.tData[ID].touchLoc[0].fineLocation.y
#define TOUCH_LAST_XPOS			touchIDSet.tData[ID].touchLoc[1].fineLocation.x
#define TOUCH_LAST_YPOS			touchIDSet.tData[ID].touchLoc[1].fineLocation.y

//GENERAL STATES//
#define GS_RESET				0

//SWIPE STATES//
#define GSS_SWIPE_CONFIRM		1
#define GSS_SWIPE_HOLD			2

//CLICK STATES//
#define GCS_FIRST_CLICK			3

/*************************************************************************//**
* Function:        	void initGestures(void)
*
* \pre		    	None
*
* \result          	None
*
* \post		    	None
*
* \brief        	initialize all gesture variables and data structures
*
* \note           	None
*
*****************************************************************************/
void initGestures(void)
{
	unsigned char x;

	for(x=0;x<MAX_TOUCHES;x++)
	{
		gesture[x].timer = 0;
		gesture[x].holdVal = 0;
		gesture[x].gestureType = GT_NONE;
		gesture[x].swipeDirection = GV_NONE;
		gesture[x].data = GV_NONE;
		gesture[x].state = GS_RESET;
	//	gesture[x].clickTime = 0;
	}
}

/*************************************************************************//**
* Function:        	void checkGestureState(void)
*
* \pre		    	None
*
* \result          	None
*
* \post		    	None
*
* \brief        	runs the gesture state machine based on the current touch ID,
*					touch state, and gesture state
*
* \note           	None
*
*****************************************************************************/
void checkGestureState(void)
{
	unsigned char ID;
	unsigned short tempVarX, tempVarY, tX, tY;

	for (ID = 0; ID < MAX_TOUCHES; ID++)
	{
		//initially clear data
		gesture[ID].data = GV_NONE;
		if (touchIDSet.tData[ID].touchState > 2)
		{  // we only care about the ID if it is actually being used

			//if between double click and time exceeds allowance, clear gesture
			if(gesture[ID].state == GCS_FIRST_CLICK)
			{
				if(gesture[ID].timer > userRam.maxClickTime)
				{
					gesture[ID].gestureType = GT_NONE;
					gesture[ID].state = GS_RESET;
				}
			}

			switch(touchIDSet.tData[ID].touchState)
			{
				case TS_ID_AVAILABLE:
					gesture[ID].gestureType = GT_NONE;
					break;

				//TOUCH UP STATE//
				case TS_TOUCH_UP:
					//if swipe gesture, load data and clear out structure
					if(gesture[ID].gestureType == GT_SWIPE)
					{
						gesture[ID].data = (flipCorrectSwipe(gesture[ID].swipeDirection) << 4) + gesture[ID].state;
						gesture[ID].state = GS_RESET;
						gesture[ID].gestureType = GT_NONE;
					}
					//if still undefined gesture, test for single/double click
					else if(gesture[ID].gestureType == GT_UNDEFINED)
					{
						tX = (unsigned short)(userRam.numberOfRXChannels)<<7;
						tY = (unsigned short)(userRam.numberOfTXChannels)<<7;
						tX -= userRam.edgeKeepoutDistance;
						tY -= userRam.edgeKeepoutDistance;
						//check "keepout" area around sensor border
						if((gesture[ID].start_x > userRam.edgeKeepoutDistance) &&
						   (gesture[ID].start_y > userRam.edgeKeepoutDistance) &&
						   (gesture[ID].start_x < tX) &&
						   (gesture[ID].start_y < tY))
						{
							tempVarX = magnitude(TOUCH_XPOS, gesture[ID].start_x);
							tempVarY = magnitude(TOUCH_YPOS, gesture[ID].start_y);

							//check if touch has moved much since pen down coordinates
							if((tempVarX <= userRam.tapThresh) && (tempVarY <= userRam.tapThresh))
							{
								//load single click data
								gesture[ID].data = (GV_SINGLECLICK<<4);
								gesture[ID].state = GCS_FIRST_CLICK;
								gesture[ID].second_x = gesture[ID].start_x;
								gesture[ID].second_y = gesture[ID].start_y;
								gesture[ID].timer = 0;
							}
							else
							{
								gesture[ID].state = GS_RESET;
							}
						}
						else
						{
							gesture[ID].state = GS_RESET;
						}
					}
					gesture[ID].gestureType = GT_NONE;
					gesture[ID].swipeDirection = GV_NONE;
					break;

				//HOLD TOUCH STATE//
				case TS_TOUCH_STREAM:
					//if start of gesture, grab initial coordinates
					if (gesture[ID].gestureType == GT_NONE)
					{
					 	gesture[ID].start_x = TOUCH_XPOS;		//capture x and y coordinates as beginning of swipe
						gesture[ID].start_y = TOUCH_YPOS;
						gesture[ID].gestureType = GT_UNDEFINED;
						if(gesture[ID].state != GCS_FIRST_CLICK)
						{
							gesture[ID].timer = 0;				//clear swipe time counter
						}
					}
					//if undefined gesture, check for swipe or double click
					else if(gesture[ID].gestureType == GT_UNDEFINED)
					{
						if(gesture[ID].state == GCS_FIRST_CLICK)
						{
							//check if the click timer hasn't overflowed
							if(gesture[ID].timer < userRam.maxClickTime)
							{
								tempVarX = magnitude(gesture[ID].start_x, gesture[ID].second_x);
								tempVarY = magnitude(gesture[ID].start_y, gesture[ID].second_y);
								if(tempVarX <= userRam.tapThresh && tempVarY <= userRam.tapThresh)
								{
									//load double click data and clear out structure
									gesture[ID].data = (GV_DOUBLECLICK<<4);
									gesture[ID].state = GS_RESET;
									gesture[ID].gestureType = GT_DONE;
								}
							}
							gesture[ID].timer = 0;
						}
						if(gesture[ID].timer <= userRam.swipeTime)
						{
							detectSwipe(ID);
						}
						if(gesture[ID].timer > userRam.tapTime)
						{
							gesture[ID].gestureType = GT_HOLDCLICK;
						}
					}
					//if swipe gesture, check for user holding a swipe
					else if(gesture[ID].gestureType == GT_SWIPE)
					{
						if(gesture[ID].state == GSS_SWIPE_CONFIRM)
						{
							checkSwipeHold(ID);
						}
						else if(gesture[ID].state == GSS_SWIPE_HOLD)
						{
							tempVarX = magnitude(TOUCH_XPOS, gesture[ID].xBoundary);
							tempVarY = magnitude(TOUCH_YPOS, gesture[ID].yBoundary);

							//check for touch movement when user is holding a swipe
							if((tempVarX > userRam.holdSwipeBoundary) || (tempVarY > userRam.holdSwipeBoundary))
							{
								gesture[ID].gestureType = GT_DONE;
								gesture[ID].swipeDirection = GV_NONE;
								gesture[ID].state = GS_RESET;
							}
							else
							{
								gesture[ID].data = (flipCorrectSwipe(gesture[ID].swipeDirection) << 4) + gesture[ID].state;
							}
						}
					}
					//if user is holding a single click, monitor for touch movement
					else if (gesture[ID].gestureType == GT_HOLDCLICK)
					{
						detectTap(ID);
					}
					break;

				default:
					break;
			}
			#ifdef ENABLE_DEBUG
			if (userRam.flag1&CONTROLLERDIAGNOSTICS)
			{
				debugRam.size = 2;
				debugRam.ID = DIAGGESTURE;
				debugRam.buffer[0] = ID;
				debugRam.buffer[1] = gesture[ID].data;
				sendDebugBytes();
			}
			#endif // ENABLE_DEBUG
		}
	}
}

/*************************************************************************//**
* Function:        	unsigned char flipCorrectSwipe(unsigned char baseDir)
*
* \pre		    	None
*
* \result          	None
*
* \post		    	None
*
* \brief        	Takes a swipe gesture and corrects the direction based
*					upon the current flip state of the sensor.
*
* \note           	None
*
*****************************************************************************/
unsigned char flipCorrectSwipe(unsigned char baseDir)
{
	//
	// apply the flipState
	//

	if (baseDir&0x01)
	{ // swipe is Up or down
		if (userRam.flipState&0x02)
		{	// swap direction
			baseDir ^= 0x06;
		}
	}
	else
	{ // swipe is left or right
		if (userRam.flipState&0x01)
		{
			baseDir ^= 0x02;
		}
	}

	if (userRam.flipState&0x04)
	{ // swap X & Y
		if (baseDir&0x02)
		{
			baseDir ^= 0x05;
		}
		else
		{
			baseDir ^= 0x01;
		}
	}
	return baseDir;
}

/*************************************************************************//**
* Function:        	void detectSwipe(unsigned char ID)
*
* \pre		    	None
*
* \result          	None
*
* \post		    	None
*
* \brief        	detects the initial action of a swipe gesture as well as
*					the direction in which the swipe is moving
*
* \note           	None
*
*****************************************************************************/
void detectSwipe(unsigned char ID)
{
	unsigned short mag_x, mag_y;
	unsigned char dir_x, dir_y;

	mag_x = magnitude(TOUCH_XPOS, gesture[ID].start_x);
	mag_y = magnitude(TOUCH_YPOS, gesture[ID].start_y);

	dir_x = direction(TOUCH_XPOS, gesture[ID].start_x);
	dir_y = direction(TOUCH_YPOS, gesture[ID].start_y);

	//check if touch has moved far enough to be a swipe
	if((mag_y >= userRam.swipeLengthY) || (mag_x >= userRam.swipeLengthX))
	{
		//increment gesture state
		gesture[ID].state = GSS_SWIPE_CONFIRM;
		gesture[ID].gestureType = GT_SWIPE;

		//store coordinates where swipe was identified
		gesture[ID].second_x = TOUCH_XPOS;
		gesture[ID].second_y = TOUCH_YPOS;

		//check the direction of the movement
		if(mag_x >= mag_y)
		{
			gesture[ID].holdVal = TOUCH_XPOS;
			if(dir_x == 1)
			{
				gesture[ID].swipeDirection = GV_RIGHT_SWIPE;
			}
			else
			{
				gesture[ID].swipeDirection = GV_LEFT_SWIPE;
			}
		}
		else
		{
			gesture[ID].holdVal = TOUCH_YPOS;
			if(dir_y == 1)
			{
				gesture[ID].swipeDirection = GV_DOWN_SWIPE;
			}
			else
			{
				gesture[ID].swipeDirection = GV_UP_SWIPE;
			}
		}
	}
}

/*************************************************************************//**
* Function:        	void checkSwipeHold(unsigned char ID)
*
* \pre		    	swipe gesture must be confirmed
*
* \result          	None
*
* \post		    	None
*
* \brief        	based on the direction in which the swipe is moving, set a
*					boundary so that a swipe cannot move backwards. Also monitor
*					the velocity of a swipe so we can determine when the user has
*					stopped swiping and is now holding the gesture.
*
* \note           	None
*
*****************************************************************************/
void checkSwipeHold(unsigned char ID)
{
	unsigned short mag_x, mag_y;

	mag_x = magnitude(TOUCH_XPOS, TOUCH_LAST_XPOS);
	mag_y = magnitude(TOUCH_YPOS, TOUCH_LAST_YPOS);

	switch(gesture[ID].swipeDirection)
	{
		case GV_RIGHT_SWIPE:
			//check swipe for backwards movement
			if (TOUCH_XPOS > gesture[ID].holdVal)
			{
				gesture[ID].holdVal = TOUCH_XPOS;
			}
			if (TOUCH_XPOS >= (gesture[ID].holdVal - userRam.swipeHoldThresh))
			{
				//check swipe velocity
				if ((mag_x < userRam.minSwipeVelocity) && (mag_y < userRam.minSwipeVelocity))
				{
					gesture[ID].xBoundary = TOUCH_XPOS;
					gesture[ID].yBoundary = TOUCH_YPOS;

					gesture[ID].state = GSS_SWIPE_HOLD;
				}
				else
				{
					//check the secondary direction for excessive movement
					mag_y = magnitude(TOUCH_YPOS, gesture[ID].second_y);
					if(mag_y > 400)
					{
						gesture[ID].gestureType = GT_DONE;
						gesture[ID].state = GS_RESET;
					}
				}
			}
			else
			{
				gesture[ID].gestureType = GT_DONE;
				gesture[ID].state = GS_RESET;
				gesture[ID].swipeDirection = GV_NONE;
			}
			break;

		case GV_LEFT_SWIPE:
			//check swipe for backwards movement
			if (TOUCH_XPOS < gesture[ID].holdVal)
			{
				gesture[ID].holdVal = TOUCH_XPOS;
			}
			//check swipe velocity
			if (TOUCH_XPOS <= (gesture[ID].holdVal + userRam.swipeHoldThresh))
			{
				if ((mag_x < userRam.minSwipeVelocity) && (mag_y < userRam.minSwipeVelocity))								//if not, then check velocity of swipe
				{
					gesture[ID].xBoundary = TOUCH_XPOS;
					gesture[ID].yBoundary = TOUCH_YPOS;

					gesture[ID].state = GSS_SWIPE_HOLD;
				}
				else
				{
					//check the secondary direction for excessive movement
					mag_y = magnitude(TOUCH_YPOS, gesture[ID].second_y);
					if(mag_y > 400)
					{
						gesture[ID].gestureType = GT_DONE;
						gesture[ID].state = GS_RESET;
					}
				}
			}
			else
			{
				gesture[ID].gestureType = GT_DONE;
				gesture[ID].state = GS_RESET;
				gesture[ID].swipeDirection = GV_NONE;
			}
			break;

		case GV_DOWN_SWIPE:
			//check swipe for backwards movement
			if (TOUCH_YPOS > gesture[ID].holdVal)
			{
				gesture[ID].holdVal = TOUCH_YPOS;
			}
			if (TOUCH_YPOS >= (gesture[ID].holdVal - userRam.swipeHoldThresh))
			{
				//check swipe velocity
				if ((mag_x < userRam.minSwipeVelocity) && (mag_y < userRam.minSwipeVelocity))								//if not, then check velocity of swipe
				{
					gesture[ID].xBoundary = TOUCH_XPOS;
					gesture[ID].yBoundary = TOUCH_YPOS;

					gesture[ID].state = GSS_SWIPE_HOLD;
				}
				else
				{
					//check the secondary direction for excessive movement
					mag_x = magnitude(TOUCH_XPOS, gesture[ID].second_x);
					if(mag_x > 400)
					{
						gesture[ID].gestureType = GT_DONE;
						gesture[ID].state = GS_RESET;
					}
				}
			}
			else
			{
				gesture[ID].gestureType = GT_DONE;
				gesture[ID].state = GS_RESET;
				gesture[ID].swipeDirection = GV_NONE;
			}
			break;

		case GV_UP_SWIPE:
			//check swipe for backwards movement
			if (TOUCH_YPOS < gesture[ID].holdVal)
			{
				gesture[ID].holdVal = TOUCH_YPOS;
			}
			if (TOUCH_YPOS <= (gesture[ID].holdVal + userRam.swipeHoldThresh))
			{
				//check swipe velocity
				if ((mag_x < userRam.minSwipeVelocity) && (mag_y < userRam.minSwipeVelocity))								//if not, then check velocity of swipe
				{
					gesture[ID].xBoundary = TOUCH_XPOS;
					gesture[ID].yBoundary = TOUCH_YPOS;

					gesture[ID].state = GSS_SWIPE_HOLD;
				}
				else
				{
					//check the secondary direction for excessive movement
					mag_x = magnitude(TOUCH_XPOS, gesture[ID].second_x);
					if(mag_x > 400)
					{
						gesture[ID].gestureType = GT_DONE;
						gesture[ID].state = GS_RESET;
					}
				}
			}
			else
			{
				gesture[ID].gestureType = GT_DONE;
				gesture[ID].state = GS_RESET;
				gesture[ID].swipeDirection = GV_NONE;
			}
			break;

		default:
			break;
	}
}

/*************************************************************************//**
* Function:        	void detectTap(unsigned char ID)
*
* \pre		    	None
*
* \result          	None
*
* \post		    	None
*
* \brief        	detects a tap gesture from the user by monitoring movement
*					of the touch since the initial pen down coordinates
*
* \note           	None
*
*****************************************************************************/
void detectTap(unsigned char ID)
{
	unsigned short tempVarX, tempVarY;

	tempVarX = magnitude(TOUCH_XPOS, gesture[ID].start_x);
	tempVarY = magnitude(TOUCH_YPOS, gesture[ID].start_y);

	//check if touch has moved from initial pen down coordinates
	if((tempVarX <= userRam.tapThresh) && (tempVarY <= userRam.tapThresh))
	{
		//load hold click data
		gesture[ID].data = (GV_SINGLECLICK<<4) + 0x01;
		gesture[ID].timer = userRam.tapTime;
	}
	else
	{
		gesture[ID].gestureType = GT_DONE;
		gesture[ID].state = GS_RESET;
		gesture[ID].timer = 0;
	}
}
