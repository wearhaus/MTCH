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
/*! \file ColCache.c
	\brief Caching system for Mutual Measurement Columns.

	This file contains the logic for mutual measurement caching.  In order
	to minimize RAM use, it is limited to caching 5 columns of the sensor.
*/
#include "main\main.h"

// Data Structures first, optimized for storage space

#ifdef ENABLE_DEBUG
extern DEBUGRAM debugRam;
#endif // ENABLE_DEBUG
extern COLCACHE colCache;
extern USERRAM userRam;


/*************************************************************************//**
* Function:        	unsigned char initColCache()
*
* \pre		    	None
*
* \result          	initializes column cache data structures.
*
* \post		    	None
*
* \brief        	Initializes "colCache" data structure for the colum cache.
*
* \note           	
*			
*****************************************************************************/
unsigned char initColCache()
{
	unsigned char counter = 0;
	unsigned char counter2 = 0;
	colCache.currentLeftEdge = 0xff;
	colCache.currentRightEdge = 0xff;
	for (counter = 0; counter < userRam.numberOfTXChannels; counter++)
	{
		for (counter2 = 0; counter2 < COLCACHESIZE; counter2++)
		{
			colCache.mutCol[counter2][counter] = 0xffff;
		}
	}

#ifdef DIAGCOLCACHE_ENABLE
	debugRam.ID = DIAGCOLCACHE;
    debugRam.size = 1;
    debugRam.buffer[0] = DIAGCOLCACHE_INIT;
    sendDebugBytes();
#endif
	return 0;
}

/*************************************************************************//**
* Function:        	unsigned int queryColCache(unsigned char col, unsigned char row, unsigned char scanCycles)
*
* \pre		    	cocCache must be initialized.
*
* \result          	returns the mutual measurement at col, row.
*
* \post		    	None
*
* \brief        	Caches the mutual measurement at a given location.  Contains
*					five "columns" of cache, allowing some left-right motion of
*					the scans, but is designed for a linear analyisis of the sensor,
*					starting at one side and moving to the other side.
*
* \note           	Automatically moves with columns it is caching to try to
*					cover the most likely columns that will be needed.
*			
*****************************************************************************/
unsigned short queryColCache(unsigned char col, unsigned char row, unsigned char scanCycles)
{
	unsigned short returnValue = 0xffff;

#ifdef DIAGCOLCACHE_ENABLE
	if (userRam.flag1&CONTROLLERDIAGNOSTICS)
	{
		debugRam.ID = DIAGCOLCACHE;
	    debugRam.size = 3;
	    debugRam.buffer[0] = DIAGCOLCACHE_QUERY;
		debugRam.buffer[1] = col;
		debugRam.buffer[2] = row;
	    sendDebugBytes();
	}
#endif
/*	if (scanCycles != 1)
	{
		// We need to get the mutual value, query it and put it in the buffer
		returnValue = 0;
		scanMutual(col, row, scanCycles);
		returnValue = getNormalizedMutual(col, row, scanCycles);
		return returnValue;
	}*/

	// We want to find the data in col, row location.
	// first, are we "empty"?
	if (colCache.currentLeftEdge == 0xff)
	{
		if (col > 0)
		{
			colCache.currentLeftEdge = col-1;
		}
		else
		{
			colCache.currentLeftEdge = 0;
		}
		colCache.currentRightEdge = colCache.currentLeftEdge + (COLCACHESIZE-1);
		if (colCache.currentRightEdge > userRam.numberOfRXChannels)
		{
			colCache.currentRightEdge = userRam.numberOfRXChannels;
		}
	}

	// Now check to see if we are within the constraints of the cache
	if (colCache.currentLeftEdge > col)
	{
		// failure mode - the column is to our left!
		while (colCache.currentLeftEdge > col && colCache.currentLeftEdge > 0)
		{
			unsigned char counter = 0;
			for (counter = 0; counter < MAXTX; counter++)
			{
				colCache.mutCol[colCache.currentRightEdge%COLCACHESIZE][counter] = 0xffff;
			}
			colCache.currentLeftEdge--;
			colCache.currentRightEdge--;
		}
	}
	if (colCache.currentRightEdge < col)
	{
		// failure mode, column is to our right (outside of window)
		while (colCache.currentRightEdge < col && colCache.currentRightEdge < userRam.numberOfRXChannels)
		{
			unsigned char counter = 0;
			for(counter = 0; counter < MAXTX; counter++)
			{
				colCache.mutCol[colCache.currentLeftEdge%COLCACHESIZE][counter] = 0xffff;
			}
			colCache.currentLeftEdge++;
			colCache.currentRightEdge++;
		}	
	}
	
	
	// Our cache should now cover the queried area
	// Query the actual value and return it
	returnValue = colCache.mutCol[col%COLCACHESIZE][row];

	if (0xffff == returnValue)
	{	
		// We need to get the mutual value, query it and put it in the buffer
		returnValue = 0;
		scanMutual(col, row, NORMAL_SCAN);
		returnValue = getNormalizedMutual(col, row);
		
#ifdef DIAGCOLCACHE_ENABLE
		if (userRam.flag1&CONTROLLERDIAGNOSTICS)
		{
			debugRam.ID = DIAGCOLCACHE;
		    debugRam.size = 5;
		    debugRam.buffer[0] = DIAGCOLCACHE_QUERYVALUE;
			debugRam.buffer[1] = col;
			debugRam.buffer[2] = row;
			debugRam.buffer[3] = (returnValue>>8)&0xff;
			debugRam.buffer[4] = returnValue&0xff;
		    sendDebugBytes();
		}
#endif

		if (0xffff == returnValue)
		{
			returnValue = 0xfffe;
		}
		
		// now we have a value, save it in the cache	
		colCache.mutCol[col%COLCACHESIZE][row] = returnValue;
	}
		
	return returnValue;
}
