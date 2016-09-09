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

/*! \file lcddisplay.h
	\brief Contains all headers and data structures for the lcd display.

	This file handles all functionality of the lcd display.  This implementation
	uses a EA DOGL128x-9 with no backlight.
*/
#ifndef __LCDDISPLAY_H
#define __LCDDISPLAY_H

#include "main\main.h"

#define USE_SMALL_ICONS

#ifndef BYTE
#define BYTE unsigned char
#endif

#ifdef __18CXX
// C18 Compiler
#define ROM_DATA static near rom const
#define inline
#else
#define ROM_DATA static const
#endif
// LCD Init options
typedef enum {
	INIT_FAST = 0, // Just init the display
	INIT_FULL = 1  // Clear the buffer
} LCD_INIT_TYPE;

// LCD Mode options
typedef enum {
	LCD_CONTROL = 0,
	LCD_DISPLAY = 1
} LCD_MODE;

// LCD Clear options
typedef enum {
	CLEAR_BLANK = 0,
	CLEAR_INVERT = 1,
	CLEAR_LOGO = 2
} LCD_CLEAR_TYPE;

// LCD Color options
typedef enum {
	COLOR_WHITE = 0,
	COLOR_BLACK = 1,
	COLOR_INVERT = 2
} LCD_COLOR;

// Glyph copy options
typedef enum {
	GLYPH_COPY = 0,
	GLYPH_OR = 1,
	GLYPH_AND = 2,
	GLYPH_XOR = 3
} GLYPH_COPY_TYPE;

// Configure overlay timeout (how long it stays on the display) and position
#define OVERLAY_TIMEOUT 400
#define OVERLAY_POSITION_X 112
#define OVERLAY_POSITION_Y 0

// Overlay options
typedef enum {
	OVERLAY_NONE = 0,
	OVERLAY_SWIPE_RIGHT = 1,
	OVERLAY_SWIPE_RIGHT_HOLD = 2,
	OVERLAY_SWIPE_LEFT = 3,
	OVERLAY_SWIPE_LEFT_HOLD = 4,
	OVERLAY_SWIPE_UP = 5,
	OVERLAY_SWIPE_UP_HOLD = 6,
	OVERLAY_SWIPE_DOWN = 7,
	OVERLAY_SWIPE_DOWN_HOLD = 8,
	OVERLAY_CLICK = 9,
	OVERLAY_CLICK_HOLD = 10,
	OVERLAY_DOUBLE_CLICK = 11,
	OVERLAY_SLEEP = 12
} LCD_OVERLAY_ID;

// String Options
typedef enum {
	STRING_VERSION = 0,
	STRING_DATE = 1,
	STRING_TOUCH = 2
} LCD_STRING_ID;

// Potential glyphs
typedef enum {
	GLYPH_NONE = 0,
	GLYPH_FONT = 0,
	GLYPH_SMILE = 1,
	GLYPH_FROWN = 2,
	GLYPH_HATCH = 3,
	GLYPH_GHOST = 4,
	GLYPH_FACE_MASK = 5,
	GLYPH_GHOST_MASK = 6,
	GLYPH_SWIPE_RIGHT = 7,
	GLYPH_SWIPE_RIGHT_HOLD = 8,
	GLYPH_SWIPE_RIGHT_MASK = 9,
	GLYPH_SWIPE_LEFT = 10,
	GLYPH_SWIPE_LEFT_HOLD = 11,
	GLYPH_SWIPE_LEFT_MASK = 12,
	GLYPH_SWIPE_UP = 13,
	GLYPH_SWIPE_UP_HOLD = 14,
	GLYPH_SWIPE_UP_MASK = 15,
	GLYPH_SWIPE_DOWN = 16,
	GLYPH_SWIPE_DOWN_HOLD = 17,
	GLYPH_SWIPE_DOWN_MASK = 18,
	GLYPH_CLICK = 19,
	GLYPH_CLICK_HOLD = 20,
	GLYPH_CLICK_MASK = 21,
	GLYPH_DOUBLE_CLICK = 22,
	GLYPH_DOUBLE_CLICK_MASK = 23,
	GLYPH_SLEEP = 24,
	GLYPH_SLEEP_MASK = 25,
	GLYPH_HEX_FONT = 26
} LCD_GLYPH_ID;

#define sgn(x) ((x<0)?-1:((x>0)?1:0)) /* macro to return the sign of a
                                         number */
#define abs(x) ((x<0)?(-x):x)

typedef enum {
	GS_WAIT = 0,
	GS_CLEAR = 1,
	GS_DRAW = 2
} GUI_GLYPH_STATE;

typedef enum {
	UI_INIT = 0,
	UI_STARTUP = 1,
	UI_LOGO = 2,
	UI_DRAW = 3,
	UI_CLEARSCREEN = 4
} LCD_UI_STATE;

typedef enum {
	TDM_NONE = 0,
	TDM_MAPONLY = 1,
	TDM_FULL = 2
} TOUCH_DISPLAY_MODE;

// buffer for the size of the lcd display
typedef struct _LCDBUFFER
{
	int guiGlyphTimer;
	BYTE guiGlyphOverlay;
	BYTE guiGlyphMask;
	GUI_GLYPH_STATE guiGlyphState;
	LCD_UI_STATE guiState;
	BYTE lcdData[8][128]; // 128 columns wide, 64 bits high
	BYTE changed[16]; // which areas of the sensor have changed
	BYTE map[16];
} LCDBUFFER;

typedef struct _LCDTOUCHREC
{
	UCHARCOORD loc[2];
	BYTE state;
} LCDTOUCHREC;

typedef struct _LCDTOUCHLOC
{
	LCDTOUCHREC Locs[MAX_TOUCHES];
} LCDTOUCHLOC;

typedef struct _GLYPHINFO
{
	BYTE width;
	BYTE height;
	int start;
} GLYPHINFO;

typedef struct _GLYPHSET
{
	BYTE ID;
	BYTE mask;
} GLYPHSET;

void lcdFrame(void);
void lcdInit(LCD_INIT_TYPE initType);
void lcdTest(void);
void lcdBufferClear(LCD_CLEAR_TYPE type);

void lcdSetGUIState(LCD_UI_STATE guiState);
void lcdSetOverlay(LCD_OVERLAY_ID overlayID, int timeout);
void lcdPutPoint(BYTE x, BYTE y, LCD_COLOR color);
void lcdPutLine(BYTE x1, BYTE y1, BYTE x2, BYTE y2, LCD_COLOR color);
void lcdPutGlyph(LCD_GLYPH_ID glyphID, BYTE x, BYTE y, BYTE gx, BYTE gy, BYTE dx, BYTE dy, GLYPH_COPY_TYPE copyStyle);
BYTE lcdPutString(LCD_STRING_ID stringID, BYTE x, BYTE y, GLYPH_COPY_TYPE copyStyle);
BYTE lcdPutStringRaw(const unsigned char *buffer, unsigned char strSize, BYTE x, BYTE y, GLYPH_COPY_TYPE copyStyle);
void lcdPutHexValue(BYTE hexValue, BYTE x, BYTE y);
void lcdBufferBlit(void);

void addTouch(BYTE ID, BYTE x, BYTE y, BYTE state, TOUCH_DISPLAY_MODE displayMode);

#endif // __LCDDISPLAY_H
