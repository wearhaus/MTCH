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
	TSCG				10/07/10


*/
#include "usb\usb_function_cdc.h"
#include "usb\usb_common.h"
#include "usb\usb_config.h"
#include "main\main.h"

//
// Globals
//
extern COMMRAM commRam;
extern VARRAM varRam;
extern volatile short globalCounter;

//void sendUSBState(void);
/******************************************************************************
* Function:        	void initUSB(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Initializes the USB
*
* Note:            	None		
*			
*****************************************************************************/
void initUSB(void)
{
#ifdef USB
	//IPC21bits.USB1IP = 7;
	USBDeviceInit();	// initialize the stack
	#ifdef USB_INTERRUPT
	USBDeviceAttach();
	#endif
	commRam.rcvCount = 0;
	commRam.rcvBuffer[0] = 0;
	//
	// init comm FIFO
	//
	commRam.loadIndex = 0;
	commRam.retrieveIndex = 0;
	varRam.flag &= NOTINTERRUPTOCCURRED;
	while(USBDeviceState < CONFIGURED_STATE)
	{}
	//
	// run an additinal 400ms delay to let USB finish enumeration
	//
	//
	// Setup TMR1 32us, timeout of 50ms
	//
	globalCounter = 50;
	while(globalCounter > 0)
	{}
//	T5CONbits.TON = 0;	// turn off TMR1
#endif
}

void sendUSBChar(unsigned char *data, BYTE  length)
{
#ifdef USB
	if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1))
	{
		return;
	}
	if (length > 0)
	{
		//
		// Setup TMR1 32us, timeout of 5ms
		//
		globalCounter = 5;
		while(!USBUSARTIsTxTrfReady())
		{
			if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1))
			{
				return;
			}
			CDCTxService();
			if (globalCounter == 0)
			{
				break;
			}
		}
		if(USBUSARTIsTxTrfReady())
		{
			putUSBUSART((char *)data, length);
		}
	}
	CDCTxService();
	//sendUSBState();
#endif
}

/******************************************************************************
* Function:        	void usbReceive(void)
*
* PreCondition:    	None
*
* Input:           	None
*
* Output:          	None
*
* Side Effects:    	None
*
* Overview:        	Retrieves received data if avialble from USB EP3 OUT
*
* Note:            	
*
*****************************************************************************/
void usbReceive(void)
{
#ifdef USB
	if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1))
	{
		return;
	}
	if (commRam.loadIndex == commRam.retrieveIndex)
	{
		return; // No data
	}


	if (commRam.rcvCount > RCVBUFFERSIZE)
	{
		commRam.rcvCount = 0;
	}
	/*commRam.rcvBuffer[commRam.rcvCount] = commRam.usbBuffer[commRam.retrieveIndex];
	commRam.rcvCount ++;
	commRam.retrieveIndex++;
	commRam.retrieveIndex&=0x07;*/
	while (commRam.loadIndex != commRam.retrieveIndex)
	{
		commRam.rcvBuffer[commRam.rcvCount] = commRam.usbBuffer[commRam.retrieveIndex];
		commRam.rcvCount ++;
		commRam.rcvCount &= 0x0f;
		commRam.retrieveIndex++;
		commRam.retrieveIndex&=0x0f;
	}
#endif
}

void sendUSBState(void)
{
#ifdef USB
    if(USBSuspendControl == 1)
    {
        while(!U1STAbits.TRMT)
		{}
		U1TXREG = 0x01;
    }
    else
    {
        if(USBDeviceState == DETACHED_STATE)
        {
            while(!U1STAbits.TRMT)
			{}
			U1TXREG = 0x02;
        }
        else if(USBDeviceState == ATTACHED_STATE)
        {
            while(!U1STAbits.TRMT)
			{}
			U1TXREG = 0x03;
        }
        else if(USBDeviceState == POWERED_STATE)
        {
            while(!U1STAbits.TRMT)
			{}
			U1TXREG = 0x04;
        }
        else if(USBDeviceState == DEFAULT_STATE)
        {
            while(!U1STAbits.TRMT)
			{}
			U1TXREG = 0x05;
        }
        else if(USBDeviceState == ADDRESS_STATE)
        {
            while(!U1STAbits.TRMT)
			{}
			U1TXREG = 0x06;
        }
        else if(USBDeviceState == CONFIGURED_STATE)
        {
            while(!U1STAbits.TRMT)
			{}
			U1TXREG = 0x07;
        }//end if(...)

		if(cdc_trf_state == CDC_TX_READY)
		{
			while(!U1STAbits.TRMT)
			{}
			U1TXREG = 0x08;
		}
    }//end if(UCONbits.SUSPND...)
#endif
}//end BlinkUSBStatus


