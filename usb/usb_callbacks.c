/*****************************************************************************
* CODE OWNERSHIP AND DISCLAIMER OF LIABILITY
* 
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in 
* all derivatives hereto.  You may use this code, and any derivatives created 
* by any person or entity by or on your behalf, exclusively with Microchip�s 
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
* 
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT 
* LIMITED TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND 
* FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH 
* MICROCHIP�S PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY 
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
#include "usb\USB.h"
#include "usb\usb_function_cdc.h"
#include "usb\HardwareProfile.h"
#include "main\main.h"

//
// Globals
//
extern USERRAM userRam;
extern COMMRAM commRam;

/******************************************************************************
 * Function:        void mySetLineCodingHandler(void)
 *
 * PreCondition:    USB_CDC_SET_LINE_CODING_HANDLER is defined
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function gets called when a SetLineCoding command
 *                  is sent on the bus.  This function will evaluate the request
 *                  and determine if the application should update the baudrate
 *                  or not.
 *
 * Note:            
 *
 *****************************************************************************/
#if defined(USB_CDC_SET_LINE_CODING_HANDLER)
void mySetLineCodingHandler(void)
{
    //If the request is not in a valid range
    if(cdc_notice.GetLineCoding.dwDTERate.Val > 115200)
    {
        //NOTE: There are two ways that an unsupported baud rate could be
        //handled.  The first is just to ignore the request and don't change
        //the values.  That is what is currently implemented in this function.
        //The second possible method is to stall the STATUS stage of the request.
        //STALLing the STATUS stage will cause an exception to be thrown in the 
        //requesting application.  Some programs, like HyperTerminal, handle the
        //exception properly and give a pop-up box indicating that the request
        //settings are not valid.  Any application that does not handle the
        //exception correctly will likely crash when this requiest fails.  For
        //the sake of example the code required to STALL the status stage of the
        //request is provided below.  It has been left out so that this demo
        //does not cause applications without the required exception handling
        //to crash.
        //---------------------------------------
        //USBStallEndpoint(0,1);
    }
    else
    {
		CDCSetBaudRate(cdc_notice.GetLineCoding.dwDTERate.Val);
        /*DWORD_VAL dwBaud;
		
        //Update the baudrate info in the CDC driver
        CDCSetBaudRate(cdc_notice.GetLineCoding.dwDTERate.Val);
        
        //Update the baudrate of the UART
        #if defined(__18CXX)
            dwBaud.Val = (GetSystemClock()/4)/line_coding.dwDTERate.Val-1;
            SPBRG = dwBaud.v[0];
            SPBRGH = dwBaud.v[1];
        #elif defined(__C30__)
            dwBaud.Val = (((GetPeripheralClock()/2)+(BRG_DIV2/2*line_coding.dwDTERate.Val))/BRG_DIV2/line_coding.dwDTERate.Val-1);
            U2BRG = dwBaud.Val;
        #elif defined(__C32__)
            U2BRG = ((GetPeripheralClock()+(BRG_DIV2/2*line_coding.dwDTERate.Val))/BRG_DIV2/line_coding.dwDTERate.Val-1);
            //U2MODE = 0;
            U2MODEbits.BRGH = BRGH2;
            //U2STA = 0;
        #endif*/
    }
}
#endif

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
* Function:        void USBCBSuspend(void)
*
* PreCondition:    None
*
* Input:           None
*
* Output:          None
*
* Side Effects:    None
*
* Overview:        Call back that is invoked when a USB suspend is detected
*
* Note:            None
*****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:
	
	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	
	

    #if defined(__C30__)
    #if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        Sleep();
    #endif
    #endif
}


/******************************************************************************
* Function:        void _USB1Interrupt(void)
*
* PreCondition:    None
*
* Input:           None
*
* Output:          None
*
* Side Effects:    None
*
* Overview:        This function is called when the USB interrupt bit is set
*					In this example the interrupt is only used when the device
*					goes to sleep when it receives a USB suspend command
*
* Note:            None
*****************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
* Function:        void USBCBWakeFromSuspend(void)
*
* PreCondition:    None
*
* Input:           None
*
* Output:          None
*
* Side Effects:    None
*
* Overview:        The host may put USB peripheral devices in low power
*					suspend mode (by "sending" 3+ms of idle).  Once in suspend
*					mode, the host may wake the device back up by sending non-
*					idle state signalling.
*					
*					This call back is invoked when a wakeup from USB suspend 
*					is detected.
*
* Note:            None
*****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
* Function:        void USBCB_SOF_Handler(void)
*
* PreCondition:    None
*
* Input:           None
*
* Output:          None
*
* Side Effects:    None
*
* Overview:        The USB host sends out a SOF packet to full-speed
*                  devices every 1 ms. This interrupt may be useful
*                  for isochronous pipes. End designers should
*                  implement callback routine as necessary.
*
* Note:            None
*******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
	//userRam.flag1 |= USBSOF; // indicate a USB interrupt occurred
}

/*******************************************************************
* Function:        void USBCBErrorHandler(void)
*
* PreCondition:    None
*
* Input:           None
*
* Output:          None
*
* Side Effects:    None
*
* Overview:        The purpose of this callback is mainly for
*                  debugging during development. Check UEIR to see
*                  which error causes the interrupt.
*
* Note:            None
*******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
* Function:        void USBCBCheckOtherReq(void)
*
* PreCondition:    None
*
* Input:           None
*
* Output:          None
*
* Side Effects:    None
*
* Overview:        When SETUP packets arrive from the host, some
* 					firmware must process the request and respond
*					appropriately to fulfill the request.  Some of
*					the SETUP packets will be for standard
*					USB "chapter 9" (as in, fulfilling chapter 9 of
*					the official USB specifications) requests, while
*					others may be specific to the USB device class
*					that is being implemented.  For example, a HID
*					class device needs to be able to respond to
*					"GET REPORT" type of requests.  This
*					is not a standard USB chapter 9 request, and 
*					therefore not handled by usb_device.c.  Instead
*					this request should be handled by class specific 
*					firmware, such as that contained in usb_function_hid.c.
*
* Note:            None
*******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckCDCRequest();
}//end


/*******************************************************************
* Function:        void USBCBStdSetDscHandler(void)
*
* PreCondition:    None
*
* Input:           None
*
* Output:          None
*
* Side Effects:    None
*
* Overview:        The USBCBStdSetDscHandler() callback function is
*					called when a SETUP, bRequest: SET_DESCRIPTOR request
*					arrives.  Typically SET_DESCRIPTOR requests are
*					not used in most applications, and it is
*					optional to support this type of request.
*
* Note:            None
*******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
* Function:        void USBCBInitEP(void)
*
* PreCondition:    None
*
* Input:           None
*
* Output:          None
*
* Side Effects:    None
*
* Overview:        This function is called when the device becomes
*                  initialized, which occurs after the host sends a
* 					SET_CONFIGURATION (wValue not = 0) request.  This 
*					callback function should initialize the endpoints 
*					for the device's usage according to the current 
*					configuration.
*
* Note:            None
*******************************************************************/
void USBCBInitEP(void)
{
    CDCInitEP();
}

/********************************************************************
* Function:        void USBCBSendResume(void)
*
* PreCondition:    None
*
* Input:           None
*
* Output:          None
*
* Side Effects:    None
*
* Overview:        The USB specifications allow some types of USB
* 					peripheral devices to wake up a host PC (such
*					as if it is in a low power suspend to RAM state).
*					This can be a very useful feature in some
*					USB applications, such as an Infrared remote
*					control	receiver.  If a user presses the "power"
*					button on a remote control, it is nice that the
*					IR receiver can detect this signalling, and then
*					send a USB "command" to the PC to wake up.
*					
*					The USBCBSendResume() "callback" function is used
*					to send this special USB signalling which wakes 
*					up the PC.  This function may be called by
*					application firmware to wake up the PC.  This
*					function should only be called when:
*					
*					1.  The USB driver used on the host PC supports
*						the remote wakeup capability.
*					2.  The USB configuration descriptor indicates
*						the device is remote wakeup capable in the
*						bmAttributes field.
*					3.  The USB host PC is currently sleeping,
*						and has previously sent your device a SET 
*						FEATURE setup packet which "armed" the
*						remote wakeup capability.   
*
*					This callback should send a RESUME signal that
*                  has the period of 1-15ms.
*
* Note:            Interrupt vs. Polling
*                  -Primary clock
*                  -Secondary clock***** MAKE NOTES ABOUT THIS *******
*                   > Can switch to primary first by calling USBCBWakeFromSuspend()
 
*                  The modifiable section in this routine should be changed
*                  to meet the application needs. Current implementation
*                  temporary blocks other functions from executing for a
*                  period of 1-13 ms depending on the core frequency.
*
*                  According to USB 2.0 specification section 7.1.7.7,
*                  "The remote wakeup device must hold the resume signaling
*                  for at lest 1 ms but for no more than 15 ms."
*                  The idea here is to use a delay counter loop, using a
*                  common value that would work over a wide range of core
*                  frequencies.
*                  That value selected is 1800. See table below:
*                  ==========================================================
*                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
*                  ==========================================================
*                      48              12          1.05
*                       4              1           12.6
*                  ==========================================================
*                 * These timing could be incorrect when using code
*                    optimization or extended instruction mode,
*                    or when having other interrupts enabled.
*                    Make sure to verify using the MPLAB SIM's Stopwatch
*                    and verify the actual signal on an oscilloscope.
*******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    USBResumeControl = 1;                // Start RESUME signaling
    
    delay_count = 1800U;                // Set RESUME line for 1-13 ms
    do
    {
        delay_count--;
    }while(delay_count);
    USBResumeControl = 0;
}


/*******************************************************************
* Function:        void USBCBEP0DataReceived(void)
*
* PreCondition:    ENABLE_EP0_DATA_RECEIVED_CALLBACK must be
*                  defined already (in usb_config.h)
*
* Input:           None
*
* Output:          None
*
* Side Effects:    None
*
* Overview:        This function is called whenever a EP0 data
*                  packet is received.  This gives the user (and
*                  thus the various class examples a way to get
*                  data that is received via the control endpoint.
*                  This function needs to be used in conjunction
*                  with the USBCBCheckOtherReq() function since 
*                  the USBCBCheckOtherReq() function is the apps
*                  method for getting the initial control transfer
*                  before the data arrives.
*
* Note:            None
*******************************************************************/
#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)
void USBCBEP0DataReceived(void)
{
}
#endif

/*******************************************************************
* Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
*                        USB_EVENT event, void*pdata, WORD size)
*
* PreCondition:    None
*
* Input:           USB_EVENT event - the type of event
*                  void*pdata - pointer to the event data
*                  WORD size - size of the event data
*
* Output:          None
*
* Side Effects:    None
*
* Overview:        This function is called from the USB stack to
*                  notify a user application that a USB event
*                  occured.  This callback is in interrupt context
*                  when the USB_INTERRUPT option is selected.
*
* Note:            None
*******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
#ifdef USB
	char usbOutData[CDC_DATA_IN_EP_SIZE];
	unsigned char usbCount;
	unsigned char x;

    switch(event)
    {
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER:
			//
			// First verify we are in a configured state
			//
			if((USBDeviceState == CONFIGURED_STATE)&&(USBSuspendControl==0))
			{
				// 
				// check for OUT on EP3
				//
				//if(((*(BYTE*)pdata) & ENDPOINT_MASK) == 0x30 && ((*(BYTE*)pdata) & 0x08) == 0x00)
				if(((*(BYTE*)pdata) & 0x38) == 0x30)
				{
					/*usbCount = getsUSBUSART(usbOutData,64);
					commRam.usbBuffer[commRam.loadIndex] = usbOutData[0];
					commRam.loadIndex++;		// Circular FIFO, increment load index
					commRam.loadIndex&=0x07;	// Currently only 8 bytes deep*/
					usbCount = getsUSBUSART(usbOutData,64);
					for (x=0;x<usbCount;x++)
					{
						commRam.usbBuffer[commRam.loadIndex] = usbOutData[x];
						commRam.loadIndex++;		// Circular FIFO, increment load index
						commRam.loadIndex&=0x0f;	// Currently only 16 bytes deep
					}
				}
			}
            break;
        default:
            break;
    }      
    return TRUE; 
#endif
	return FALSE;
}
