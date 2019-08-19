/**********************************************************************************
 * FileName: main.c Adapted from Microchip CDC serial emulator
 * Compiled for PIC32MX795 XC32 compiler version 1.30
 * 
 * Jim Sedgwick 7-29-18   
 * Separated UART and USB buffers.
 * 8-24-18: Implemented simple USB-UART bridge.
 * 8-25-18: Combined SD card writes and reads accepting data from both UART and USB
 * 9-2-18: Wrote code for MIDI record and play.
 * 9-3-18: Records and plays back MIDI on SD card. Uses serial MIDI UART on UBW32 board.
 * 8-19-19: Works nicely with VC++ Robotnik Controller at 961200 baud.
 *          Copied project to USB MIDI Robotnik, stripped all MIDI stuff.
 ***********************************************************************************/
#include <xc.h>
#include "./USB/usb.h"
#include "./USB/usb_function_cdc.h"
#include "HardwareProfile.h"
#include "./uart2.h"
#include "Delay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select

/** I N C L U D E S **********************************************************/

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb.h"
#include "HardwareProfile.h"
#include <ctype.h>

#include "FSIO.h"
#include "Delay.h"
#include "Defs.h"

#define LED_OUT LATEbits.LATE6
#define TEST_OUT LATEbits.LATE3

// UART FOR PC SERIAL PORT
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

#define MIDIuart UART1
#define MIDIbits U1STAbits
#define MIDI_VECTOR _UART_1_VECTOR


#define CR 13
#define LF 10
#define BACKSPACE 8
#define SPACE 32
#define ESCAPE 27
#define RIGHT_ARROW 67
#define LEFT_ARROW 68
#define UP_ARROW 65
#define DOWN_ARROW 66

#define false FALSE
#define true TRUE
/** V A R I A B L E S ********************************************************/
// #define MAXBUFFER CDC_DATA_IN_EP_SIZE
#define MAXBUFFER 256
unsigned char HOSTRxBuffer[MAXBUFFER+1];
unsigned char HOSTRxBufferFull = false;
unsigned char HOSTTxBuffer[MAXBUFFER+1];
unsigned char HOSTTxBufferFull = false;
unsigned char ADready = false;
#define MAXPOTS 4
unsigned int ADresult[MAXPOTS];

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void UserInit(void);
void ConfigAd(void);

int main(void)
{   
    InitializeSystem();
    DelayMs(100);
    printf("\rTesting POTS\r");
    
    while(1)
    {
    #if defined(USB_POLLING)
      USBDeviceTasks();
        #endif
        ProcessIO();                        
    }
}//end main

//#define mDataRdyUSART() UART2IsPressed()
// #define mTxRdyUSART()   U2STAbits.TRMT

void USBCBSuspend(void)
{
}
void USBCBWakeFromSuspend(void)
{
}
void USBCB_SOF_Handler(void)
{
}
void USBCBErrorHandler(void)
{
}
void USBCBCheckOtherReq(void)
{
    USBCheckCDCRequest();
}//end

void USBCBStdSetDscHandler(void)
{
}//end
void USBCBInitEP(void)
{
    CDCInitEP();
}

void USBCBSendResume(void)
{
    static WORD delay_count;
    
    if(USBGetRemoteWakeupStatus() == TRUE) 
    {
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;  //So we don't execute this code again, 
            delay_count = 3600U;        
            do
            {
                delay_count--;
            } while(delay_count);
            
            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            } while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)
void USBCBEP0DataReceived(void)
{
}
#endif

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
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
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            break;
        default:
            break;
    }      
    return TRUE; 
}





void InitializeSystem(void) 
{
	unsigned char i; 
    
    SYSTEMConfigPerformance(80000000);
    
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif        
    
    // UserInit();
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);
    
    ConfigAd();
    
    PORTSetPinsDigitalOut(IOPORT_E, BIT_3 | BIT_6);    // RE3 = TEST_OUT, RE6 = LED_OUT    
    TEST_OUT = 0;
    LED_OUT = 0;
    
    // Set up Timer 2 for 1 millisecond
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_64, 1250);
    // set up the core timer interrupt with a priority of 5 and zero sub-priority
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_5);    
    
    
    #define SYS_FREQ 80000000    
    
    // Set up HOST UART    
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 921600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);

    //PORTClearBits(IOPORT_B, BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);
    //mPORTBSetPinsDigitalOut(BIT_0 | BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);

    // PORTSetPinsDigitalIn(IOPORT_B, BIT_1 | BIT_2);
    // PORTSetPinsDigitalOut(IOPORT_B, BIT_0);
    
    PORTSetBits(IOPORT_E, BIT_0);
    PORTSetPinsDigitalIn(IOPORT_D, BIT_15);  // Initialize FAULT signal input   
    
    //PORTSetPinsDigitalIn(IOPORT_B, BIT_1 | BIT_4 | BIT_2);
    //mCNOpen(CN_ON, CN6_ENABLE | CN4_ENABLE, CN6_PULLUP_ENABLE | CN4_PULLUP_ENABLE);
    //ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);
    
    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

	// lastTransmission = 0;

    USBDeviceInit();	// Initializes USB module SFRs and firmware
    					// variables to known states.            
}//end UserInit




/******************************************************************************
 *	Change Notice Interrupt Service Routine
 *
 *   Note: Switch debouncing is not performed.
 *   Code comes here if SW2 (CN16) PORTD.RD7 is pressed or released.
 *   The user must read the IOPORT to clear the IO pin change notice mismatch
 *	condition first, then clear the change notice interrupt flag.
 ******************************************************************************/

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) {
/*    
    // Step #1 - always clear the mismatch condition first
    // dummy = PORTReadBits(IOPORT_B, BIT_4 | BIT_2) & 0x0F;
    PortBRead = PORTB;// & 0x14;
    PortBflag = true;

    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();

    // Step #3 - process the switches
    if ((PortBRead & 0x10) == 0)
    {
        BlueButtonPushed = true;
        ButtonsReleased = false;
    }
    else if ((PortBRead & 0x04) == 0) 
    {
        YellowButtonPushed = true;
        ButtonsReleased = false;
    }
    else ButtonsReleased = true;
*/ 
}



// HOST UART interrupt handler it is set at priority level 2
void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    unsigned char ch;
    static unsigned short HOSTRxIndex = 0;
    static unsigned char arrowIndex = 0;
    static unsigned char arrArrow[3];
    int i;

    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = UARTGetDataByte(HOSTuart);
            {
                if (ch == LF || ch == 0);
                else if (ch == BACKSPACE) 
                {
                    while (!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte(HOSTuart, ' ');
                    while (!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte(HOSTuart, BACKSPACE);
                    if (HOSTRxIndex > 0) HOSTRxIndex--;
                } 
                else if (ch == CR) 
                {
                    if (HOSTRxIndex < (MAXBUFFER-1)) 
                    {
                        HOSTRxBuffer[HOSTRxIndex] = CR;
                        HOSTRxBuffer[HOSTRxIndex + 1] = '\0'; 
                        HOSTRxBufferFull = true;
                    }
                    HOSTRxIndex = 0;
                }                
                else 
                {
                    if (HOSTRxIndex < (MAXBUFFER-1))
                        HOSTRxBuffer[HOSTRxIndex++] = ch;                    
                }
            }
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
}


void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) 
{
    static short milliCounter = 0;
    mT2ClearIntFlag(); // clear the interrupt flag
    
    milliCounter++;
    if (milliCounter >= 30)
    {
        milliCounter = 0;
    }
}


void ProcessIO(void)
{   
    static int bytesReceived = 0;     
    int length, i;       
    unsigned char TestBuffer[64];
    static unsigned char USBreceived = false;
    unsigned char USBReceivedData[64];    
    
    //Blink the LEDs according to the USB device status
    
    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) 
    {
        LED_OUT = 0;
        return;
    }   
    

    // if (!HOSTTxBufferFull)
    {
        bytesReceived = getsUSBUSART(USBReceivedData, 64); //until the buffer is free.    
        
        if (bytesReceived > 0)
        {	            
            USBReceivedData[bytesReceived] = '\0';            
            length = strlen(HOSTTxBuffer) + bytesReceived;            
            if (length <= MAXBUFFER) strcat(HOSTTxBuffer, USBReceivedData);            
            if (strchr(USBReceivedData, '\r')) 
            {                
                HOSTTxBufferFull = true;    
                USBreceived = true;
            }                
        }	        
    }
    
    if (USBreceived)
    {
        if (LED_OUT) LED_OUT = 0;
        else LED_OUT = 1;
        length = sprintf(TestBuffer, ">%d %d %d %d<\r", ADresult[0], ADresult[1], ADresult[2], ADresult[3]);
        mAD1IntEnable(INT_ENABLED);
    }

    // Check if any bytes are waiting in the queue to send to the USB host.
    // If any bytes are waiting, and the endpoint is available, prepare to
    // send the USB packet to the host.
    
    if (USBUSARTIsTxTrfReady() && USBreceived)
    {        
        USBreceived = false;  
        if (length < 64) putUSBUSART(TestBuffer, length);        		
        else putUSBUSART("ERROR\r", 6);        
	}
    
    if (HOSTTxBufferFull)
    {
        printf("%s", HOSTTxBuffer);
        HOSTTxBuffer[0] = '\0';
        HOSTTxBufferFull = false;
    }
    
    CDCTxService();
    
}//end ProcessIO


void ConfigAd(void) 
{
    // mPORTBSetPinsAnalogIn(BIT_12 | BIT_13 | BIT_14 | BIT_15); // $$$$
    mPORTBSetPinsAnalogIn(BIT_1 | BIT_2 | BIT_3 | BIT_4); // $$$$

    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
    // #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31
#define PARAM3  ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_31 | ADC_CONV_CLK_32Tcy

/*
#define PARAM4    ENABLE_AN12_ANA | ENABLE_AN13_ANA | ENABLE_AN14_ANA | ENABLE_AN15_ANA
#define PARAM5 SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 |\
SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 |\
SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11   
*/
    

#define PARAM4    ENABLE_AN1_ANA | ENABLE_AN2_ANA | ENABLE_AN3_ANA | ENABLE_AN4_ANA    
#define PARAM5 SKIP_SCAN_AN0 |\
SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 |\
SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 |\
SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15


    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();
}

void __ISR(_ADC_VECTOR, ipl6) ADHandler(void) 
{
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        ADresult[i] = (unsigned short) ReadADC10(offSet + i); // read the result of channel 0 conversion from the idle buffer
    ADready = true;
}

