/**********************************************************************************
 * PROJECT: USB ROBOTNIK
 * Adapted to Brain Board
 * 
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
 * 8-23-19: DMA for TX: got rid of bugs - works great at 921600 baud
 * 8-25-19: Works receiving/sending pot data from VC++ Robotnik Controller
 * 8-26-19: Verified receiving/sending 100 servos.
 * 9-1-19:  Works with Robotnik Brain Board and VC++ Robotnik Controller recording/playing four servo motors.
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

#define RS485uart UART5
#define RS485bits U5STAbits
#define RS485_VECTOR _UART_5_VECTOR

#define USE_RS485

#ifdef USE_RS485  
    #define UART_TX_REG U5TXREG
    #define UART_TX_IRQ _UART5_TX_IRQ   
    #define RS485_Control LATBbits.LATB0
#else
   #define UART_TX_REG U2TXREG
   #define UART_TX_IRQ _UART2_TX_IRQ
#endif

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
unsigned char RS485TxBuffer[MAXBUFFER+1] = "Starting RS485 TX Message\r";
short RS485TxIndex = 0;
unsigned char ADready = false;
#define MAXPOTS 4
unsigned int ADresult[MAXPOTS];

DmaChannel	DmaUARTChannel = DMA_CHANNEL1;	// DMA channel to use for our example
						// NOTE: the ISR setting has to match the channel number


/** P R I V A T E  P R O T O T Y P E S ***************************************/
unsigned short decodePacket(unsigned char *ptrInPacket, unsigned char *ptrData);
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
    unsigned char ch;
    InitializeSystem();
    DelayMs(100);
    
    RS485TxIndex = 0;        
    ch = RS485TxBuffer[RS485TxIndex++];
    while (!UARTTransmitterIsReady(RS485uart));
    UARTSendDataByte(RS485uart, ch);
    
    INTEnable(INT_SOURCE_UART_TX(RS485uart), INT_ENABLED);
        
    printf("\r#2 Testing USB TX/RX and DMA TX using RS485\r");
    
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
    
    PORTSetPinsDigitalOut(IOPORT_B, BIT_0);  // RB0 = RS485 control
    RS485_Control = 1;
    
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
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);
        
#ifdef USE_RS485    
    // Set up RS485 UART    
    UARTConfigure(RS485uart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(RS485uart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(RS485uart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(RS485uart, SYS_FREQ, 921600);
    // UARTSetDataRate(RS485uart, SYS_FREQ, 2000000);
    UARTEnable(RS485uart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure RS485 UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(RS485uart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(RS485uart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(RS485uart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(RS485uart), INT_SUB_PRIORITY_LEVEL_0);  
#endif        

    //PORTClearBits(IOPORT_B, BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);
    //mPORTBSetPinsDigitalOut(BIT_0 | BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);

    // PORTSetPinsDigitalIn(IOPORT_B, BIT_1 | BIT_2);
    // PORTSetPinsDigitalOut(IOPORT_B, BIT_0);
    
    PORTSetBits(IOPORT_E, BIT_0);
    PORTSetPinsDigitalIn(IOPORT_D, BIT_15);  // Initialize FAULT signal input   
    
    //PORTSetPinsDigitalIn(IOPORT_B, BIT_1 | BIT_4 | BIT_2);
    //mCNOpen(CN_ON, CN6_ENABLE | CN4_ENABLE, CN6_PULLUP_ENABLE | CN4_PULLUP_ENABLE);
    //ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);
    
    
   	// now the TX part
	// reconfigure the channel
	DmaChnOpen(DmaUARTChannel, DMA_CHN_PRI2, DMA_OPEN_MATCH);
    DmaChnSetMatchPattern(DmaUARTChannel, '\r');	// set \r as ending character    
    
	// set the events: now the start event is the UART tx being empty
	// we maintain the pattern match mode
	DmaChnSetEventControl(DmaUARTChannel, DMA_EV_START_IRQ_EN|DMA_EV_MATCH_EN|DMA_EV_START_IRQ(UART_TX_IRQ));
	// set the transfer source and dest addresses, source and dest size and cell size     
	DmaChnSetTxfer(DmaUARTChannel, RS485TxBuffer, (void*)&UART_TX_REG, 256, 1, 1);
	DmaChnSetEvEnableFlags(DmaUARTChannel, DMA_EV_BLOCK_DONE);		// enable the transfer done interrupt: pattern match or all the characters transferred
    
	INTSetVectorPriority(INT_VECTOR_DMA(DmaUARTChannel), INT_PRIORITY_LEVEL_5);		// set INT controller priority
	INTSetVectorSubPriority(INT_VECTOR_DMA(DmaUARTChannel), INT_SUB_PRIORITY_LEVEL_3);		// set INT controller sub-priority
	INTEnable(INT_SOURCE_DMA(DmaUARTChannel), INT_ENABLED);		// enable the DmaUARTChannel interrupt in the INT controller    
	
	// DmaChnStartTxfer(DmaUARTChannel, DMA_WAIT_NOT, 0);	// force the DMA transfer: the UART2 tx flag it's already been active
	
	
	// DMA Echo is complete
	INTEnable(INT_SOURCE_DMA(DmaUARTChannel), INT_DISABLED);		// disable further interrupts from the DMA controller        
    
    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();
	// lastTransmission = 0;

    USBDeviceInit();	// Initializes USB module SFRs and firmware
    					// variables to known states.            
}//end UserInit


// handler for the DMA channel 1 interrupt
void __ISR(_DMA1_VECTOR, IPL5SOFT) DmaHandler1(void)
{
	int	evFlags;				// event flags when getting the interrupt
	INTClearFlag(INT_SOURCE_DMA(DmaUARTChannel));	// release the interrupt in the INT controller, we're servicing int
	evFlags = DmaChnGetEvFlags(DmaUARTChannel);	// get the event flags
    if(evFlags & DMA_EV_BLOCK_DONE)
    { 
        // just a sanity check. we enabled just the DMA_EV_BLOCK_DONE transfer done interrupt
        DmaChnClrEvFlags(DmaUARTChannel, DMA_EV_BLOCK_DONE);
        // RS485TxBufferFull = false;
        RS485TxBuffer[0] = '\0';
    }
}

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

void ProcessIO(void)
{   
    int numBytes = 0;     
    unsigned short length, i;     
    static unsigned short packetLength = 0;
    unsigned char USBNewRxBuffer[64];  
    static unsigned char USBRxBuffer[MAXBUFFER] = "\0";
    static unsigned char USBBufferFull = false;
    unsigned char ch;
    unsigned char TestBuffer[64] = "";
    
    //Blink the LEDs according to the USB device status
    
    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) 
    {
        LED_OUT = 0;
        return;
    }   
    
    numBytes = getsUSBUSART(USBNewRxBuffer, 64); //until the buffer is free.            
    if (numBytes > 0)
    {	        
        if (USBBufferFull) printf("\rOVERLAP ERROR");
        for (i = 0; i < numBytes; i++)
        {
            ch = USBNewRxBuffer[i];
            USBRxBuffer[i+packetLength] = ch;
            if (ch == ETX) USBBufferFull = true;
        }
        packetLength = packetLength + numBytes;
        if (USBBufferFull)
        {
            for (i = 0; i < packetLength; i++) RS485TxBuffer[i] = USBRxBuffer[i];
            packetLength = 0;            
        }        
    }	      

    // Check if any bytes are waiting in the queue to send to the USB host.
    // If any bytes are waiting, and the endpoint is available, prepare to
    // send the USB packet to the host.    
    if (USBUSARTIsTxTrfReady() && USBBufferFull)
    {        
        if (LED_OUT) LED_OUT = 0;
        else LED_OUT = 1;
        length = sprintf(TestBuffer, ">0 %d %d %d %d\r", ADresult[0], ADresult[1], ADresult[2], ADresult[3]);
        mAD1IntEnable(INT_ENABLED);       
        
        if (length < 64) putUSBUSART(TestBuffer, length);        		
        else putUSBUSART("ERROR\r", 6);        
        
        //length = strlen(USBRxBuffer);
        // for (i = 0; i < length; i++) RS485TxBuffer[i] = USBRxBuffer[i];
        USBRxBuffer[0] = '\0';
        USBBufferFull = false;        
        
        DmaChnSetTxfer(DmaUARTChannel, RS485TxBuffer, (void*)&UART_TX_REG, 256, 1, 1);        
        DmaChnStartTxfer(DmaUARTChannel, DMA_WAIT_NOT, 0);	// force the DMA transfer: the UART2 tx flag it's already been active       	        
        
        //RS485TxIndex = 0;        
        //ch = RS485TxBuffer[RS485TxIndex++];
        //while (!UARTTransmitterIsReady(RS485uart));
        //UARTSendDataByte(RS485uart, ch);
        //INTEnable(INT_SOURCE_UART_TX(RS485uart), INT_ENABLED);
    }
    
    CDCTxService();
    
}//end ProcessIO


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

// RS485 UART interrupt handler it is set at priority level 2
void __ISR(RS485_VECTOR, ipl2) IntRS485UartHandler(void) {
    unsigned char ch;

   if (INTGetFlag(INT_SOURCE_UART_RX(RS485uart))) 
        INTClearFlag(INT_SOURCE_UART_RX(RS485uart));
        
    if (INTGetFlag(INT_SOURCE_UART_TX(RS485uart))) 
    {
        INTClearFlag(INT_SOURCE_UART_TX(RS485uart));
        ch = RS485TxBuffer[RS485TxIndex++];
        while (!UARTTransmitterIsReady(RS485uart));
        UARTSendDataByte(RS485uart, ch);
        if (ch == '\r' || RS485TxIndex > MAXBUFFER) 
            INTEnable(INT_SOURCE_UART_TX(RS485uart), INT_DISABLED);
    }
}

/*
unsigned short decodePacket(unsigned char *ptrInPacket, unsigned char *ptrData) 
{
    unsigned short i, j;
    unsigned char escapeFlag = FALSE;
    unsigned char startFlag = false;
    unsigned char ch;

    j = 0;
    for (i = 0; i < MAXBUFFER; i++) 
    {
        ch = ptrInPacket[i];
        // Escape flag not active
        if (!escapeFlag) 
        {
            if (ch == STX) 
            {
                if (!startFlag) 
                {
                    startFlag = true;
                    j = 0;
                }
                else return (0);
            } 
            else if (ch == ETX) 
                return (j);
            else if (ch == DLE)
                escapeFlag = TRUE;
            else if (j < MAXBUFFER)
                ptrData[j++] = ch;
            else return (0);
        } 
        // Escape flag active
        else 
        {
            escapeFlag = FALSE;
            if (ch == ETX-1) ch = ETX;            
            if (j < MAXBUFFER)
                ptrData[j++] = ch;
            else return(0);
        }
    }
    return (j);
}
*/