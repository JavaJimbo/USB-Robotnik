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
 * 
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

#define TEST_OUT LATBbits.LATB0


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
#define MAXBUFFER 64
char USBReceivedData[MAXBUFFER+1];
char USBRXBuffer[MAXBUFFER+1];
unsigned char HOSTRxBuffer[MAXBUFFER+1];
unsigned char HOSTRxBufferFull = false;
unsigned char MemoryBufferFull = false;
unsigned char HOSTTxBuffer[MAXBUFFER+1];
unsigned char HOSTTxBufferFull = false;
unsigned char MIDITxBuffer[MAXBUFFER+1];
unsigned char MIDIRxBuffer[MAXBUFFER+1];
unsigned short MIDIRxIndex = 0, MIDIRxPtr = 0, RxIndex = 0;
unsigned short MIDITxIndex = 0, MIDITxPtr = 0;
unsigned char tempBuffer[MAXBUFFER+1];
unsigned char MemoryBuffer[MAXBUFFER+1];

unsigned char outMessage[] = "\rJust putzing around";

short MIDItimeout = 0;


/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);
void InitializeUSART(void);
void putcUSART(char c);
unsigned char getcUSART ();

unsigned char BlueButtonPushed = false;
unsigned char YellowButtonPushed = false;
unsigned char ButtonsReleased = true;
unsigned char command = 0;
unsigned char mode = 0;

enum
{
    STANDBY = 0,
    RECORD,
    PLAY,
    PAUSE_RECORD,
    PAUSE_PLAY,
    HALT
};

short time, seconds = 0, minutes = 0, hundredths = 0;

/*
int strcat_s (char *strDest, int MaxBufferSize, char *strSource)
{
    char ch;
    short i = 0, j = 0;
    do {
        ch = strDest[i++];
    } while (ch != '\0' && i < MAXBUFFER);    
    if (i >= MaxBufferSize) 
    {
        printf("\r\nNo 0");
        return (1);
    }
    i--;
    j = 0;
    do {
        ch = strSource[j++];        
        strDest[i++] = ch;
    } while ((ch != '\0') && (i < MaxBufferSize-1) && (j < MaxBufferSize));    
    strDest[i] = '\0';
    return (0);
}
*/
/*
int strlen(char *ptrString)
{
    int i = 0;
    char ch;
    do {
      ch = ptrString[i++];  
    } while (ch != '\0' && i < MAXBUFFER);
    if (i == MAXBUFFER) return 0;
    else return i;
}
*/

int main(void)
{   
unsigned short i = 0, j = 0, numBytes;
FSFILE *filePtr;
char filename[] = "TestFile.txt";
unsigned char ch;    
char strMIDI[MAXBUFFER] = "";
short stringIndex = 0;
char strLine[MAXBUFFER+1];
unsigned char nextLineLoaded = false;
short nextTime = 0;
char strSeconds[3], strMinutes[3], strHundredths[3];
char strMEM[MAXBUFFER] = "\0";
char strMIDIbyte[MAXBUFFER] = "\0";
unsigned char MIDInote = 0x24;
unsigned char MIDIbyte;
short startPos = 0;
char strHEX[3];
short length = 0;

    InitializeSystem();
    DelayMs(100);    
    printf("\r\nTesting MIDI RECORDING\r\n");
    
/*    
#ifdef USE_UBW32    
    printf("\r\r\rSTARTING SD card demo for UBW32 board");
#else     
    printf("\r\r\rSTARTING SD card demo for SNAD PIC board");
#endif
    printf("\rWait for Media Detect...");
    while (!MDD_MediaDetect());     // Wait for SD detect to go low    
    printf("\rInitializing SD card...");
    while (!FSInit());
    printf("\rOpening test file...");
    filePtr = FSfopen(filename, FS_READ);            
    if (filePtr==NULL) printf("Error: could not open %s", filename);
    else
    {
        printf("\rSuccess! Opened %s. Reading data\r", filename);    
        do {           
            numBytes = FSfread(&ch, 1, 1, filePtr);
            putchar(ch);
        } while (numBytes);    
        printf("\rClosing file");
        FSfclose(filePtr); 
        printf("\rDONE");        
    }   
    DelayMs(10);        // Initialize SD card       
    printf("\rTESTING WITH MEMORY BUFFER");
    */
    
    DelayMs(200);

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif

    strMIDI[0] = '\0';
    while(1)
    {            
        while (MIDIRxPtr != MIDIRxIndex && mode == STANDBY)
        {            
            ch = MIDIRxBuffer[MIDIRxPtr++];            
            if (MIDIRxPtr >= MAXBUFFER) MIDIRxPtr = 0;
            printf("%02X ", ch);
        }        
        if (mode == RECORD)
        {        
            length = 0;
            if (MIDIRxPtr != MIDIRxIndex)
            { 
                strMIDI[0] = '\0';
                do{
                    if (MIDIRxPtr < MAXBUFFER) 
                    {
                        ch = MIDIRxBuffer[MIDIRxPtr];  
                        sprintf(strMIDIbyte, " %02X", ch);
                        strcat(strMIDI, strMIDIbyte); 
                    }
                    MIDIRxPtr++;
                    if (MIDIRxPtr >= MAXBUFFER) MIDIRxPtr = 0;
                } while (MIDIRxPtr != MIDIRxIndex);                
                sprintf(strMEM, "[%02d:%02d:%02d]", minutes, seconds, hundredths);
                strcat(strMEM, strMIDI); 
                strcat(strMEM, "\r\n"); 
                length = strlen(strMEM); 
                printf("%s", strMEM, length);   
                FSfwrite(strMEM, 1, length, filePtr);                                                     
            }
        }
        if (mode == PLAY)
        {
            if (nextLineLoaded)
            {
                if (time >= nextTime) 
                {         
                    printf("%s", strLine);                    
                    if (MIDITxIndex)
                    {
                        i = 0;
                        do {
                            if (i < MAXBUFFER)
                            {
                                MIDIbyte = MIDITxBuffer[i];
                                while(!UARTTransmitterIsReady(MIDIuart));
                                UARTSendDataByte (MIDIuart, MIDIbyte);
                            }
                            i++;
                        } while(i < MIDITxIndex);
                    }
                    nextLineLoaded = false;
                }
            }
            else
            {                                
                ch = 0;
                stringIndex = 0;
                startPos = 0;
                do {           
                    numBytes = FSfread(&ch, 1, 1, filePtr);
                    if (!numBytes) break;
                    if (stringIndex < MAXBUFFER) strLine[stringIndex] = ch;
                    else break;
                    if (ch == '[') startPos = stringIndex;
                    stringIndex++;
                } while (ch != '\n');  
                strLine[stringIndex] = '\0';
                
                if (numBytes == 0)
                {
                    mode = STANDBY;
                    printf("\r\n END OF FILE - STANDBY MODE");
                }
                else if (i >= MAXBUFFER) 
                {
                    mode = STANDBY;
                    printf("\r\nERROR: no '['");
                }                
                else if (stringIndex <= 0)
                {
                    mode = STANDBY;
                    printf("\r\nTIME FORMAT ERROR #1");
                }                
                else if (strLine[startPos] != '[' || strLine[startPos+3] != ':' || strLine[startPos+6] != ':' || strLine[startPos+9] != ']')
                {
                    mode = STANDBY;
                    printf("\r\nTIME FORMAT ERROR #2");
                }                
                else
                {
                    // 0  1 2 3 4 5 6 7 8  9
                    // [  0 0 : 0 0 : 0 0  ]       
                    strMinutes[0] = strLine[startPos+1];
                    strMinutes[1] = strLine[startPos+2];
                    strMinutes[2] = '\0';
                    
                    strSeconds[0] = strLine[startPos+4];
                    strSeconds[1] = strLine[startPos+5];
                    strSeconds[2] = '\0';

                    strHundredths[0] = strLine[startPos+7];
                    strHundredths[1] = strLine[startPos+8];
                    strHundredths[2] = '\0';   
                    
                    short nextMinutes, nextSeconds, nextHundredths;
                    
                    nextMinutes = atoi(strMinutes);
                    nextSeconds = atoi(strSeconds);
                    nextHundredths = atoi(strHundredths);
                    
                    nextTime = (nextMinutes * 60 * 100) + (nextSeconds * 60) + nextHundredths;
                    
                    nextLineLoaded = true;
                        
                    i = 10;
                    MIDITxIndex = 0;
                    do {                        
                        strHEX[0] = strHEX[1] = strHEX[2] = '\0';
                        ch = strLine[i++];
                        if (isxdigit(ch)) 
                        {
                            strHEX[0] = ch;
                            ch = strLine[i++];
                            if (isxdigit(ch)) strHEX[1] = ch;
                            MIDITxBuffer[MIDITxIndex] = strtol (strHEX, NULL, 16);                            
                            MIDITxIndex++;
                        }                        
                    } while (ch != '\n' && i < MAXBUFFER && MIDITxIndex < MAXBUFFER);
                }
            }
        }       
        
        if (command) 
        {
            // printf("\r\nCOMMAND: %d", command);
            switch (command)
            {
                case 18:
                    printf("Wait for Media Detect...\r\n");
                    while (!MDD_MediaDetect());     // Wait for SD detect to go low       
                    printf("Initializing SD card...\r\n");
                    while (!FSInit());
                    printf("Opening file to write...\r\n");                    
                    filePtr = FSfopen(filename, FS_WRITE);   
                    if (filePtr==NULL) 
                    {
                        printf("Error: could not open %s - STANDBY\r\n", filename);
                        mode = STANDBY;
                    }
                    else
                    {
                        minutes = 0;
                        seconds = 0;
                        hundredths = 0;
                        mode = RECORD;                        
                        printf("%s open - RECORDING\r\n", filename);
                    }
                    break;
                case 16: 
                    printf("Wait for Media Detect...\r\n");
                    while (!MDD_MediaDetect());     // Wait for SD detect to go low    
                    printf("Initializing SD card...\r\n");
                    while (!FSInit());
                    printf("Opening file to read...\r\n");
                    filePtr = FSfopen(filename, FS_READ);            
                    if (filePtr==NULL)
                    {
                        mode = STANDBY;
                        printf("Error: could not open %s - STANDBY\r\n", filename);
                    }
                    else
                    {
                        minutes = 0;
                        seconds = 0;
                        hundredths = 0;                        
                        mode = PLAY;                        
                        nextLineLoaded = false;
                        printf("%s open - PLAY MODE\r\n", filename);                    
                    }
                    break;
                
                case 17: mode = STANDBY;
                    FSfclose(filePtr);
                    printf("STANDBY - File closed\r\n");
                    break;
                case 32: 
                    if (mode == RECORD)
                    {
                        mode = PAUSE_RECORD;
                        printf("PAUSE RECORD\r\n");
                    }
                    else if (mode == PLAY)
                    {
                        mode = PAUSE_PLAY;
                        printf("PAUSE PLAY\r\n");
                    }
                    else if (mode == PAUSE_RECORD)
                    {
                        mode = RECORD;
                        printf("RECORD\r\n");
                    }
                    else if (mode == PAUSE_PLAY)
                    {
                        mode = PLAY;
                        printf("PAUSE PLAY\r\n");
                    }
                    break;
            }
            command = 0;
        }
        /*
        if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = false;
            printf("\rReceived: %s", HOSTRxBuffer);            
        }
        */
        
        if (BlueButtonPushed)
        {
            printf("BLUE PUSHED.\r\n");
            while(!UARTTransmitterIsReady(MIDIuart));
            UARTSendDataByte (MIDIuart, 0x90);
            while(!UARTTransmitterIsReady(MIDIuart));
            UARTSendDataByte (MIDIuart, MIDInote);
            while(!UARTTransmitterIsReady(MIDIuart));
            UARTSendDataByte (MIDIuart, 0x00);            
            BlueButtonPushed = false;        
        }
        else if (YellowButtonPushed)
        {
            printf("YELLOW PUSHED: MIDI note: %02X\r\n", MIDInote);
            YellowButtonPushed = false;            
            
            MIDInote++;
            if (MIDInote > 0x60) MIDInote = 0x24;            
            
            while(!UARTTransmitterIsReady(MIDIuart));
            UARTSendDataByte (MIDIuart, 0x90);            
            while(!UARTTransmitterIsReady(MIDIuart));
            UARTSendDataByte (MIDIuart, MIDInote);            
            while(!UARTTransmitterIsReady(MIDIuart));
            UARTSendDataByte (MIDIuart, 0x40);            
            
            DelayMs(200);

            //while(!UARTTransmitterIsReady(MIDIuart));
            //UARTSendDataByte (MIDIuart, 0x80);                        
            while(!UARTTransmitterIsReady(MIDIuart));
            UARTSendDataByte (MIDIuart, MIDInote);
            while(!UARTTransmitterIsReady(MIDIuart));
            UARTSendDataByte (MIDIuart, 0x00);
            
        }
        
        /*
        if (MemoryBufferFull)
        {
            MemoryBufferFull = false;        
            
            printf("\rOpening %s to append...", filename);
            filePtr = FSfopen(filename, FS_APPEND);
            length = strlen(MemoryBuffer);
            printf("\rWriting %d bytes...", length);        
            numBytes = FSfwrite(MemoryBuffer, 1, length, filePtr);
            MemoryBuffer[0] = '\0';
            printf("\r%d bytes written", numBytes);
            FSfclose(filePtr); 
            printf("\rDONE");            
            
        }                
        */
        
        #if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
        USBDeviceTasks();
        #endif
        ProcessIO();                
        
        if (HOSTTxBufferFull)
        {
            printf("\r\nUSB RX: %s", HOSTTxBuffer);
            HOSTTxBuffer[0] = '\0';
            HOSTTxBufferFull = false;
        }
         
    }//end while
}//end main
 
void InitializeUSART(void)
{
    UART2Init();
}//end InitializeUSART

#define mDataRdyUSART() UART2IsPressed()
#define mTxRdyUSART()   U2STAbits.TRMT

void putcUSART(char c)  
{
    UART2PutChar(c);
}

unsigned char getcUSART ()
{
	char  c;
    c = UART2GetChar();
	return c;
}

void BlinkUSBStatus(void)
{
    static WORD led_count=0;
    
    if(led_count == 0)led_count = 10000U;
    led_count--;

    #define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
    #define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
    #define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
    #define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if(USBSuspendControl == 1)
    {
        if(led_count==0)
        {
            mLED_1_Toggle();
            if(mGetLED_1())
            {
                mLED_2_On();
            }
            else
            {
                mLED_2_Off();
            }
        }//end if
    }
    else
    {
        if(USBDeviceState == DETACHED_STATE)
        {
            mLED_Both_Off();
        }
        else if(USBDeviceState == ATTACHED_STATE)
        {
            mLED_Both_On();
        }
        else if(USBDeviceState == POWERED_STATE)
        {
            mLED_Only_1_On();
        }
        else if(USBDeviceState == DEFAULT_STATE)
        {
            mLED_Only_2_On();
        }
        else if(USBDeviceState == ADDRESS_STATE)
        {
            if(led_count == 0)
            {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        }
        else if(USBDeviceState == CONFIGURED_STATE)
        {
            if(led_count==0)
            {
                mLED_1_Toggle();
                if(mGetLED_1())
                {
                    mLED_2_Off();
                }
                else
                {
                    mLED_2_On();
                }
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus

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
            // ch = toupper(UARTGetDataByte(HOSTuart));
            ch = UARTGetDataByte(HOSTuart);
            if (mode == STANDBY)
            {
                if (ch == 27 && arrowIndex == 0) 
                    arrowIndex++;
                else if (ch == 91 && arrowIndex == 1)
                    arrowIndex++;
                else if ((ch >= 65 && ch <=68) && arrowIndex == 2)                
                {
                    command = ch;
                    arrowIndex = 0;
                }
            }
            
            if (ch == SPACE)
            {
                if (mode != STANDBY) command = SPACE;
            }
            else if (ch < 27 && ch != CR) 
            {
                command = ch;
            }
            else
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
    
    // Set up Timer 2 for 1 millisecond
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_64, 1250);
    // set up the core timer interrupt with a priority of 5 and zero sub-priority
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_5);    
    
    
    #define SYS_FREQ 80000000    
    
    // Set up HOST UART    
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);
    
    // Set up MIDI UART    
    UARTConfigure(MIDIuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(MIDIuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(MIDIuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(MIDIuart, SYS_FREQ, 31250);
    UARTEnable(MIDIuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure MIDI UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(MIDIuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(MIDIuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(MIDIuart), INT_SUB_PRIORITY_LEVEL_0);
    

    PORTClearBits(IOPORT_B, BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);
    mPORTBSetPinsDigitalOut(BIT_3 | BIT_12 | BIT_13 | BIT_14 | BIT_15);

    PORTSetPinsDigitalIn(IOPORT_B, BIT_1);
    PORTSetPinsDigitalOut(IOPORT_B, BIT_0);
    
    PORTSetBits(IOPORT_E, BIT_0);
    PORTSetPinsDigitalIn(IOPORT_D, BIT_15);  // Initialize FAULT signal input   
    
    PORTSetPinsDigitalIn(IOPORT_B, BIT_4 | BIT_2);
    mCNOpen(CN_ON, CN6_ENABLE | CN4_ENABLE, CN6_PULLUP_ENABLE | CN4_PULLUP_ENABLE);
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);
    
    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

	// lastTransmission = 0;

    USBDeviceInit();	// Initializes USB module SFRs and firmware
    					// variables to known states.        
    
	mInitAllLEDs();    
}//end UserInit

void ProcessIO(void)
{   
    static int bytesReceived = 0;     
    int length, i;
    unsigned char ch;   
    unsigned char USBReceived = false;
    
    
    //Blink the LEDs according to the USB device status
    BlinkUSBStatus();
    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

    if (!HOSTTxBufferFull)
    {
        bytesReceived = getsUSBUSART(USBReceivedData, MAXBUFFER); //until the buffer is free.        
        if (bytesReceived > 0)
        {	            
            if (bytesReceived == MAXBUFFER) bytesReceived = MAXBUFFER-1;
            USBReceivedData[bytesReceived] = '\0';            
            length = strlen(HOSTTxBuffer) + bytesReceived;
            if (length <= MAXBUFFER) strcat(HOSTTxBuffer, USBReceivedData);			
            if (strchr(USBReceivedData, '\r')) 
            {
                HOSTTxBufferFull = true;    
                USBReceived = true;
                for (i = 0; i < MAXBUFFER; i++)
                {
                    ch = HOSTTxBuffer[i];
                    MemoryBuffer[i] = ch;
                    if (ch == '\0') break; 
                }
                MemoryBufferFull = true;        
            }                
        }	
    }

    // Check if any bytes are waiting in the queue to send to the USB host.
    // If any bytes are waiting, and the endpoint is available, prepare to
    // send the USB packet to the host.
    if (USBUSARTIsTxTrfReady() && HOSTRxBufferFull)
    {        
        HOSTRxBufferFull = false;
        
        length = strlen(HOSTRxBuffer);         
        
        if (length < (MAXBUFFER-6))
        {
            strcpy (tempBuffer, "UART: ");
            strcat (tempBuffer, HOSTRxBuffer);
            strcpy (HOSTRxBuffer, tempBuffer);
            length = strlen(HOSTRxBuffer); 
            putUSBUSART(HOSTRxBuffer, length);        		
        }               
        else putUSBUSART("ERROR\r", 6);
	}
    
    CDCTxService();
    
}//end ProcessIO

void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) 
{
    static short milliCounter = 0;
    mT2ClearIntFlag(); // clear the interrupt flag
    
    if (TEST_OUT) TEST_OUT = 0;
    else TEST_OUT = 1;
    
    if (mode == RECORD || mode == PLAY)
    {
        milliCounter++;
        if (milliCounter >= 10)
        {
            milliCounter = 0;
            hundredths++;
            if (hundredths >= 100)
            {
                hundredths = 0;
                seconds++;
                if (seconds >= 60)
                {
                    seconds = 0;
                    minutes++;
                }
            }
            time = (minutes * 60 * 100) + (seconds * 60) + hundredths;
        }
    }
    if (MIDItimeout)
    {
        MIDItimeout--;
        if (MIDItimeout == 0) 
            MIDIRxIndex = RxIndex;
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
unsigned short PortBRead = 0x0000;    

    // Step #1 - always clear the mismatch condition first
    // dummy = PORTReadBits(IOPORT_B, BIT_4 | BIT_2) & 0x0F;
    PortBRead = PORTB & 0x14;

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
}


// MIDI UART interrupt handler it is set at priority level 2
void __ISR(MIDI_VECTOR, ipl2) IntMIDIUartHandler(void) {
    unsigned char ch;
    // static unsigned char MIDIstate = 0;    

    if (MIDIbits.OERR || MIDIbits.FERR) 
    {
        if (UARTReceivedDataIsAvailable(MIDIuart))
            ch = UARTGetDataByte(MIDIuart);
        MIDIbits.OERR = 0;
        MIDIRxIndex = 0;
    } 
    else if (INTGetFlag(INT_SOURCE_UART_RX(MIDIuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(MIDIuart));
        if (UARTReceivedDataIsAvailable(MIDIuart)) 
        {
            ch = UARTGetDataByte(MIDIuart);
            if (ch != 0xFE)
            {                
                MIDIRxBuffer[RxIndex++] = ch;
                if (RxIndex >= MAXBUFFER) RxIndex = 0;    
                /*
                if (ch & 0x80) MIDIstate = 0;                    
                else if (ch == 0) MIDIstate = 2;
                else MIDIstate++;                         
                if (MIDIstate >= 2) 
                {
                    MIDIstate = 0;
                    MIDIRxIndex = RxIndex;
                }
                */
                MIDItimeout = 10;
            }
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(MIDIuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(MIDIuart));
    
}




/** EOF main.c *************************************************/

