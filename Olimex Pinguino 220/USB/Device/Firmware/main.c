/***************************************************************************************
 * FileName:  main.c
 * Project:   Olimex Pinguino 220 for PIC32MX220F032D 
 * Compiler:  XC32 V1.30
 * 
 *
 * 12-18-2016: Got USB communication and UART Rx/Tx 
 *  working using PPS Peripheral Pin Select.
 *  System clock set to 60 Mhz. Got LED #1 and #2 working.
 *  ProcessIO() implements USB-UART bridge.
 * 
 ****************************************************************************************/

/** INCLUDES *******************************************************/
#include "USB/usb.h"
#include "USB/usb_function_cdc.h"
#include "HardwareProfile.h"

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_15        // PLL Multiplier for 220 - yields 60 Mhz
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

/*** DEFINES *****************************************************************/
#define HOSTuart UART2
#define SYS_FREQ 60000000  // With 8 Mhz crystal and FPLLMUL = MUL_15
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR     
#define MAXBUFFER 64

#define PUSHBUTTON_DOWN !PORTBbits.RB7 

// #define FALSE 0
// #define TRUE !FALSE
#define false FALSE
#define true TRUE


/** I N C L U D E S **********************************************************/
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb.h"
#include "HardwareProfile.h"


/** V A R I A B L E S ********************************************************/
unsigned char HOSTRxBuffer[MAXBUFFER];
unsigned char HOSTRxBufferFull = FALSE;

unsigned char HOSTTxBuffer[MAXBUFFER];

unsigned char USBRxBuffer[MAXBUFFER];
unsigned char USBTxBuffer[MAXBUFFER];

unsigned short HOSTRxLength = 0;
unsigned short HOSTTxLength = 0;

char USB_In_Buffer[64];
char USB_Out_Buffer[64];

BOOL stringPrinted;
volatile BOOL buttonPressed;
volatile BYTE buttonCount;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);

int main(void) {

    InitializeSystem();
    DelayMs(200);
    printf("\rTesting USB and UART  bridge...");

    while (1) {
#if defined(USB_INTERRUPT)
        if (USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE)) {
            USBDeviceAttach();
        }
#endif

#if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks();
#endif

        ProcessIO();
    }//end while
}//end main

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void) {
#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif
    UserInit();

    USBDeviceInit();
}//end InitializeSystem

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 * PPSInput(2,U2RX,RPA9);       // Assign U2RX to pin RPA09
 * PPSInput(3,U2CTS,RPC3);      // Assign U2CTS to pin RPC3
 * PPSOutput(4,RPC4,U2TX);      // Assign U2TX to pin RPC4
 * PPSOutput(1,RPB15,U2RTS);    // Assign U2RTS to pin RPB15    
 *
 *****************************************************************************/
void UserInit(void) {
    mJTAGPortEnable(false);

    // Initialize all of the debouncing variables
    buttonCount = 0;
    buttonPressed = FALSE;
    stringPrinted = TRUE;

    // Initialize all of the LED pins
    mInitAllLEDs();
    PORTSetPinsDigitalOut(IOPORT_A, BIT_10);
    PORTClearBits(IOPORT_A, BIT_10);

    PORTSetPinsDigitalIn(IOPORT_B, BIT_7);

    // Set up main UART    
    PPSOutput(4, RPC9, U2TX);
    PPSInput(2, U2RX, RPC8);    

    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    // INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);        

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();
}//end UserInit

void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    static unsigned short HOSTRxIndex = 0;
    static unsigned char TxIndex = 0;
    unsigned char ch;

    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = UARTGetDataByte(HOSTuart);
            if (ch != 0 && ch != '\n') {
                if (HOSTRxIndex < MAXBUFFER - 2)
                    HOSTRxBuffer[HOSTRxIndex++] = ch;
                if (ch == '\r') {
                    HOSTRxBuffer[HOSTRxIndex] = '\0';
                    HOSTRxBufferFull = TRUE;
                    HOSTRxIndex = 0;
                }
            }
            // UARTtimeout=UART_TIMEOUT;
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
        if (HOSTTxLength) {
            if (TxIndex < MAXBUFFER) {
                ch = HOSTTxBuffer[TxIndex++];
                if (TxIndex <= HOSTTxLength) {
                    while (!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte(HOSTuart, ch);
                } else {
                    while (!UARTTransmitterIsReady(HOSTuart));
                    HOSTTxLength = false;
                    TxIndex = 0;
                }
            } else {
                TxIndex = 0;
                HOSTTxLength = false;
                INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
            }
        } else INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    }
}

/*
void ProcessIO(void) {
    BYTE numBytesRead;

    // Blink the LEDs according to the USB device status
    BlinkUSBStatus();
    
    // User Application USB tasks
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

    if (buttonPressed) {
        if (stringPrinted == FALSE) {
            if (mUSBUSARTIsTxTrfReady()) {
                putrsUSBUSART("Button Pressed -- \r\n");
                stringPrinted = TRUE;
            }
        }
    } else  stringPrinted = FALSE;

    if (USBUSARTIsTxTrfReady()) {
        numBytesRead = getsUSBUSART(USB_Out_Buffer, 64);
        if (numBytesRead != 0) {
            BYTE i;

            for (i = 0; i < numBytesRead; i++) {
                switch (USB_Out_Buffer[i]) {
                    case 0x0A:
                    case 0x0D:
                        USB_In_Buffer[i] = USB_Out_Buffer[i];
                        break;
                    default:
                        USB_In_Buffer[i] = USB_Out_Buffer[i] + 1;
                        break;
                }
            }
            putUSBUSART(USB_In_Buffer, numBytesRead);
        }
    }
    CDCTxService();
} //end ProcessIO
 */

void BlinkUSBStatus(void) {
    static WORD led_count = 0;

    if (led_count == 0)led_count = 10000U;
    led_count--;

#define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
#define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
#define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
#define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if (USBSuspendControl == 1) {
        if (led_count == 0) {
            mLED_1_Toggle();
            if (mGetLED_1()) mLED_2_On()
            else mLED_2_Off()
            }//end if
    }

    else {
        if (USBDeviceState == DETACHED_STATE)
            mLED_Both_Off()
        else if (USBDeviceState == ATTACHED_STATE)
            mLED_Both_On()
        else if (USBDeviceState == POWERED_STATE)
            mLED_Only_1_On()
        else if (USBDeviceState == DEFAULT_STATE)
            mLED_Only_2_On()
        else if (USBDeviceState == ADDRESS_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle()
                mLED_2_Off()
            }//end if
        } else if (USBDeviceState == CONFIGURED_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle()
                if (mGetLED_1())
                    mLED_2_Off()
                else mLED_2_On()
                }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus

void USBCBSuspend(void) {
    ;
}

void USBCBWakeFromSuspend(void) {
    ;
}

void USBCB_SOF_Handler(void) {
    if (buttonPressed == PUSHBUTTON_DOWN) {
        if (buttonCount != 0)
            buttonCount--;
        else {
            //This is reverse logic since the pushbutton is active low
            buttonPressed = !PUSHBUTTON_DOWN;
            //Wait 100ms before the next press can be generated
            buttonCount = 100;
        }
    } else {
        if (buttonCount != 0)
            buttonCount--;
    }
}

void USBCBErrorHandler(void) {
    ;
}

void USBCBCheckOtherReq(void) {
    USBCheckCDCRequest();
}

void USBCBStdSetDscHandler(void) {
    ;
}

void USBCBInitEP(void) {
    CDCInitEP();
}

void USBCBSendResume(void) {
    static WORD delay_count;

    if (USBGetRemoteWakeupStatus() == TRUE) {
        if (USBIsBusSuspended() == TRUE) {
            USBMaskInterrupts();
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FALSE; //So we don't execute this code again, 
            //until a new suspend condition is detected.
            delay_count = 3600U;
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 1; // Start RESUME signaling
            delay_count = 1800U; // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 0; //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)

void USBCBEP0DataReceived(void) {
}
#endif

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size) {
    switch (event) {
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

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/

void ProcessIO(void) {
    static unsigned char ch, USBTxIndex = 0;
    unsigned short i, length;
    BYTE numBytesRead;

    // Blink the LEDs according to the USB device status
    BlinkUSBStatus();

    // User Application USB tasks
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

    numBytesRead = getsUSBUSART(USBRxBuffer, 64);
    
    for (i = 0; i < numBytesRead; i++){
        ch = USBRxBuffer[i];
        if (USBTxIndex < MAXBUFFER-2) HOSTTxBuffer[USBTxIndex++] = ch;
        if (ch == '\r'){
            HOSTTxBuffer[USBTxIndex] = '\0';
            printf("\rUSB RECEIVED: %s", HOSTTxBuffer);
            USBTxIndex = 0;
        }
    }

    if (USBUSARTIsTxTrfReady()) {
        if (HOSTRxBufferFull) {
            HOSTRxBufferFull = FALSE;
            length = strlen(HOSTRxBuffer);
            if (length >= MAXBUFFER) length = MAXBUFFER;
            putUSBUSART(HOSTRxBuffer, length);
        }
    }
    CDCTxService();
} //end ProcessIO
