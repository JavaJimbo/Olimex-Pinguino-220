/***************************************************************************************
 * FileName:  main_LINX.c LINX communication with MAGIC WAND & ACCELEROMETER
 * 
 * 12-19-16: Added LINX communication with magic wand/accelerometer
 * Interference from USB causes lots of errors.
 * 
 ****************************************************************************************/

/************************************ INCLUDES ******************************************/
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

#define TEST_OUT LATBbits.LATB1
#define PUSHBUTTON_DOWN !PORTBbits.RB7 

#define false FALSE
#define true TRUE

#define START_ONE 80
#define STOP 3000
#define START_TWO 80
#define START_THREE 40
#define START_FOUR 40
#define TIMEOUT 200
#define UART_TIMEOUT 400
#define MAXBITLENGTH 20

/** I N C L U D E S **********************************************************/
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb.h"
#include "HardwareProfile.h"


/** V A R I A B L E S ********************************************************/
#define MAXDATABYTES 64
unsigned char arrData[MAXDATABYTES];
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

unsigned char dataReady = FALSE;
unsigned short previousExpected = 0, numExpectedBytes = 0;
unsigned char error = 0;
unsigned char RXstate = 0;
unsigned char timeoutFlag = FALSE;
unsigned short numBytesReceived = 0;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
extern unsigned short CRCcalculate(unsigned char *message, unsigned char nBytes);
static void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);

unsigned char pushbuttonFlag = FALSE;

int main(void) {
    unsigned short trialCounter = 0;
    unsigned short CRCcheck;
    short rawVectx, rawVecty, rawVectz;
    unsigned char motionData = 0;    
    
    union {
        unsigned char byte[2];
        unsigned short integer;
    } convert;
    
    InitializeSystem();
    DelayMs(200);

    printf("\rTesting LINX Communication ...");

    while (1) {
        if (numBytesReceived) {
            CRCcheck = CRCcalculate(&arrData[1], numBytesReceived - 3);
            convert.byte[0] = arrData[numBytesReceived - 2];
            convert.byte[1] = arrData[numBytesReceived - 1];

            if (convert.integer != CRCcheck) printf("\r\rCRC ERROR: %X != %X", convert.integer, CRCcheck);
                        
            motionData = arrData[1];
            
            convert.byte[0] = arrData[2];
            convert.byte[1] = arrData[3];
            rawVectx = (short) convert.integer / 4;
            
            convert.byte[0] = arrData[4];
            convert.byte[1] = arrData[5];
            rawVectz = (short) convert.integer / 4;
            
            convert.byte[0] = arrData[6];
            convert.byte[1] = arrData[7];
            rawVecty = (short) convert.integer / 4;
            
            printf ("\r\r%d: X: %d, Y: %d, Z: %d", ++trialCounter, (short) motionData, rawVectx, rawVecty, rawVectz);
            printf ("\r");
            if (0b00000010 & motionData){
                if (0b00000001 & motionData) printf("-X, ");
                else printf ("+X, ");
            } 
            if (0b00001000 & motionData){
                if (0b00000100 & motionData) printf("-Y, ");
                else printf ("+Y, ");
            }       
            if (0b00100000 & motionData){
                if (0b00010000 & motionData) printf("-Z ");
                else printf ("+Z ");
            }                         
            if (timeoutFlag) {
                printf("\rTIMEOUT");
                timeoutFlag = FALSE;
            }
            numBytesReceived = 0;
        }  // End if (numBytesReceived)
        if (error) {
            printf("\rError: %X", error);
            error = 0;
        }
        
#if defined(USB_INTERRUPT)
        if (USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE)) 
            USBDeviceAttach();        
#endif
        #if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks();
#endif
        ProcessIO();        
    }//end while
}    
 

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

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) {
    static unsigned short Timer2Counter = 0;
    static unsigned short byteMask = 0x0001;
    static unsigned short dataInt = 0x00;
    static unsigned char oddFlag = FALSE;
    static unsigned short dataIndex = 0;
    unsigned short PORTin, RX_PIN;   
    
    // Step #1 - always clear the mismatch condition first
    PORTin = PORTBbits.RB0;
    if (!(PORTin & 0x0001)) pushbuttonFlag = TRUE;

    // Step #2 - then clear the interrupt flag for PORTB
    IFS1bits.CNBIF = 0;

    TEST_OUT = 1;

    Timer2Counter = TMR2 / 75;
    TMR2 = 0x0000;

    if (!PORTin) RX_PIN = 0;
    else RX_PIN = 1;

    // if (RXstate < 5) ConfigIntCN(CHANGE_INT_OFF | CHANGE_INT_PRI_2);

    if (RXstate && (Timer2Counter > TIMEOUT)) {
        if (RXstate == 6) {
            timeoutFlag = TRUE;
            numBytesReceived = dataIndex;
        }
        RXstate = 0;
    }// RX_IN goes HIGH when state machine is idle: first START
    else if (RX_PIN && !RXstate)
        RXstate = 1;

    // RX_IN goes LOW after long pulse: second START
    if (!RX_PIN && Timer2Counter > START_ONE && Timer2Counter < START_ONE * 2)
        RXstate = 2;

    // RX_IN goes HIGH: third START      
    if (RX_PIN && RXstate == 2) {
        if (Timer2Counter > START_TWO && Timer2Counter < START_TWO * 2)
            RXstate++;
        else RXstate = 0;
        // RX_IN goes LOW: fourth START                 
    } else if (RXstate == 3) {
        if (Timer2Counter > START_THREE && Timer2Counter < START_THREE * 2)
            RXstate++;
        else RXstate = 0;
        // RX_IN goes HIGH: dummy bit even pulse
    } else if (RXstate == 4) {
        if (Timer2Counter > START_FOUR && Timer2Counter < START_FOUR * 2) RXstate++;
        else RXstate = 0;
        // RX_IN goes LOW: dummy bit odd pulse                  
    } else if (RXstate == 5) {
        byteMask = 0x01;
        oddFlag = FALSE;
        dataInt = 0x00;
        error = 0;
        dataIndex = 0;        
        RXstate++;
        // RX STATE = 6:
        // DATA BITS get processed here. 
        // Clock state toggles between EVEN and ODD with each transition.
        // Data bit is always read on ODD half of clock cycle, 
        // indicated when oddFlag = TRUE.
        // All LONG pulses begin and end when clock is ODD,
        // so data bit is always read when long pulse is detected.
    } else if (RXstate == 6) {
        // If this a long pulse, data bit always gets read,
        // since all long pulses end on odd clock cycle:
        if (Timer2Counter > MAXBITLENGTH) {
            if (RX_PIN) dataInt = dataInt | byteMask;
            if (byteMask == 0x80) {
                byteMask = 0x01;
                if (dataIndex == 0) {
                    numExpectedBytes = dataInt + 3;
                }
                if (dataIndex < MAXDATABYTES) arrData[dataIndex++] = (unsigned char) dataInt;
                dataInt = 0x00;
            } else byteMask = byteMask << 1;
            oddFlag = FALSE;

            // Otherwise, this must be a short pulse,
            // in which case 
        } else if (oddFlag) {
            oddFlag = FALSE;
            if (RX_PIN) dataInt = dataInt | byteMask;
            if (byteMask == 0x80) {
                byteMask = 0x01;
                if (dataIndex == 0) {
                    numExpectedBytes = dataInt + 3;
                }
                if (dataIndex < MAXDATABYTES) arrData[dataIndex++] = (unsigned char) dataInt;
                dataInt = 0x00;
            } else byteMask = byteMask << 1;
        } else oddFlag = TRUE;

        if (dataIndex >= numExpectedBytes && numExpectedBytes != 0) {
            numBytesReceived = dataIndex;
            dataIndex = 0;
            RXstate = 0;            
        }   
    } // End else    

    TEST_OUT = 0;
}


void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) {
    mT2ClearIntFlag(); // clear the interrupt flag    
    
    //if (TEST_OUT) TEST_OUT = 0;
    //else TEST_OUT = 1;
}


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
unsigned long dummyRead;

    mJTAGPortEnable(false);

    // Initialize all of the debouncing variables
    buttonCount = 0;
    buttonPressed = FALSE;
    stringPrinted = TRUE;

    // Initialize all of the LED pins
    mInitAllLEDs();
    PORTSetPinsDigitalOut(IOPORT_A, BIT_10);
    PORTClearBits(IOPORT_A, BIT_10);

    PORTSetPinsDigitalIn(IOPORT_B, BIT_0); 
    PORTSetPinsDigitalOut(IOPORT_B, BIT_1);
    
    CNCONBbits.ON = 1;      // CN is enabled
    CNCONBbits.SIDL = 0;    // CPU Idle does not affect CN operation
    CNENBbits.CNIEB0 = 1;   // Enable RB0 change notice    
    
    // Read port B to clear mismatch condition
    dummyRead = PORTB;
 
    // Clear CN interrupt flag
    IFS1bits.CNBIF = 0;                     // Clear status register for port B
    IPC8CLR = _IPC8_CNIP_MASK;              // Clear priority
    IPC8SET = (2 << _IPC8_CNIP_POSITION);   // Set priority (2)
    IEC1bits.CNBIE = 1;                     // Enable CN interrupts on port B    
    CNPUBbits.CNPUB0 = 1;                   // Pullup enable  

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
    
    // Set up Timer 2, no interrupts 
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:8 Prescaler
    T2CONbits.TCKPS1 = 1;
    T2CONbits.TCKPS0 = 1;
    T2CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR2 = 0xFFFF;
    T2CONbits.TON = 1; // Let her rip   
    // ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_5);

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

