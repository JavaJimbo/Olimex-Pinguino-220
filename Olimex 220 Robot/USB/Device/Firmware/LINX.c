/***************************************************************************************
 * Project:     Olimex 220 Robot
 * FileName:    main.c 
 * 
 *
 * 1-15-17: 
 * 2-25-17: Add decoding and 
 ****************************************************************************************/
// #define USE_USB
// #define USE_PWM
// #define USE_AD

// #define I2C_CLOCK_FREQ              100000
// #define EEPROM_I2C_BUS              I2C1
// #define EEPROM_ADDRESS              0x50        // 0b1010000 Serial EEPROM address

#define SYS_FREQ 60000000
#define GetPeripheralClock() SYS_FREQ 
#define USE_LINX

#define EA PORTAbits.RA10 
#define EB PORTAbits.RA1
#define PWM1 OC1RS
#define PWM2 OC2RS
#define PWM3 OC5RS
#define PWM4 OC4RS

#include "USB/usb.h"
#include "USB/usb_function_cdc.h"
#include "HardwareProfile.h"
#include "I2C_EEPROM_PIC32.h"
#include "PCA9685.h"
#include "Delay.h"
#include <plib.h>

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
#pragma config JTAGEN   = OFF

/*** DEFINES *****************************************************************/
#define HOSTuart UART2
#define SYS_FREQ 60000000  // With 8 Mhz crystal and FPLLMUL = MUL_15
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR     
#define MAXBUFFER 64

// #define TEST_OUT LATBbits.LATB1
#define TEST_OUT LATCbits.LATC0
#define PUSHBUTTON_DOWN !PORTBbits.RB7 

#define false FALSE
#define true TRUE

#define START_ONE 80
#define START_TWO 80
#define START_THREE 20
#define START_FOUR 20
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
unsigned char RxBuffer[MAXBUFFER];
unsigned char RxBufferFull = FALSE;

unsigned char TxBuffer[MAXBUFFER];
unsigned char USBRxBuffer[MAXBUFFER];
unsigned char USBTxBuffer[MAXBUFFER];

unsigned short RxLength = 0;
unsigned short TxLength = 0;

unsigned short previousExpected = 0, numExpectedBytes = 0;
unsigned char error = 0;
unsigned char RXstate = 0;
unsigned char timeoutFlag = FALSE;
unsigned short numBytesReceived = 0;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
// extern unsigned short CRCcalculate(unsigned char *message, unsigned char nBytes);
extern unsigned char getCRC8(unsigned char *ptrMessage, short numBytes);
static void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
// void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);
void ConfigAd(void);

unsigned char PCAReadByte(unsigned char device, unsigned char PCAcontrolRegister, unsigned char *ptrData);
unsigned char PCAWriteByte(unsigned char device, unsigned char PCAcontrolRegister, unsigned char data);

unsigned char setPCA9685outputs(unsigned char device, unsigned short channel, unsigned short turnON, unsigned short turnOFF);
unsigned char initializePCA9685(unsigned char device);
unsigned char setPWM(unsigned char device, unsigned char motor, short PWMdata);
unsigned int decodePacket (unsigned char *ptrInPacket, unsigned int numInBytes, unsigned char *ptrData);

#define MAXPOTS 1
unsigned short arrPots[MAXPOTS];
unsigned char ADflag = FALSE;

/*
int main(void) {
    unsigned short trialCounter = 0;
    unsigned char CRCcheck, CRCdata;
    signed char rawVectx, rawVecty, rawVectz;
    unsigned char i;
    unsigned short loopCounter = 0;     
    //unsigned short CRCcomplete;
    //unsigned char errorFlag = FALSE;

    //union {
    //    unsigned char byte[2];
    //    unsigned short integer;
    //} convert;

    InitializeSystem();
    DelayMs(200);
    printf("\rTESTING: ");

#ifdef USE_USB
    printf("USB BRIDGE, ");
#endif    

#ifdef USE_LINX
    printf("LINX COMMUNICATION, ");
#endif

#ifdef USE_AD
    printf(" AD CONVERSION, ");
#endif    

#ifdef USE_PWM
    printf("PWM");
#endif

    while (1) {
#ifdef USE_LINX     
        if (numBytesReceived){
            //errorFlag = FALSE;
            
            CRCcheck = getCRC8(arrData, numBytesReceived - 1);
            CRCdata = arrData[numBytesReceived - 1];
            if (CRCcheck != CRCdata) {
                printf("\rCRC ERROR");
                //errorFlag = TRUE;
            }

            //CRCcomplete = CRCcalculate (arrData, 4);
            //convert.byte[0] = arrData[numBytesReceived-2];
            //convert.byte[1] = arrData[numBytesReceived-1];
            
            //if (convert.integer != CRCcomplete){
            //    if (!errorFlag) printf("\r\rCRC 7 ERROR!!!!!!!!!!!!!");
                // printf("\rCRCconvert: %X, CRCccmplete: %X", convert.integer, CRCcomplete);
            //}

            rawVectx = (signed char) arrData[1];
            rawVecty = (signed char) arrData[2];
            rawVectz = (signed char) arrData[3];
            
            printf("\r%d: X: %d, Y: %d, Z: %d", ++trialCounter, (int) rawVectx, (int) rawVecty, (int) rawVectz);            

            numBytesReceived = 0;
        } // End if (numBytesReceived)


        if (error) {
            printf("\rError: %X", error);
            error = 0;
        }
#endif

#ifdef USE_AD
        DelayMs(1);
        if (loopCounter) loopCounter--;
        if (!loopCounter) {
            loopCounter = 100;
            // printf("\rPWM value: %d", (int) arrPots[0]);
            OC1RS = arrPots[0];
            if (ADflag) {
                ADflag = FALSE;
                // printf(" AD INT");
                mAD1IntEnable(INT_ENABLED);
            }
        }
#endif        

#ifdef USE_USB        

#if defined(USB_INTERRUPT)
        if (USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE))
            USBDeviceAttach();
#endif
#if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks();
#endif
        ProcessIO();
#endif

    }//end while
}
 */

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

#ifdef USE_USB
    USBDeviceInit();
#endif

}//end InitializeSystem

#ifdef USE_LINX

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) {
    unsigned short Timer2Counter = 0;
    static unsigned short byteMask = 0x0001;
    static unsigned short dataInt = 0x00;
    static unsigned char oddFlag = FALSE;
    static unsigned short dataIndex = 0;
    unsigned short PORTin, RX_PIN;

    // Step #1 - always clear the mismatch condition first
    PORTin = PORTBbits.RB0;

    // Step #2 - then clear the interrupt flag for PORTB
    IFS1bits.CNBIF = 0;

    if (TEST_OUT) TEST_OUT = 0;
    else TEST_OUT = 1;

    Timer2Counter = TMR2 / 75; // Timer2Counter is the elapsed time since last interrupt
    // in increments of 10 microseconds. 1/10 micro = 100 kHz
    // 60 Mhz system clock / 8 prescale / 75 = 100 khz
    TMR2 = 0x0000; // Reset both timers.
    TMR4 = 0x0000;
    mT4ClearIntFlag(); // Clear the Timer 4 interrupt flag to ensure
    // it won't re-enable interrupt on change for another 100 microseconds
    if (!PORTin) RX_PIN = 0;
    else RX_PIN = 1;

    if (RXstate < 5) IEC1bits.CNBIE = 0; // If the RXstate hasn't yet incremented to 5,
    // then a START transmission sequence hasn't completed,
    // and we assume the LINX isn't receiving valid data.
    // Input on change interrupts are temporarily disabled here 
    // until Timer 4 re-enables them in 100 microseconds.
    // Otherwise random input noise could swamp the CPU
    // with interrupts. 
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
    if (!RX_PIN && Timer2Counter > START_ONE) RXstate = 2;
    // RX_IN goes HIGH: third START      
    if (RX_PIN && RXstate == 2) {
        if (Timer2Counter > START_TWO) RXstate++;
        else RXstate = 0;
        // RX_IN goes LOW: fourth START                 
    } else if (RXstate == 3) {
        if (Timer2Counter > START_THREE) RXstate++;
        else RXstate = 0;
        // RX_IN goes HIGH: dummy bit even pulse
    } else if (RXstate == 4) {
        if (Timer2Counter > START_FOUR) RXstate++;
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
                    numExpectedBytes = dataInt;
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
                    numExpectedBytes = dataInt;
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
}
#endif

// This interrupt occurs at 10 khz, or once every 100 microseconds
// The purpose is to re enable interrupt on change 
// for LINX input on PORT B

void __ISR(_TIMER_4_VECTOR, ipl5) Timer4Handler(void) {
    mT4ClearIntFlag(); // Clear the interrupt flag    
    IEC1bits.CNBIE = 1; // Enable CN interrupts
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
    PORTSetPinsDigitalOut(IOPORT_C, BIT_0);

    /*    
        // Initialize all of the LED pins
        mInitAllLEDs();
        PORTSetPinsDigitalOut(IOPORT_A, BIT_10);
        PORTClearBits(IOPORT_A, BIT_10);

        PORTSetPinsDigitalIn(IOPORT_B, BIT_0);
        PORTSetPinsDigitalOut(IOPORT_B, BIT_1);

    #ifdef USE_LINX
        // Set up interrupt on change for the PORT B LINX receiver input pin RB0
        CNCONBbits.ON = 1; // CN is enabled
        CNCONBbits.SIDL = 0; // CPU Idle does not affect CN operation
        CNENBbits.CNIEB0 = 1; // Enable RB0 change notice    

        // Read port B to clear mismatch condition
        dummyRead = PORTB;

        // Clear CN interrupt flag
        IFS1bits.CNBIF = 0; // Clear status register for port B
        IPC8CLR = _IPC8_CNIP_MASK; // Clear priority
        IPC8SET = (2 << _IPC8_CNIP_POSITION); // Set priority (2)
        IEC1bits.CNBIE = 1; // Enable CN interrupts on port B    
        CNPUBbits.CNPUB0 = 1; // Pullup enable  
    #endif    
     */
    // Set up main UART    
    PPSOutput(4, RPC9, U2TX);
    PPSInput(2, U2RX, RPC8);

    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 19200);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    // INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);  

    // Set up Timer 2 with no interrupts.
    // This timer is used to measure the intervals
    // between interrupts on change on the LINX receiver PORT B input pin.
    // The interrupt on change routine divides the timer 2 counter TMR2 by 75
    // to get multiples of 10 microseconds.
    // So the 60 mhz system clock / 8 prescaler / 75 = 100 khz,
    // which is the maximum resolution for the LINX input.
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:8 Prescaler
    T2CONbits.TCKPS1 = 1;
    T2CONbits.TCKPS0 = 1;
    T2CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR2 = 0xFFFF;
    T2CONbits.TON = 1; // Let her rip   

    // Set up Timer 4, to interrupt every 100 microseconds, 10 khz
    T4CON = 0x00;
    T4CONbits.TCKPS2 = 0; // 1:8 Prescaler
    T4CONbits.TCKPS1 = 1;
    T4CONbits.TCKPS0 = 1;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR4 = 750;
    T4CONbits.TON = 1; // Let her rip   
    ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_5);


#ifdef USE_PWM    
    // Set up Timer 3 for PWM time base    
    T3CON = 0x00;
    T3CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T3CONbits.TCKPS1 = 0;
    T3CONbits.TCKPS0 = 0;
    PR3 = 1024; // Use 50 microsecond rollover for 20 khz
    T3CONbits.TON = 1; // Let her rip

    // Set up PWM OC1 on D8 on the Olimex 220 board:
    PPSOutput(1, RPB7, OC1);
    OC1CON = 0x00;
    OC1CONbits.OC32 = 0; // 16 bit PWM
    OC1CONbits.ON = 1; // Turn on PWM
    OC1CONbits.OCTSEL = 1; // Use Timer 3 as PWM time base
    OC1CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC1CONbits.OCM1 = 1;
    OC1CONbits.OCM0 = 0;
    OC1RS = 0;

    // Set up PWM OC2 on D11 on the Olimex 220 board:
    PPSOutput(2, RPB5, OC2);
    OC2CON = 0x00;
    OC2CONbits.OC32 = 0; // 16 bit PWM
    OC2CONbits.ON = 1; // Turn on PWM
    OC2CONbits.OCTSEL = 1; // Use Timer 3 as PWM time base
    OC2CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC2CONbits.OCM1 = 1;
    OC2CONbits.OCM0 = 0;
    OC2RS = 0;

    // Set up PWM OC5 on D12 on the Olimex 220 board:
    PPSOutput(3, RPB13, OC5);
    OC5CON = 0x00;
    OC5CONbits.OC32 = 0; // 16 bit PWM
    OC5CONbits.ON = 1; // Turn on PWM
    OC5CONbits.OCTSEL = 1; // Use Timer 3 as PWM time base
    OC5CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC5CONbits.OCM1 = 1;
    OC5CONbits.OCM0 = 0;
    OC5RS = 0;

    // Set up PWM OC4 on D6 on the Olimex 220 board:
    PPSOutput(3, RPC6, OC4);
    OC4CON = 0x00;
    OC4CONbits.OC32 = 0; // 16 bit PWM
    OC4CONbits.ON = 1; // Turn on PWM
    OC4CONbits.OCTSEL = 1; // Use Timer 3 as PWM time base
    OC4CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC4CONbits.OCM1 = 1;
    OC4CONbits.OCM0 = 0;
    OC4RS = 0;

    PORTSetPinsDigitalOut(IOPORT_A, BIT_1 | BIT_10);
    PORTClearBits(IOPORT_A, BIT_1 | BIT_10);
    ANSELAbits.ANSA1 = 0;

#endif

#ifdef USE_AD    
    ConfigAd();
#endif    

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();
}//end UserInit

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
    } else {
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
    ;
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

/*
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
 */

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
#ifdef USE_USB

void ProcessIO(void) {
    static unsigned char ch, USBTxIndex = 0;
    unsigned short i, length;
    BYTE numBytesRead;

    // Blink the LEDs according to the USB device status
    BlinkUSBStatus();

    // User Application USB tasks
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

    numBytesRead = getsUSBUSART(USBRxBuffer, 64);

    for (i = 0; i < numBytesRead; i++) {
        ch = USBRxBuffer[i];
        if (USBTxIndex < MAXBUFFER - 2) HOSTTxBuffer[USBTxIndex++] = ch;
        if (ch == '\r') {
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
#endif

#ifdef USE_AD    

void ConfigAd(void) {

    mPORTCSetPinsAnalogIn(BIT_1);

    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31

    //  set AM7 (A1 on Olimex 220 board) input to analog
    // #define PARAM4    ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA | ENABLE_AN3_ANA
#define PARAM4    ENABLE_AN7_ANA


    // USE AN7    
#define PARAM5 SKIP_SCAN_AN0 |SKIP_SCAN_AN1 |SKIP_SCAN_AN2 |SKIP_SCAN_AN3 |\
SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 |\
SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 |\
SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15


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
#endif

void __ISR(_ADC_VECTOR, ipl6) ADHandler(void) {
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();


    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        arrPots[i] = (unsigned short) ReadADC10(offSet + i); // read the result of channel 0 conversion from the idle buffer
    ADflag = TRUE;
}

int main(void) {
    unsigned char writeString[] = "Testing data blocks";
    int size = sizeof (writeString);
    unsigned char readString[size + 1], writeByte, readByte;
    unsigned short memoryAddress;
    unsigned short Duty1 = 0, Duty2 = 0, Duty3 = 0, Duty4 = 0, i;
#define MAXREGISTER 64
    unsigned char PCAregister[MAXREGISTER], testData[MAXREGISTER];

    // Initialize debug messages (when supported)    
    UserInit();

    i = 0;
    while (1) {
        DelayMs(200);
        printf("%d", i++);
    }
    // OpenI2C(I2C_EN, 299);

    /*
    printf("\rWriting %d bytes to EEPROM address %X", size, memoryAddress);
    printf("\rWRITE BLOCK");
    if (!EepromWriteBlock(EEPROM_ADDRESS, memoryAddress, writeString, size)) printf("\rWrite error");

    printf("\rREAD BLOCK:");
    if (!EepromReadBlock(EEPROM_ADDRESS, memoryAddress, readString, size)) printf("\rRead error");
    else {
        printf("\rSUCCESS: ");
        short length;
        length = strlen(readString);
        readString[length] = '\0';
        printf("\rBlock: %s", readString);
    }
    CloseI2C();
    printf("\rDONE");
    while (1);
     */

    //printf("\rWRITE BYTE:");
    //writeByte = 76;
    //if (!EepromWriteByte(EEPROM_ADDRESS, 0x1234, writeByte)) printf("\rWrite error");

    //printf("\rREAD BYTE:");
    //if (!EepromReadByte(EEPROM_ADDRESS, 0x1234, &readByte)) printf("\rRead error");
    //else printf("\rSUCCESS byte = %d", readByte);


#define PCA9685_ADDRESS 0b11000000   
    printf("\rInitializing: PCA9685: ");

    if (!initializePCA9685(PCA9685_ADDRESS)) printf("\rWrite error #1");
    else printf("\rSetting PWM:");

#define DELAYLOOPS 3
    while (1) {
        setPWM(PCA9685_ADDRESS, M1, 2048);
        setPWM(PCA9685_ADDRESS, M2, 0);
        setPWM(PCA9685_ADDRESS, M3, 0);
        setPWM(PCA9685_ADDRESS, M4, 0);
        for (i = 0; i < DELAYLOOPS; i++) DelayMs(100);
        setPWM(PCA9685_ADDRESS, M1, 0);
        setPWM(PCA9685_ADDRESS, M2, 2048);
        setPWM(PCA9685_ADDRESS, M3, 0);
        setPWM(PCA9685_ADDRESS, M4, 0);
        for (i = 0; i < DELAYLOOPS; i++) DelayMs(100);
        setPWM(PCA9685_ADDRESS, M1, 0);
        setPWM(PCA9685_ADDRESS, M2, 0);
        setPWM(PCA9685_ADDRESS, M3, 2048);
        setPWM(PCA9685_ADDRESS, M4, 0);
        for (i = 0; i < DELAYLOOPS; i++) DelayMs(100);
        setPWM(PCA9685_ADDRESS, M1, 0);
        setPWM(PCA9685_ADDRESS, M2, 0);
        setPWM(PCA9685_ADDRESS, M3, 0);
        setPWM(PCA9685_ADDRESS, M4, 2048);
        for (i = 0; i < DELAYLOOPS; i++) DelayMs(100);

        setPWM(PCA9685_ADDRESS, M1, -2048);
        setPWM(PCA9685_ADDRESS, M2, 0);
        setPWM(PCA9685_ADDRESS, M3, 0);
        setPWM(PCA9685_ADDRESS, M4, 0);
        for (i = 0; i < DELAYLOOPS; i++) DelayMs(100);
        setPWM(PCA9685_ADDRESS, M1, 0);
        setPWM(PCA9685_ADDRESS, M2, -2048);
        setPWM(PCA9685_ADDRESS, M3, 0);
        setPWM(PCA9685_ADDRESS, M4, 0);
        for (i = 0; i < DELAYLOOPS; i++) DelayMs(100);
        setPWM(PCA9685_ADDRESS, M1, 0);
        setPWM(PCA9685_ADDRESS, M2, 0);
        setPWM(PCA9685_ADDRESS, M3, -2048);
        setPWM(PCA9685_ADDRESS, M4, 0);
        for (i = 0; i < DELAYLOOPS; i++) DelayMs(100);
        setPWM(PCA9685_ADDRESS, M1, 0);
        setPWM(PCA9685_ADDRESS, M2, 0);
        setPWM(PCA9685_ADDRESS, M3, 0);
        setPWM(PCA9685_ADDRESS, M4, -2048);
        for (i = 0; i < DELAYLOOPS; i++) DelayMs(100);
    }
}




/*
//#define BRG_VAL 299        
//OpenI2C(I2C_EN, BRG_VAL);  
//EA = 0;
//EB = 0;
PORTSetBits(IOPORT_A, BIT_10);
PORTSetBits(IOPORT_A, BIT_1);
    
while(1){        
    PWM2 = PWM4 = 0;
    for(i = 0; i < 1024; i++){
        PWM1 = PWM3 = i;
        DelayMs(10);
    }
    DelayMs(100);
        
    PWM2 = PWM4 = 0;
    for(i = 1023; i > 0; i--){
        PWM1 = PWM3 = i;
        DelayMs(10);
    }
    DelayMs(100);
        
    PWM1 = PWM3 = 0;
    for(i = 0; i < 1024; i++){
        PWM2 = PWM4 = i;
        DelayMs(10);
    }
    DelayMs(100);
        
    PWM1 = PWM3 = 0;
    for(i = 1023; i > 0; i--){
        PWM2 = PWM4 = i;
        DelayMs(10);
    }
    DelayMs(100);
 */


#define STANDBY 0
#define PACKETOK 1
#define PACKETERROR 2

void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
	static unsigned char startFlag = false;
    static unsigned char escapeFlag = false;
    static unsigned int RxIndex = 0;
	unsigned char ch;
	unsigned int i = 0, dataLength;

    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));        
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                ch = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;
            RxIndex = 0;
        }    
    
/*    
    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = UARTGetDataByte(HOSTuart);

            j = 0;
            for (i = 0; i < PACKETSIZE; i++) {
                ch = ptrPacket[i];
                if (!status) {
                    if (ch == STX) {
                        status = 1;
                        escapeFlag = FALSE;
                    }
                } else {
                    if (!escapeFlag) {
                        if (ch == STX) {
                            status = PACKETERROR;
                            escapeFlag = FALSE;
                        } else if (ch == ETX) {
                            status = DONE;
                        } else if (ch == DLE)
                            escapeFlag = TRUE;
                        else if (j < MAXBITMAP)
                            ptrBitmap[j++] = ch;
                        else {
                            status = USB_PACKETERROR;
                        }
                    } else {
                        escapeFlag = FALSE;
                        if (j < MAXBITMAP)
                            ptrBitmap[j++] = ch;
                        else {
                            USBstatus = USB_PACKETERROR;
                            j = 0;
                        }
                    }
                }
            }
        }
    }
 */
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
}


// DECODER ROUTINE

#define STX 36
#define ETX 13
#define DLE 16

unsigned int decodePacket (unsigned char *ptrInPacket, unsigned int numInBytes, unsigned char *ptrData) {	
	static unsigned char startFlag = false, escapeFlag = false;
    static unsigned char dataIndex = 0;
	unsigned char ch;
	unsigned int i = 0, dataLength;

	if (ptrInPacket == NULL || numInBytes == 0 || ptrData == NULL || numInBytes >= MAXBUFFER || dataIndex >= MAXBUFFER)
	{
		startFlag = false;
		escapeFlag = false;
		dataIndex = 0;
		return (0);
	}				
	do {		
		ch = ptrInPacket[i++];	
        
		// Store next char if packet is valid and board number matches
		if (startFlag && dataIndex < MAXBUFFER) ptrData[dataIndex] = ch;   
			
		// If preceding character wasn't an escape char:
		// check whether it is STX, ETX or DLE,
		// otherwise if board number matches then store and advance for next char
		if (escapeFlag == false) {
			if (ch == DLE) escapeFlag = true;
			else if (ch == STX) {
				dataIndex = 0;
				startFlag = true;			
			}
			else if (ch == ETX) {
				startFlag = false;				
				dataLength = dataIndex;
				// dataIndex = 0;
				return (dataLength);
			}
			else if (startFlag) dataIndex++;
		}		
		// Otherwise if preceding character was an escape char:	
		else { 
			escapeFlag = false;
			if (startFlag) dataIndex++;  	
		}	
	} while (i < numInBytes && i < MAXBUFFER && dataIndex < MAXBUFFER);
	return (0);
}	
