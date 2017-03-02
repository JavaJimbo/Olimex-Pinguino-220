/***************************************************************************************
 * PCA9685.h
 * Routines for initializing and setting PWM on Adafruit V2 Motor Shield
 * Written for PIC 32MX220F032D on Olimex PIC32-Pinguino-MX220 board
 * Compiled with Micrchip XC32 V1.30 C compiler
 * 
 * Usage: 
 * 
 ****************************************************************************************/

#define LEFTFRONTPWM M2
#define LEFTFRONTPENC TMR5        

#define RIGHTFRONTPWM M1
#define RIGHTFRONTPENC TMR2
    
#define LEFTREARPWM M3
#define LEFTREARENC TMR4   
    
#define RIGHTREARPWM M4
#define RIGHTREARENC TMR3  

#define M1 0
#define M2 1
#define M3 2
#define M4 3
#define FULL_ON 0x1000

#define PWMA 2
#define AIN2 3
#define AIN1 4
#define BIN1 5
#define BIN2 6
#define PWMB 7
#define PWMC 8
#define CIN2 9
#define CIN1 10
#define DIN1 11
#define DIN2 12
#define PWMD 13

#define ALL_LED_ON 0xFA
#define ALL_LED_OFF 0xFC
#define MODE1_REG 0x00
#define MODE2_REG 0x01
#define OUTDRV 0x04
#define ALLCALL 0x01
#define SLEEP_BIT 0x10
#define PRESCALE 0xFE
#define RESTART 0x80

#define LED_ON_REGISTER 0x06
#define MAX_LED_CHANNEL 15

unsigned char PCAReadByte (unsigned char device, unsigned char PCAcontrolRegister, unsigned char *ptrData);
unsigned char PCAWriteByte (unsigned char device, unsigned char PCAcontrolRegister, unsigned char data);
unsigned char setPCA9685outputs (unsigned char device, unsigned short channel, unsigned short turnON, unsigned short turnOFF);
unsigned char initializePCA9685(unsigned char device);
unsigned char setPWM (unsigned char device, unsigned char motor, short PWMdata);

