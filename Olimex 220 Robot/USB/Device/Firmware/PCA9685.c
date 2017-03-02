/***************************************************************************************
 * PCA9685.c 
 * Routines for initializing and setting PWM on Adafruit V2 Motor Shield
 * Written for PIC 32MX220F032D on Olimex PIC32-Pinguino-MX220 board
 * Compiled with Micrchip XC32 V1.30 C compiler
 * 
 * Usage: 
 * 
 ****************************************************************************************/

#include "I2C_EEPROM_PIC32.h"
#include "PCA9685.h"
#include "Delay.h"
#include <plib.h>

unsigned char setPWM (unsigned char device, unsigned char motor, short PWMdata){
    if (abs(PWMdata) > 4095) return (FALSE);
    
    else if (motor == M1){
        if (PWMdata < 0){
            if (!setPCA9685outputs (device, AIN1, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, AIN2, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMA, 0, 0-PWMdata)) return (FALSE);
        }
        else {
            if (!setPCA9685outputs (device, AIN1, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, AIN2, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMA, 0, PWMdata)) return (FALSE);
        }
    }        
    else if (motor == M2){
        if (PWMdata < 0){
            if (!setPCA9685outputs (device, BIN1, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, BIN2, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMB, 0, 0-PWMdata)) return (FALSE);
        }
        else {
            if (!setPCA9685outputs (device, BIN1, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, BIN2, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMB, 0, PWMdata)) return (FALSE);
        }
    }   
    else if (motor == M3){
        if (PWMdata > 0){
            if (!setPCA9685outputs (device, CIN1, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, CIN2, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMC, 0, PWMdata)) return (FALSE);
        }
        else {
            if (!setPCA9685outputs (device, CIN1, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, CIN2, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMC, 0, 0-PWMdata)) return (FALSE);
        }
    }   
    else {
        if (PWMdata > 0){
            if (!setPCA9685outputs (device, DIN1, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, DIN2, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMD, 0, PWMdata)) return (FALSE);
        }
        else {
            if (!setPCA9685outputs (device, DIN1, FULL_ON, 0)) return (FALSE);
            if (!setPCA9685outputs (device, DIN2, 0, 0)) return (FALSE);
            if (!setPCA9685outputs (device, PWMD, 0, 0-PWMdata)) return (FALSE);
        }
    }   
    return(TRUE);
}

#define RD 1
#define WR 0

unsigned char PCAWriteByte (unsigned char device, unsigned char PCAcontrolRegister, unsigned char data)
{

    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    

    MasterWriteI2C(device | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM

    MasterWriteI2C(PCAcontrolRegister); // Send PCA register to be written to
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    MasterWriteI2C(data);
    IdleI2C(); //Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM        

    StopI2C(); // Send the Stop condition
    IdleI2C(); // Wait to complete

    // Wait for EEprom to complete write process
    getWriteAck(device);
    return (1);
}

unsigned char PCAReadByte (unsigned char device, unsigned char PCAcontrolRegister, unsigned char *ptrData)
{
    StartI2C(); // Send the Start Bit
    IdleI2C(); // Wait to complete    

    MasterWriteI2C(device | WR); // Send EEPROM Device ID and WRITE Command    
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM
    
    MasterWriteI2C(PCAcontrolRegister); // Send PCA register to be read
    IdleI2C(); // Wait to complete
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    RestartI2C(); // Now send START sequence again:
    IdleI2C(); // Wait to complete

    MasterWriteI2C(device | RD); // Now send ID with READ Command    
    IdleI2C(); // Wait to complete    
    if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM

    // Now receive data byte:
    I2CCONbits.RCEN = 1;
    while (!DataRdyI2C()); // Wait for incoming data byte
    *ptrData = I2CRCV; // Read data byte from buffer

    I2CCONbits.ACKDT = 1; // Send NACK to EEPROM
    I2CCONbits.ACKEN = 1;
    while (I2CCONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 

    StopI2C();
    IdleI2C();
    return (1); // Return 1 to indicate successful read operation
}



unsigned char initializePCA9685(unsigned char device) 
{ 
    unsigned char dataByte;
    OpenI2C(I2C_EN, 299);
    
    if (!PCAWriteByte (device, ALL_LED_ON, 0)) return FALSE;
    if (!PCAWriteByte (device, ALL_LED_ON+1, 0)) return FALSE;
    if (!PCAWriteByte (device, ALL_LED_OFF, 0)) return FALSE;
    if (!PCAWriteByte (device, ALL_LED_OFF+1, 0)) return FALSE;
 
    // Configure totem pole structured output:    
    if (!PCAReadByte (device, MODE2_REG, &dataByte)) return FALSE;
    dataByte = dataByte | OUTDRV;
    if (!PCAWriteByte (device, MODE2_REG, dataByte)) return FALSE;
            
    // respond to ALL_LED register changes 
    if (!PCAReadByte (device, MODE1_REG, &dataByte)) return FALSE;
    dataByte = dataByte | ALLCALL;
    if (!PCAWriteByte (device, MODE1_REG, dataByte)) return FALSE;
    
    DelayMs(10); // wait for oscillator 

    // To set PWM frequency, first make sure PCA9685 is in SLEEP mode:
    if (!PCAReadByte (device, MODE1_REG, &dataByte)) return FALSE;    
    dataByte = dataByte |= SLEEP_BIT;
    if (!PCAWriteByte (device, MODE1_REG, dataByte)) return FALSE;
    
    // For maximum PWM frequency of 1526 Hz, set PRESCALE register to 0x03h:
    if (!PCAWriteByte (device, PRESCALE, 0x03)) return FALSE;
    
    // Now clear SLEEP bit for normal operation mode:
    dataByte = dataByte &= ~SLEEP_BIT;
    if (!PCAWriteByte (device, MODE1_REG, dataByte)) return FALSE;       
 
    DelayMs(10); // wait for oscillator
    
    // Now enable restart:
    dataByte = dataByte |= RESTART;
    if (!PCAWriteByte (device, MODE1_REG, dataByte)) return FALSE;    

    return (TRUE);
}


unsigned char setPCA9685outputs (unsigned char device, unsigned short channel, unsigned short turnON, unsigned short turnOFF)
{
    unsigned char registerAddress;

    if (channel > MAX_LED_CHANNEL) return (FALSE);    
    registerAddress = LED_ON_REGISTER + (4 * channel);
    
    if (!PCAWriteByte (device, registerAddress, (unsigned char)(turnON & 0xFF))) return FALSE;
    if (!PCAWriteByte (device, registerAddress+1, (unsigned char)((turnON & 0xFF00) >> 8))) return FALSE;        
    if (!PCAWriteByte (device, registerAddress+2, (unsigned char)(turnOFF & 0xFF))) return FALSE;
    if (!PCAWriteByte (device, registerAddress+3, (unsigned char)((turnOFF & 0xFF00) >> 8))) return FALSE;    

    return (TRUE);
}



