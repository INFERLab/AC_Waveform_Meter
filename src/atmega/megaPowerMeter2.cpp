#include <SPI.h>

#define FOSC 16000000 // Clock speed
#define BAUD 2000000 // 2Mbps
#define MYUBRR 0 //FOSC/8/BAUD -1

/** Pin mapping for ADC, Arduino UNO, Atmega328P
 *   ADC     ARDUINO UNO    Atmega328P
 *   BUSY        PIN3          PD03
 *  RESET        PIN4          PD04
 *  CONVST       PIN5          PD05
 *    CS        PIN10        PB02(SS)
 *   DB07       PIN12        PB04(MISO)
 *    RD        PIN13        PB05(SCK)
 */

// Global Variables
#define TOTAL_RAW_BYTES 16 // #channels x #bytes per channel = 8 x 2 = 16
#define MAX_CHANNELS 8 
#define NUM_CHANNELS 2 // WE ARE TRYING WITH 2 CHANNELS SIMULTANEOUS

#include "Arduino.h"
void setup();
void loop();
void USART_Init(unsigned int ubrr);
void USART_Transmit(signed char data);
#line 22


byte rawCode[220][TOTAL_RAW_BYTES];

// for testing purposes
unsigned long timeCycle;

short last_voltage;
short voltage;
short cycle_num;
bool collecting = false;


/**
 * setup()
 * Purpose: Port state definition and initial values
 */
void setup() {
    // Output: PD05/CONVST, PD04/RESET
    // Input: PD03/BUSY
    DDRD |= 0b00110000;
                
    // Output: PB05(SCK)/RD, PB02(SS)/CS
    // Input: PB04(MISO)/DB07
    DDRB |= 0b00100100;
    
    // For external serial communication (e.g. PC, Raspberry Pi)
    USART_Init(MYUBRR);
          
    /**
     * Enable SPI, set as Master
     * Prescaler: Fosc/2
     * MSB first (that's how the ADC operates)
     * SPI Mode 2 (Sample Falling, Setup Rising)
     */
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL);
    SPSR |= (1 << SPI2X);

    // Set PD05/CONVST to HIGH
    PORTD |= 0b00100000;  
    // Set PB02/CS to HIGH
    PORTB |= 0b00000100;
    // Set PD04/RESET to HIGH
    PORTD |= 0b00010000;  
    delayMicroseconds(1);
    // Set PD04/RESET to LOW
    PORTD &= 0b11101111;
}

/**
 * loop()
 * Purpose: infinite loop to sample data with the ADC
 */
void loop() {    
    // Set PD05/CONVST to LOW
    PORTD &= 0b11011111;
    //delayMicroseconds(10);
    // Set PD05/CONVST to HIGH
    PORTD |= 0b00100000;
 
    // for debugging purposes
    //timeCycle = millis();
    delayMicroseconds(1);

    // wait for conversion to complete, if BUSY is HIGH while conversion
    while ((PIND & 0b00001000) == 8);

    // Set PB02/CS to LOW
    PORTB &= 0b11111011;
                      
    /**
     * Read two bytes (16 bits) for each channel from the ADC, 
     * starting with the MSB and channel 1 (V1).
     * Final value of the array will be:
     * rawCode[0]   MSByte V1
     * rawCode[1]   LSByte V1
     *           ...
     * rawCode[14]  MSByte V8
     * rawCode[15]  LSByte V8
     */
    
    for(int i = 0; i < TOTAL_RAW_BYTES; i++) {
        rawCode[cycle_num][i] = SPI.transfer(0x00);
    }
                
    // Set PB02/CS to HIGH
    PORTB |= 0b00000100;


    voltage = (rawCode[cycle_num][0] << 8 ) | (rawCode[cycle_num][1]);


    if(collecting){
	
	if (voltage < 0 && last_voltage > 0){

	   //zero-crossing detected and therefore transmitting
	   USART_Transmit('a');
    	   USART_Transmit((byte) cycle_num);
	   USART_Transmit((byte) (cycle_num>>8));
	   

	   for(int j = 0; j < cycle_num; j++) {
		for(int i = 0; i < TOTAL_RAW_BYTES; i++) {
		    USART_Transmit(rawCode[j][i]);
		}
    	   }

	   
	   //transmission done

	   collecting = false;
	   cycle_num = 0;
       }
    }

    if(!collecting){
	if ((voltage > 0 && last_voltage < 0)){
    	   collecting = true;
	   cycle_num = 0;
       }
    }

    cycle_num++;


    last_voltage = voltage;
}

/**
 * USART_Init()
 * Purpose: Initialize the microcontroller's USART with the specified
 * baud rate, converted to the UBRR (FOSC/8/BAUD -1) register value
 */
void USART_Init(unsigned int ubrr) {
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    // Enable Rx and TX
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    // Set frame format: 8 data, no parity, 1 stop bit
    // Default values for UCSR0C
    // Double speed for transmission
    UCSR0A |= (1 << U2X0);
}

/**
 * USART_Transmit
 * Purpose: Transmit serial data in an 8bit frame, 1 stop bit, no parity bit.
 */
void USART_Transmit(signed char data) {
    // Wait for empty transmit buffer
    while(!(UCSR0A & (1 << UDRE0)));
    // Put data into buffer, sends data
    UDR0 = data;
}
                

