#include "RingBuf.h"
#include "TinyGps-submodule/src/TinyGPS.h"
#ifndef _UARThandler_h
#define _UARThandler_h

#define F_CPU 16000000UL 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define HI(x) ((x)>>8)
#define LO(x) ((x)& 0xFF)

#define BAUDRATE 9600L  
#define BAUDRATE_REG (F_CPU/(16*BAUDRATE)-1) 
	
RingBuf<uint32_t, 128> buf_write;
RingBuf<uint32_t, 128> buf_read;
TinyGPSPlus gps;

void init_UART() {

  UBRR0H = HI(BAUDRATE_REG);
	UBRR0L = LO(BAUDRATE_REG);

	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);

}

uint8_t receive(){
  while(!(UCSR0A & (1 << RXC0))) ;	
  uint32_t value = UDR0;
  return UDR0;
}
void send(){


  while(buf_write.availableForRead()){
    while(!(UCSR0A & (1 << UDRIE0))) ;	
	  UDR0 = buf_write.read();	
  }

}

#endif