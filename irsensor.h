#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <avr/io.h>
#include <stdint.h>

/*
void UART_init(void);
void UART_transmit(char data);
void UART_print(const char* str);
void UART_printNumber(uint16_t num);

//NO NEED BECAUSE DONT NEED TO DISPLAY??
*/


void ADC_init(void);
uint16_t ADC_read(void);
uint16_t sensorValue_to_mm(uint16_t sensorValue);


#endif