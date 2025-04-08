#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <avr/io.h>
#include <stdint.h>

#define IR_ADC_CHANNEL 3      // ADC3 -> PC3
#define IR_SENSOR_PIN PC3
#define THRESHOLD_DISTANCE 100 // Distance threshold in mm to detect bar (valeur pour maintenant)
/*
void UART_init(void);
void UART_transmit(char data);
void UART_print(const char* str);
void UART_printNumber(uint16_t num);

//NO NEED BECAUSE DONT NEED TO DISPLAY??
*/


void ir_sensor_init(void);
uint16_t ADC_read(void);
uint16_t sensorValue_to_mm(uint16_t sensorValue);


#endif