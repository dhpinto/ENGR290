#include "ir_sensor.h"
#include <util/delay.h>
#include <avr/io.h>
#include "init_290.h"

#define F_CPU 16000000UL
#define IR_ADC_CHANNEL 3 // PC3 -> ADC3
#define IR_SENSOR_PIN PC3 
#define THRESHOLD_DISTANCE 100 // Distance threshold in mm to detect bar (valeur pour maintenant)

/*
void UART_init(void) {
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void UART_print(const char* str) {
    while (*str) UART_transmit(*str++);
}

void UART_printNumber(uint16_t num) {
    char buffer[10];
    itoa(num, buffer, 10);
    UART_print(buffer);
}
*/

void ir_sensor_init(void){
    adc_init(IR_ADC_CHANNEL,0); // Initialize ADC for IR sensor
}


// ADC read function
uint16_t ADC_read(void) {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}


uint16_t sensorValue_to_mm(uint16_t sensorValue) {
    uint32_t voltage_mV = sensorValue * 5000UL / 1023;
    if (voltage_mV < 100) voltage_mV = 100;
    return 27860UL / (voltage_mV - 100);
}
