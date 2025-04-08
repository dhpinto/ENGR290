#include "ir_sensor.h"
#include <util/delay.h>
#include <avr/io.h>

#define F_CPU 16000000UL
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


// ADC initialization
void ADC_init(void) {
    ADMUX = (1 << REFS0) | (1 << MUX2) | (1 << MUX1);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
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
