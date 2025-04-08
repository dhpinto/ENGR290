#include <avr/io.h>
#include "init_290.c"
#include <stdio.h>
#include <avr/delay.h>
#include "TWI_290.c"

// Port definitions for fans
#define LIFT_FAN_PORT    PORTD
#define LIFT_FAN_DDR     DDRD
#define LIFT_FAN_PIN     PD7    // On/Off channel 0

#define FRONT_FAN_PORT   PORTD
#define FRONT_FAN_DDR    DDRD
#define FRONT_FAN_PIN    PD4    // On/Off channel 1

// Fan control functions
void init_fans(void) {
    // Configure PWM pins as outputs
    DDRD |= (1 << PD7) | (1 << PD4);  // Set pins as outputs
    
    // Initialize pins to low
    PORTD &= ~((1 << PD7) | (1 << PD4));
}

void set_lift_fan(uint8_t speed) {
    if (speed > 0) {
        LIFT_FAN_PORT |= (1 << LIFT_FAN_PIN);  // Turn on fan
        OCR0A = speed;  // PWM1 - Lift fan control
    } else {
        LIFT_FAN_PORT &= ~(1 << LIFT_FAN_PIN); // Turn off fan
        OCR0A = 0;
    }
}

void set_front_fan(uint8_t speed) {
    if (speed > 0) {
        FRONT_FAN_PORT |= (1 << FRONT_FAN_PIN);  // Turn on fan
        OCR0B = speed;  // PWM2 - Front fan control
    } else {
        FRONT_FAN_PORT &= ~(1 << FRONT_FAN_PIN); // Turn off fan
        OCR0B = 0;
    }
}

// Function to test fans
void test_fans(void) {
    // Test lift fan
    set_lift_fan(200);  // Set lift fan to 78% speed
    _delay_ms(2000);    // Run for 2 seconds
    set_lift_fan(0);    // Stop lift fan
    _delay_ms(1000);    // Wait 1 second
    
    // Test front fan
    set_front_fan(200); // Set front fan to 78% speed
    _delay_ms(2000);    // Run for 2 seconds
    set_front_fan(0);   // Stop front fan
    _delay_ms(1000);    // Wait 1 second
}

int main() {
    // Initialize systems
    gpio_init();
    uart_init(9600);
    timer1_50Hz_init(1);  // Initialize Timer1 for servo
    timer0_init();        // Initialize Timer0 for PWM fans
    init_fans();          // Initialize fan ports
    
    // Set servo to middle position
    OCR1A = Servo_angle[127];
    
    while (1) {
        // Test sequence
        test_fans();
        
        // Optional: Run both fans together
        set_lift_fan(180);   // 70% speed
        set_front_fan(180);  // 70% speed
        _delay_ms(3000);     // Run for 3 seconds
        
        // Stop all fans
        set_lift_fan(0);
        set_front_fan(0);
        _delay_ms(2000);     // Wait 2 seconds before next test
    }
}