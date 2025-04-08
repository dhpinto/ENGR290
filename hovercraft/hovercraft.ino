#include <avr/io.h>
#include "init_290.c"
#include <stdio.h>
#include <avr/delay.h>
#include "TWI_290.c"
#include "ussensor.h"
#include "ir_sensor.h"
// Port definitions for fans
#define LIFT_FAN_PORT    PORTD
#define LIFT_FAN_DDR     DDRD
#define LIFT_FAN_PIN     PD7    // On/Off channel 0

#define FRONT_FAN_PORT   PORTD
#define FRONT_FAN_DDR    DDRD
#define FRONT_FAN_PIN    PD4    // On/Off channel 1

#define SERVO_PORT   PORTB
#define SERVO_DDR    DDRB
#define SERVO_PIN    PB1  // OCR1A est sur PB1 (Arduino Pin 9)

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

void set_servo_position(uint8_t angle) {
    OCR1A = Servo_angle[angle]; // position
}

// Function to test fans
void test_fans(void) {
    // Test lift fan
    set_lift_fan(200);  // Set lift fan to 78% speed
    _delay_ms(2000);    // REMPLACE CA PAR LA DETECTION DE US SENSOR
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
    uart_init();
    timer1_50Hz_init(1);  // Initialize Timer1 for servo
    timer0_init();        // Initialize Timer0 for PWM fans
    init_fans();          // Initialize fan ports
    init_ultrasonic();    
    //ADC_init();           // Initialize ADC for IR sensor
    
   uint16_t dist1, dist2;
   sei();
    while (1) {
        


       update_sensor_readings();
      get_distances(&dist1, &dist2);
        // Optional: Run both fans together
        set_servo_position(127);
        set_lift_fan(180);   // 70% speed
        set_front_fan(180);  // 70% speed
        _delay_ms(3000);     // Run for 3 seconds

        
        
        // Stop all fans (PUT IT IN A IF QUAND L'INFRAROUGE EST DETECTER)
       
    //uint16_t ir_distance = sensorValue_to_mm(ADC_read());
    //if (ir_distance < IR_THRESHOLD) {  
        set_servo_position(0);
        set_lift_fan(0);
        set_front_fan(0);
        break;
    //}
       

        _delay_ms(2000);     // Wait 2 seconds before next test
    }
}