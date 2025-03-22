#include <avr/io.h>
#include "init_290.c"
#include <stdio.h>
#include <avr/delay.h>
#include "TWI_290.c"

// SERVO MOTOR CONTROL
void init_servo(void) {
    // Timer1 is already configured for PWM in init_290.c
    // We just need to set initial position to middle
    OCR1A = Servo_angle[127];  // Set to middle position
}

// Function to set servo angle (-90 to +90 degrees)
void set_servo_angle(int8_t angle) {
    // Map angle from -90 to +90 to array index 0 to 255
    uint8_t index;
    if (angle <= -90) {
        index = 0;
    } else if (angle >= 90) {
        index = 255;
    } else {
        index = (angle + 90) * 255 / 180;
    }
    
    // Set servo position using lookup table
    OCR1A = Servo_angle[index];
}

// Function to lift fan for direction change
void lift_fan(void) {
    // Lift fan to 45 degrees
    set_servo_angle(45);
    _delay_ms(500);  // Wait for servo to reach position
}

// Function to lower fan back to normal position
void lower_fan(void) {
    // Return to 0 degrees (horizontal position)
    set_servo_angle(0);
    _delay_ms(500);  // Wait for servo to reach position
}

// Function to change direction with fan lift
void change_direction(void) {
    lift_fan();
    _delay_ms(1000);  // Wait while changing direction
    lower_fan();
}

// Main function
int main(void) {
    // Initialize systems
    init_290();
    init_servo();
    
    // Initial position - fan horizontal
    set_servo_angle(0);
    
    while(1) {
        // Your main hovercraft control loop here
        
        // Example of direction change:
        // change_direction();
        // _delay_ms(2000);  // Wait before next direction change
    }
}

