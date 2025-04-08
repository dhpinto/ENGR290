#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Pin definitions for first sensor (INT0)
#define TRIG1_PIN PB5
#define ECHO1_PIN PD2  // INT0

// Pin definitions for second sensor (INT1)
#define TRIG2_PIN PB4
#define ECHO2_PIN PD3  // INT1

// Variables to store timing and distances
volatile uint16_t time_start1 = 0;
volatile uint16_t time_start2 = 0;
volatile uint16_t time_overflow1 = 0;
volatile uint16_t time_overflow2 = 0;
volatile uint16_t distance1 = 0;
volatile uint16_t distance2 = 0;
volatile uint8_t measurement_state1 = 0;  // 0: idle, 1: waiting for rising edge, 2: measuring
volatile uint8_t measurement_state2 = 0;  // 0: idle, 1: waiting for rising edge, 2: measuring

// Initialize the ultrasonic sensors and interrupts
void init_ultrasonic(void) {
    // Disable interrupts during initialization
    cli();
    
    // Set TRIG pins as outputs
    DDRB |= (1 << TRIG1_PIN) | (1 << TRIG2_PIN);
    
    // Set ECHO pins as inputs and disable pull-ups
    DDRD &= ~((1 << ECHO1_PIN) | (1 << ECHO2_PIN));
    PORTD &= ~((1 << ECHO1_PIN) | (1 << ECHO2_PIN));
    
    // Initialize TRIG pins to low
    PORTB &= ~((1 << TRIG1_PIN) | (1 << TRIG2_PIN));
    
    // Configure INT0 and INT1 for any edge detection
    EICRA |= (1 << ISC00) | (1 << ISC01) | (1 << ISC10) | (1 << ISC11);
    
    // Enable INT0 and INT1 interrupts
    EIMSK |= (1 << INT0) | (1 << INT1);
    
    // Initialize variables
    time_start1 = 0;
    time_start2 = 0;
    time_overflow1 = 0;
    time_overflow2 = 0;
    distance1 = 0;
    distance2 = 0;
    measurement_state1 = 0;
    measurement_state2 = 0;
    
    // Re-enable interrupts
    sei();
}

// Function to trigger first sensor
void trigger_sensor1(void) {
    // Disable Timer1 overflow interrupt temporarily
    TIMSK1 &= ~(1 << TOIE1);
    
    PORTB |= (1 << TRIG1_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG1_PIN);
    
    // Re-enable Timer1 overflow interrupt
    TIMSK1 |= (1 << TOIE1);
}

// Function to trigger second sensor
void trigger_sensor2(void) {
    // Disable Timer1 overflow interrupt temporarily
    TIMSK1 &= ~(1 << TOIE1);
    
    PORTB |= (1 << TRIG2_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG2_PIN);
    
    // Re-enable Timer1 overflow interrupt
    TIMSK1 |= (1 << TOIE1);
}

// Function to update both sensor readings
void update_sensor_readings(void) {
    // Reset measurement states
    measurement_state1 = 0;
    measurement_state2 = 0;
    
    // Trigger sensors with delay between them
    trigger_sensor1();
    _delay_ms(10);  // Small delay between readings
    trigger_sensor2();
}

// Function to get the current readings
void get_distances(uint16_t *dist1, uint16_t *dist2) {
    *dist1 = distance1;
    *dist2 = distance2;
}

// INT0 ISR for first sensor
ISR(INT0_vect) {
    if (PIND & (1 << ECHO1_PIN)) {  // Rising edge
        if (measurement_state1 == 0) {  // Only process if we're not already measuring
            time_start1 = TCNT1;
            time_overflow1 = 0;
            measurement_state1 = 2;  // Set to measuring state
        }
    } else {  // Falling edge
        if (measurement_state1 == 2) {  // Only process if we were actually measuring
            uint16_t time_end = TCNT1;
            uint16_t duration;
            
            if (time_end >= time_start1) {
                duration = time_end - time_start1;
            } else {
                duration = (65535 - time_start1) + time_end + 1;
            }
            
            // Add overflow count to duration
            duration += (time_overflow1 * 65536);
            
            // Calculate distance (speed of sound = 340 m/s = 34000 cm/s)
            // Timer1 ticks at 20ms/2500 = 8us per tick
            // duration is in timer ticks, so multiply by 8 to get microseconds
            // Then divide by 58 to get distance in cm
            distance1 = (duration * 8) / 58;
            
            measurement_state1 = 0;  // Reset to idle state
        }
    }
}

// INT1 ISR for second sensor
ISR(INT1_vect) {
    if (PIND & (1 << ECHO2_PIN)) {  // Rising edge
        if (measurement_state2 == 0) {  // Only process if we're not already measuring
            time_start2 = TCNT1;
            time_overflow2 = 0;
            measurement_state2 = 2;  // Set to measuring state
        }
    } else {  // Falling edge
        if (measurement_state2 == 2) {  // Only process if we were actually measuring
            uint16_t time_end = TCNT1;
            uint16_t duration;
            
            if (time_end >= time_start2) {
                duration = time_end - time_start2;
            } else {
                duration = (65535 - time_start2) + time_end + 1;
            }
            
            // Add overflow count to duration
            duration += (time_overflow2 * 65536);
            
            // Calculate distance (speed of sound = 340 m/s = 34000 cm/s)
            // Timer1 ticks at 20ms/2500 = 8us per tick
            // duration is in timer ticks, so multiply by 8 to get microseconds
            // Then divide by 58 to get distance in cm
            distance2 = (duration * 8) / 58;
            
            measurement_state2 = 0;  // Reset to idle state
        }
    }
}

// Timer1 overflow ISR
ISR(TIMER1_OVF_vect) {
    // Increment overflow counters if we're in the middle of a measurement
    if (measurement_state1 == 2) {
        time_overflow1++;
    }
    if (measurement_state2 == 2) {
        time_overflow2++;
    }
}
