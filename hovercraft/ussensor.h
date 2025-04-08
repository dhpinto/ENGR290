#ifndef USSENSOR_H
#define USSENSOR_H
#include <avr/io.h>
#include <avr/interrupt.h>

// Define trigger and echo pins for both sensors
#define TRIG1 PD2  // Trigger pin for sensor 1
#define ECHO1 PD3  // Echo pin for sensor 1
#define TRIG2 PD4  // Trigger pin for sensor 2
#define ECHO2 PD5  // Echo pin for sensor 2

// Function prototypes
void init_ultrasonic(void);
void trigger_sensor1(void);
void trigger_sensor2(void);
void update_sensor_readings(void);
void get_distances(uint16_t *distance1, uint16_t *distance2);

#endif