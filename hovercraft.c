#include <avr/io.h>
#include "init_290.c"
#include <stdio.h>
#include <avr/delay.h>
#include "TWI_290.c"

//move_forward(200) ***IN THE MAIN FILE

OCR0A = 200; //motor for fan 1

void move_forward(uint8_t speed) {
  OCR0A = speed;
}

int main() {


  
}