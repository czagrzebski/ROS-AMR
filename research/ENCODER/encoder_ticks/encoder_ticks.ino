/*
* DC Rotary Encoder Tick Counter 
*
* Reads input from the optical/hall sensor of a DC motor to determine 
* the number of ticks within a single revolution of the output shaft
*
* @author Creed Zagrzebski (czagrzebski@gmail.com)
*
*/

#include <Arduino.h>

const byte hallA = 2; // CLK Input
const byte hallB = 3; // DT Input
volatile int encoderTicks = 0; // Number of ticks (volatile )

void setup() {
  Serial.begin(9600);
  pinMode(hallA, INPUT);
  pinMode(hallB, INPUT);
  attachInterrupt(digitalPinToInterrupt(hallA), onHallTrigger, RISING);
}

void loop() {
  Serial.println(encoderTicks);
}

// Hall Interrupt Service Routine (ISR)
void onHallTrigger() {
  if(digitalRead(hallA) != digitalRead(hallB)) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}
