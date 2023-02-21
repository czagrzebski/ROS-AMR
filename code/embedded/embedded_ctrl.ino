/**
 * Low-level Embedded Motor Controller for a Differential Drive AGV
 * 
 * Handles velocity control of DC motors using PID controllers and L298N
 * motor drivers. Feeds output back to hig h-level controller using ROSSerial
 * 
 * @author Creed Zagrzebski (czagrzebski@gmail.com)
*/

#include <Arduino.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include "motor.h"

// Number of motor encoder ticks per shaft revolution (from ticker counter program)
#define ENC_COUNT_REV 146

// Hall Sensor Pins for Motor A & B
#define motorAHallA 2
#define motorAHallB 3
#define motorBHallA 18
#define motorBHallB 19

#define MOTOR_A 0
#define MOTOR_B 1

// Stores the number of hall encoder ticks for each motor
volatile long motorATicks = 0;
volatile long motorBTicks = 0;

// Controller Update Interval
const int updateRate = 1000;
long lastUpdate = 0;

// Diameter of the wheels (in mm)
const int wheelDiameter = 65;

// PID Control Variables
double velSetPointA = 0; // velocity setpoint for motor A
double velSetPointB = 0; // velocity setpoint for motor B
double velMotorAOutput = 0; // process output for motor A (calculated velocity)
double velMotorBOutput = 0; // process output for motor B (calculated velocity)
double pwmMotorA = 0; // the control variable for motor A
double pwmMotorB = 0; // the control variable for motor B
double kp = 0;
double ki = 0;
double kd = 0;

// Setup PID Controller for both variables
PID pidMotorA = PID(&pwmMotorA, &velMotorAOutput, &velSetPointA, kp, ki, kd, DIRECT);
PID pidMotorB = PID(&pwmMotorB, &velMotorBOutput, &velSetPointB, kp, ki, kd, DIRECT);

// Setup Motors (Pins are currently undefined)
Motor motorA = Motor(0, 0, 0);
Motor motorB = Motor(0, 0, 0);

// ROS Node Communication
ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher motor_data("motor_data", &str_msg);

// Test Communication
char msg[5] = "Test";

void setup() {
    // Turn on PID control
    pidMotorA.SetMode(AUTOMATIC);
    pidMotorB.SetMode(AUTOMATIC);

    // Set how often the PID will be evaluated (50ms)
    pidMotorA.SetSampleTime(25);
    pidMotorB.SetSampleTime(25);

    // Set Max PWM Range
    pidMotorA.SetOutputLimits(0, 255);
    pidMotorB.SetOutputLimits(0, 255);

    // Attach Interrupts to ISR
    attachInterrupt(digitalPinToInterrupt(motorAHallA), doMotorATick, RISING);
    attachInterrupt(digitalPinToInterrupt(motorBHallB), doMotorBTick, RISING);

    // Initialize ROS 
    nh.initNode();
    nh.advertise(motor_data);
}

void loop() {
   if(millis() - lastUpdate >= updateRate) {
        // Calculate new setpoint
        calculateMotorVelocity();
        pidMotorA.Compute();
        pidMotorB.Compute();

        // TODO: Direction Control, Halt Motors
        // Set Speed
        motorA.setSpeed(pwmMotorA);
        motorB.setSpeed(pwmMotorB);
        
        // Publish ROS Data
        str_msg.data = msg;
        motor_data.publish(&str_msg);
        nh.spinOnce();

        lastUpdate = millis();
   }
}

void calculateMotorVelocity() {
    velMotorAOutput = ((((motorATicks * 60)/ENC_COUNT_REV) * (PI * wheelDiameter))/60)/1000;
    velMotorBOutput = ((((motorBTicks * 60)/ENC_COUNT_REV) * (PI * wheelDiameter))/60)/1000;
    
    motorATicks = 0;
    motorBTicks = 0;
}

// Interrupt Service Routine for Motor A Encoder
void doMotorATick() {
    motorATicks++;
}

// Interrupt Service Routine for Motor B Encoder
void doMotorBTick() {
    motorBTicks++;
}