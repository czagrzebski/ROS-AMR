/**
 * Low-level Embedded Motor Controller for the Differential Drive AGV
 * 
 * Controls and monitors speed of DC motors using PID controllers and L298N
 * motor drivers. Feeds output back to high-level controller using ROSSerial
 * 
 * @author Creed Zagrzebski (czagrzebski@gmail.com)
*/

#include <Arduino.h>
#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/String.h>
#include "motor.h"

// Encoder Ticks per Revolution
#define ENC_COUNT_REV 146.0

// Hall Sensor Pins for Motor A & B
#define motorAHallA 2
#define motorAHallB 3
#define motorBHallA 18
#define motorBHallB 19

// PID Update Frequency (in Hz)
#define PID_UPDATE_FREQ 100

// Stores the number of hall encoder ticks for each motor
volatile long motorATicks = 0;
volatile long motorBTicks = 0;

// Controller Update Interval
const int updateRate = (1/PID_UPDATE_FREQ) * 1000;
long lastUpdate = 0;
long lastOdomUpdate = 0;

// Diameter of the wheel (in mm)
const double wheelDiameter = 65.0; 

// PID Control Variables
double velSetPointA; // the setpoint for motor A (in m/s)
double velSetPointB; // the setpoint for motor B (in m/s
double velMotorAOutput; // process output for motor A (in m/s)
double velMotorBOutput; // process output for motor B (in m/s)
double pwmMotorA; // PWM value for motor A
double pwmMotorB; // PWM value for motor B

// TODO: Get from ROS Parameter Server
double kp = 0.1; // proportional Constant
double ki = 0.3; // integral constant
double kd = 0.001; // derivative constant

// Setup PID Controller for both variables
PID pidMotorA = PID(&velMotorAOutput, &pwmMotorA, &velSetPointA, kp, ki, kd, DIRECT);
PID pidMotorB = PID(&velMotorBOutput, &pwmMotorB, &velSetPointB, kp, ki, kd, DIRECT);

// Setup Motors (Pins are currently undefined)
Motor motorA = Motor(0, 0, 0);
Motor motorB = Motor(0, 0, 0);

// ROS Node Serial Communication
ros::NodeHandle nh;
std_msgs::String str_msg;
geometry_msgs::Vector3Stamped speed_msg;   
ros::Publisher speed("speed", &speed_msg);

void setup() {
    Serial.begin(9600);

    // Set PID Mode to Automatic
    pidMotorA.SetMode(AUTOMATIC);
    pidMotorB.SetMode(AUTOMATIC);

    // Disable PID Sample Time
    pidMotorA.SetSampleTime(1);
    pidMotorB.SetSampleTime(1);

    // Set Max PWM Range
    pidMotorA.SetOutputLimits(0, 255);
    pidMotorB.SetOutputLimits(0, 255);

    // Set Initial Setpoints for testing (in m/s)
    velSetPointA = 0.25;
    velSetPointB = 0.25;

    // Attach Interrupts to ISR
    attachInterrupt(digitalPinToInterrupt(motorAHallA), doMotorATick, RISING);
    attachInterrupt(digitalPinToInterrupt(motorBHallA), doMotorBTick, RISING);

    // Initialize ROS Node and Publisher
    //nh.initNode();
    //nh.advertise(speed);
}

void loop() {
   if(millis() - lastUpdate >= updateRate) {
        // Calculate Current velocity 
        calculateMotorVelocity();
        
        // Calculate new PWM params
        pidMotorA.Compute();
        pidMotorB.Compute();

        // Set Speed using new values
        motorA.setSpeed(pwmMotorA);
        motorB.setSpeed(pwmMotorB);

        lastUpdate = millis();
   }

   // Check for async callbacks
//   nh.spinOnce();
}

// Calculate the current velocity of the motors
void calculateMotorVelocity() {
    double dts = 0;

    // Calculate time elapsed since last odom update
    dts = (millis() - lastOdomUpdate)/1000.0;

    // Calculate current velocity of each motor
    velMotorAOutput = angularToLinearVelocity(angleToAngularVelocity(ticksToAngle(motorATicks), dts), wheelDiameter/2);
    velMotorBOutput = angularToLinearVelocity(angleToAngularVelocity(ticksToAngle(motorBTicks), dts), wheelDiameter/2);

    // Reset tick count for next iteration
    motorATicks = 0;
    motorBTicks = 0;

    lastOdomUpdate = millis();

    // For Debugging
    //Serial.print("velocity:");
    //Serial.print(velMotorAOutput);
    //Serial.println();

}

// Convert ticks to angle (in radians)
double ticksToAngle(long ticks) { 
    return (double) (ticks / ENC_COUNT_REV) * 2 * PI;
}

// Convert angle (in radians) to angular velocity (in rad/s)
double angleToAngularVelocity(double angle, double dt) {
    return angle / dt;
}

// Convert angular velocity (in rad/s) to linear velocity (in m/s)
double angularToLinearVelocity(double angularVelocity, double wheelRadius) { 
    return angularVelocity * (wheelRadius/1000.0);
}   

// Interrupt Service Routine for Motor A Encoder
void doMotorATick() {
    motorATicks++;
}

// Interrupt Service Routine for Motor B Encoder
void doMotorBTick() {
    motorBTicks++;
}
