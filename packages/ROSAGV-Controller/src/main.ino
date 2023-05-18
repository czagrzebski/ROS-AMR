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
#include <geometry_msgs/Vector3Stamped.h>
#include "motor.h"

// Encoder Ticks per Revolution
#define ENC_COUNT_REV 576.0

// Hall Sensor Pins for Motor A & B
#define motorAHallA 2
#define motorAHallB 3
#define motorBHallA 18
#define motorBHallB 19

// PID Update Frequency (in Hz)
#define PID_UPDATE_FREQ 20.0

// Publish Update Frequency (in ms)
#define PUBLISH_UPDATE_SPEED 10

// Stores the number of hall encoder ticks for each motor
volatile long motorATicks = 0;
volatile long motorBTicks = 0;

// Controller Update Interval
unsigned long updateRate = ((double) (1.0/PID_UPDATE_FREQ) * 1000);
unsigned long lastUpdate = 0;
unsigned long lastOdomUpdate = 0;

// Diameter of the wheel (in mm)
const double wheelDiameter = 65.0; 

// PID Control Variables
double velSetPointA = 0.4; // the setpoint for motor A (in m/s)
double velMotorAOutput = 0; // process output for motor A (in m/s)
double pwmMotorA = 0; // PWM value for motor A

double velSetPointB = 0.4; // the setpoint for motor B (in m/s)
double velMotorBOutput = 0; // process output for motor B (in m/s)
double pwmMotorB = 0; // PWM value for motor B

// TODO: Get from ROS Parameter Server
double kp = 325; // proportional Constant
double ki = 600; // integral constant
double kd = 0; // derivative constant

// Setup PID Controller for both variables
PID pidMotorA = PID(&velMotorAOutput, &pwmMotorA, &velSetPointA, kp, ki, kd, DIRECT);
PID pidMotorB = PID(&velMotorBOutput, &pwmMotorB, &velSetPointB, kp, ki, kd, DIRECT);

// Setup Motors (Pins are currently undefined)
Motor motorA = Motor(8, 9, 10);
Motor motorB = Motor(11, 12, 13);

// ROS Messages
geometry_msgs::Vector3Stamped speedMsg;
geometry_msgs::Vector3Stamped speedCtrlMsg;

// ROS Node (Pub and Sub) Initialization
ros::NodeHandle nh;
ros::Publisher speedPub("wheel_velocity", &speedMsg);

void speedCtrlCallback(const geometry_msgs::Vector3Stamped& msg) {
    velSetPointA = msg.vector.x;
    velSetPointB = msg.vector.y;
}

ros::Subscriber<geometry_msgs::Vector3Stamped> speedCtrlSub("wheel_velocity_cmd", &speedCtrlCallback);

void setup() {
    nh.initNode();
    nh.loginfo('Initalize Embedded Motor Controller');

    nh.advertise(speedPub);
    nh.subscribe(speedCtrlSub);

    pidMotorA.SetSampleTime(updateRate);
    pidMotorA.SetOutputLimits(0, 255);
    motorA.setSpeed(0);
    motorA.forward();
    pidMotorA.SetMode(AUTOMATIC);

    pidMotorB.SetSampleTime(updateRate);
    pidMotorB.SetOutputLimits(0, 255);
    motorB.setSpeed(0);
    motorB.forward();
    pidMotorB.SetMode(AUTOMATIC);

    // Attach Interrupts to ISR for Hall Sensors (Motor A)
    attachInterrupt(digitalPinToInterrupt(motorAHallA), doMotorATick, RISING);
    attachInterrupt(digitalPinToInterrupt(motorAHallA), doMotorATick, FALLING);
    attachInterrupt(digitalPinToInterrupt(motorAHallB), doMotorATick, RISING);
    attachInterrupt(digitalPinToInterrupt(motorAHallB), doMotorATick, FALLING);

    // Attach Interrupts to ISR for Hall Sensors (Motor B)
    attachInterrupt(digitalPinToInterrupt(motorBHallA), doMotorBTick, RISING);
    attachInterrupt(digitalPinToInterrupt(motorBHallA), doMotorBTick, FALLING);
    attachInterrupt(digitalPinToInterrupt(motorBHallB), doMotorBTick, RISING);
    attachInterrupt(digitalPinToInterrupt(motorBHallB), doMotorBTick, FALLING);

    nh.loginfo('Initialization Complete');

}

void loop() {
    if(millis() - lastUpdate >= updateRate) {
        calculateMotorVelocity();
        pidMotorA.Compute();
        pidMotorB.Compute();
        motorA.setSpeed(pwmMotorA);
        motorB.setSpeed(pwmMotorB);

        // Print out PID values for motor A
        //Serial.print(velSetPointA);
        //Serial.print("\t");
        //Serial.print(pwmMotorA);
        //Serial.print("\t");
        //Serial.print(velMotorAOutput);

        //Serial.print("\t");

        // Print out PID values for motor B
        //Serial.print(pwmMotorB);
        //Serial.print("\t");
        //Serial.println(velMotorBOutput);

        publishSpeed();
        lastUpdate = millis();
    }

    nh.spinOnce();
}

// Calculate the current velocity of the motors
void calculateMotorVelocity() {

    // Calculate current velocity of each motor
    velMotorAOutput = angularToLinearVelocity(angleToAngularVelocity(ticksToAngle(motorATicks), 50.0/1000.0), wheelDiameter);
    velMotorBOutput = angularToLinearVelocity(angleToAngularVelocity(ticksToAngle(motorBTicks), 50.0/1000.0), wheelDiameter);

    // Reset tick count for next iteration
    motorATicks = 0;
    motorBTicks = 0;

    lastOdomUpdate = millis();

}

void publishSpeed() {
    // Construct ROS Message
    speedMsg.header.stamp = nh.now();
    speedMsg.vector.x = velMotorAOutput;
    speedMsg.vector.y = velMotorBOutput;
    speedMsg.vector.z = 0;
    speedPub.publish(&speedMsg);
}

// Convert ticks to angle (in radians)
double ticksToAngle(long ticks) { 
    return ((double) ticks / ENC_COUNT_REV) * 2.0 * PI;
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

void doMotorBTick() {
    motorBTicks++;
}

