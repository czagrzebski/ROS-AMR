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
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
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
#define PUBLISH_UPDATE_SPEED 100

// Stores the number of hall encoder ticks for each motor
volatile long motorATicks = 0;
volatile long motorBTicks = 0;

// Motor Angles (for Joint State Publisher)
float motorAAngle = 0;
float motorBAngle = 0;

// Controller Update Interval
unsigned long updateRate = ((double) (1.0/PID_UPDATE_FREQ) * 1000);
unsigned long lastUpdate = 0;
unsigned long lastOdomUpdate = 0;
unsigned long lastPublish = 0;

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
geometry_msgs::Vector3Stamped speedCtrlMsg;
sensor_msgs::JointState jointStatesMsg;
float jointStateVelocity[2];
float jointStatePosition[2];
float jointStateEffort[2];
char *jointStateNames[2] = {"left_wheel_joint", "right_wheel_joint"};

// ROS Node (Pub and Sub) Initialization
ros::NodeHandle nh;
ros::Publisher jointStates("joint_states", &jointStatesMsg);

void speedCtrlCallback(const geometry_msgs::Vector3Stamped& msg) {
    velSetPointA = msg.vector.x;
    velSetPointB = msg.vector.y;
}

ros::Subscriber<geometry_msgs::Vector3Stamped> speedCtrlSub("wheel_velocity_cmd", &speedCtrlCallback);

void setup() {
    nh.initNode();
    nh.loginfo('Initalize Embedded Motor Controller');

    jointStatesMsg.name_length = 2;
    jointStatesMsg.position_length = 2;
    jointStatesMsg.velocity_length = 2;
    jointStatesMsg.effort_length = 2;
    jointStatesMsg.name = jointStateNames;
   

    nh.subscribe(speedCtrlSub);
    nh.advertise(jointStates);

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

        // Set Motor A Speed
        if(velSetPointA == 0) {
            motorA.stop();
        } else {
            pidMotorA.Compute();
            motorA.setSpeed(pwmMotorA);
        }

        // Set Motor A Direction 
        if(velSetPointA > 0) {
            motorA.forward();
        } else {
            motorA.backward();
        }

        // Set Motor B Speed
        if(velSetPointB == 0) {
            motorB.stop();
        } else {
            pidMotorB.Compute();
            motorB.setSpeed(pwmMotorB);
        }

        // Set Motor B Direction 
        if(velSetPointB > 0) {
            motorB.forward();
        } else {
            motorB.backward();
        }

        lastUpdate = millis();
    }

    if(millis() - lastPublish >= PUBLISH_UPDATE_SPEED) {
        publishState();
        lastPublish = millis();
    }

    nh.spinOnce();
}

// Calculate the current velocity of the motors
void calculateMotorVelocity() {
    float deltaAngleMotorA = ticksToAngle(motorATicks);
    float deltaAngleMotorB = ticksToAngle(motorBTicks);

    // Calculate current velocity of each motor
    velMotorAOutput = angularToLinearVelocity(angleToAngularVelocity(abs(deltaAngleMotorA), 50.0/1000.0), wheelDiameter);
    velMotorBOutput = angularToLinearVelocity(angleToAngularVelocity(abs(deltaAngleMotorB), 50.0/1000.0), wheelDiameter);

    motorAAngle += deltaAngleMotorA;
    motorBAngle += deltaAngleMotorB;

    // Normalize angle to be between 0 and 2PI
    motorAAngle = fmod(motorAAngle, 2 * PI);
    motorBAngle = fmod(motorBAngle, 2 * PI);

    if(motorAAngle < 0) {
        motorAAngle += 2 * PI;
    }

    if(motorBAngle < 0) {
        motorBAngle += 2 * PI;
    }

    // Reset tick count for next iteration
    motorATicks = 0;
    motorBTicks = 0;

    lastOdomUpdate = millis();
}

void publishState() {
    jointStateVelocity[0] = velMotorAOutput;
    jointStateVelocity[1] = velMotorBOutput;
    jointStatePosition[0] = motorAAngle;
    jointStatePosition[1] = motorBAngle;
    jointStateEffort[0] = 0;
    jointStateEffort[1] = 0;
    jointStatesMsg.header.stamp = nh.now();
    jointStatesMsg.name = jointStateNames;
    jointStatesMsg.position = jointStatePosition;
    jointStatesMsg.velocity = jointStateVelocity;
    jointStatesMsg.effort = jointStateEffort;
    jointStates.publish(&jointStatesMsg);
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
    if(digitalRead(motorAHallA) == HIGH) {
        motorATicks++;
    } else {
        motorATicks--;
    }
}

// Interrupt Service Routine for Motor B Encoder
void doMotorBTick() {
    if(digitalRead(motorBHallA) == HIGH) {
        motorBTicks++;
    } else {
        motorBTicks--;
    }
}