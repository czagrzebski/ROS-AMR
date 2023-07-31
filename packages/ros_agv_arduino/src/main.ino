/**
 * Low-level Embedded Motor Controller for the Differential Drive AGV
 * 
 * Controls and monitors speed of DC motors using PID controllers and L298N
 * motor drivers. Feeds output back to high-level controller using ROSSerial
 * 
 * @author Creed Zagrzebski (czagrzebski@gmail.com)
*/

#include <Arduino.h>
#include <PID_Plus.h>
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include "motor.h"
#include <Encoder.h>

// Encoder Ticks per Revolution
#define ENC_COUNT_REV 576.0

// Hall Sensor Pins for Motor A & B
#define motorAHallA 2
#define motorAHallB 3
#define motorBHallA 18
#define motorBHallB 19

// PID Update Frequency (in Hz)
#define PID_UPDATE_FREQ 20.0

// Publish Update Frequency (in Hz)
#define PUBLISH_UPDATE_SPEED 20.0

// Stores the number of hall encoder ticks for each motor
int previousMotorATicks = 0;
int previousMotorBTicks = 0;

// Motor Angles (for Joint State Publisher)
float motorAAngle = 0;
float motorBAngle = 0;

// Controller Update Interval
double updateRate = (1.0 / PID_UPDATE_FREQ) * 1000.0;
long lastUpdate = 0;
long lastPublish = 0;

// PID Control Variables
double velSetPointA = 0; // the setpoint for motor A (in rad/s)
double velMotorAOutput = 0; // process output for motor A (in rad/s)
double pwmMotorA = 0; // PWM value for motor A

double velSetPointB = 0; // the setpoint for motor B (in rad/s)
double velMotorBOutput = 0; // process output for motor B (in rad/s)
double pwmMotorB = 0; // PWM value for motor B

long prev_update_time = 0;

// Motor PID Constants
double kf = 17.0;
double kp = 12.0; // proportional Constant
double ki = 12.0; // integral constant
double kd = 0.0; // derivative constant

// Setup PID Controller for both variables
PID pidMotorA = PID(&velSetPointA, &velMotorAOutput, &pwmMotorA);
PID pidMotorB = PID(&velSetPointB, &velMotorBOutput, &pwmMotorB);

Encoder motorAEnc(motorAHallA, motorAHallB);
Encoder motorBEnc(motorBHallA, motorBHallB);

Motor motorA = Motor(8, 9, 10);
Motor motorB = Motor(13, 12, 11);

// ROS Messages
geometry_msgs::Vector3Stamped speedCtrlMsg;
sensor_msgs::JointState jointStatesMsg;
float jointStateVelocity[2];
float jointStatePosition[2];
float jointStateEffort[2];
char *jointStateNames[2] = {"left_wheel_joint", "right_wheel_joint"};

// ROS Node (Pub and Sub) Initialization
ros::NodeHandle nh;
ros::Publisher jointStates("joint_states_control", &jointStatesMsg);

// ROS Topic cmd_vel Callback
void speedCtrlCallback(const geometry_msgs::Vector3Stamped& msg) {
    velSetPointB = msg.vector.x;
    velSetPointA = msg.vector.y;
}

ros::Subscriber<geometry_msgs::Vector3Stamped> speedCtrlSub("angular_velocity_cmd", &speedCtrlCallback);

void setup() {
    nh.initNode();

    // Wait for ROS to connect to the Serial Port
    while (!nh.connected()) {
        nh.spinOnce();
    }

    nh.loginfo("Initalize Embedded Motor Controller");

    // Setup Joint State Message
    jointStatesMsg.name_length = 2;
    jointStatesMsg.position_length = 2;
    jointStatesMsg.velocity_length = 2;
    jointStatesMsg.effort_length = 2;
    jointStatesMsg.name = jointStateNames;
   
    nh.subscribe(speedCtrlSub);
    nh.advertise(jointStates);

    pidMotorA.setSampleTime(updateRate);
    pidMotorA.setOutputLimits(-175, 175);
    pidMotorA.setProportional(kp);
    pidMotorA.setIntegral(ki);
    pidMotorA.setFeedForward(kf);
    pidMotorA.enableController();
    motorA.setSpeed(0);
    motorA.forward();

    pidMotorB.setSampleTime(updateRate);
    pidMotorB.setOutputLimits(-175, 175);
    pidMotorB.setProportional(kp);
    pidMotorB.setIntegral(ki);
    pidMotorB.setFeedForward(kf);
    pidMotorB.enableController();
    motorB.setSpeed(0);
    motorB.forward();

    nh.loginfo("Initialization Complete");
}

void loop() {
    pidMotorA.compute();
    pidMotorB.compute();

    if(millis() - lastUpdate >= updateRate) {
      
        calculateMotorVelocity();

        // Set Motor A Direction 
        if(velSetPointA > 0) {
            motorA.forward();
        } else if(velSetPointA < 0) {
            motorA.backward();
        }

        // Set Motor A Speed
        if(velSetPointA == 0) {
            motorA.stop();
            pwmMotorA = 0;
            pidMotorA.disableController();
        } else {
            motorA.setSpeed(pwmMotorA);
            pidMotorA.enableController();
        }

        // Set Motor B Direction 
        if(velSetPointB > 0) {
            motorB.forward();
        } else if(velSetPointB < 0){
            motorB.backward();
        }

        // Set Motor B Speed
        if(velSetPointB == 0) {
            motorB.stop();
            pwmMotorB = 0;
            pidMotorB.disableController();
        } else {
            motorB.setSpeed(pwmMotorB);
            pidMotorB.enableController();
        }

        lastUpdate = millis();
    }

    if(millis() - lastPublish >= updateRate) {
        publishState();
        lastPublish = millis();
    }

    nh.spinOnce();
}

// Calculate the current velocity of the motors
void calculateMotorVelocity() {
    int motorATicks = motorAEnc.read();
    int motorBTicks = motorBEnc.read();

    long deltaMotorA = motorATicks - previousMotorATicks;
    long deltaMotorB = motorBTicks - previousMotorBTicks;

    // Calculate the change in angle of each motor
    double deltaAngleMotorA = ticksToAngle(deltaMotorA);
    double deltaAngleMotorB = ticksToAngle(deltaMotorB);

    previousMotorATicks = motorATicks;
    previousMotorBTicks = motorBTicks;

    // Calculate the time between updates
    double dt = (millis() - prev_update_time) / 1000.0;

    // Calculate current velocity of each motor
    velMotorAOutput = angleToAngularVelocity(deltaAngleMotorA, dt);
    velMotorBOutput = angleToAngularVelocity(deltaAngleMotorB, dt);

    // Calculate the current angle of each motor
    motorAAngle += deltaAngleMotorA;
    motorBAngle += deltaAngleMotorB;

    // Normalize angle to be between 0 and 2PI
    //motorAAngle = fmod(motorAAngle, 2 * PI);
    //motorBAngle = fmod(motorBAngle, 2 * PI);

    //if(motorAAngle < 0) {
        //motorAAngle += 2 * PI;
    //}

    //if(motorBAngle < 0) {
        //motorBAngle += 2 * PI;
    //}

    prev_update_time = millis();
}

void publishState() {
    jointStateVelocity[0] = velMotorBOutput;
    jointStateVelocity[1] = velMotorAOutput;
    jointStatePosition[0] = motorBAngle;
    jointStatePosition[1] = motorAAngle;
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
    return (double) ticks * (2.0 * PI/ ENC_COUNT_REV);
}

// Convert angle (in radians) to angular velocity (in rad/s)
double angleToAngularVelocity(double angle, double dt) {
    return angle / dt;
}
