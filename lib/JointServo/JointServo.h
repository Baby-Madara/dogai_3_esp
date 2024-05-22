#ifndef JOINTSERVO
#define JOINTSERVO

#include <Arduino.h>
// #include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <utilities.h>




typedef enum{
    frS=0,
    frL,
    frF,

    flS,
    flL,
    flF,

    brS,
    brL,
    brF,

    blS,
    blL,
    blF,
} jointIndex;



class JointServo{
public:
    JointServo(int _servoPin, Adafruit_PWMServoDriver &_pwm, int _potPin= -1, int _angleMin=0, int _angleMax=180, int _signalMin=0, int _signalMax =1023, double _dir = 1, double _offset=0, double _limitLow=0.0, double _limitHigh=0.0, double _defaultPose=0.0, double _maxSpeed = 4.7);
    void init();
    double readAngle();
    void writeAngleDirect(double angle);
    void writeAngleControlled(double angle);
    void servoAttach();
    void servoDetach();

// private:
    // Servo srv;
    Adafruit_PWMServoDriver &pwm;
    int servo_num;
    int potPin;
    int angleMin;
    int angleMax;
    int signalMin;
    int signalMax;
    double dir;
    double offset;
    double limitLow;
    double limitHigh;
    double defaultPose;
    
    double angleCmd;
    double maxSpeed;
    double calibratedSignal;
};











#endif