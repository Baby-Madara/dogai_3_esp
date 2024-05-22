#include <JointServo.h>
#include <Adafruit_PWMServoDriver.h>
#include <utilities.h>



// extern Adafruit_PWMServoDriver pwm;


JointServo::JointServo(int _servoPin, Adafruit_PWMServoDriver &_pwm, int _potPin, int _angleMin, int _angleMax, int _signalMin, int _signalMax, double _dir, double _offset, double _limitLow, double _limitHigh, double _defaultPose, double _maxSpeed):
    servo_num   ( _servoPin),
    pwm         (_pwm),
    potPin      ( _potPin),
    angleMin    ( _angleMin),
    angleMax    ( _angleMax),
    signalMin   ( _signalMin),
    signalMax   ( _signalMax),
    dir         ( _dir),
    offset      ( _offset),
    limitLow    (_limitLow),
    limitHigh   (_limitHigh),
    defaultPose (_defaultPose),
    angleCmd    (_defaultPose),
    maxSpeed    (_maxSpeed)
{}

void JointServo::init()
{
    if(servo_num != -1){
        
        writeAngleDirect(defaultPose);
        // Serial.println(String(servoPin) + String("\t") + String(potPin) + String("\t") + String(angleMax) + String("\t") + String(signalMax) + String("\t") + String(dir) + String("\t") + String(offset));
    }
    if(potPin != -1){
        pinMode(potPin, INPUT);
    }
}

double JointServo::readAngle()
{
    if(potPin != -1){
        return MAP(analogRead(potPin), signalMin, signalMax, angleMin, angleMax);
    }
    else{
        return angleCmd;
    }
}

// void JointServo::servoAttach(){
//     srv.attach(servoPin, 500, 2500);
// }

// void JointServo::servoDetach(){
//     srv.detach();
// }


// in degrees
void JointServo::writeAngleDirect(double angle)
{
    angleCmd = angle;
    if     (angle<limitLow)     {    angle=limitLow;        }
    else if(angle>limitHigh)    {    angle=limitHigh;       }

#ifdef OLD_DOG
    double calibratedSignal = ((angle*dir) + offset);
#else
    calibratedSignal = ((angle + offset)*dir);
#endif

    if(servo_num != -1)  {                    pwm.writeMicroseconds(servo_num, MAP(calibratedSignal, 0.0, 180.0, 500.0, 2500.0));       }
    
}







// in degrees
void JointServo::writeAngleControlled(double angle)
{
    // if no potenntiometerfeedback (potPin == -1)
    if(potPin == -1)
    {
        writeAngleDirect(angle);
        // Serial.println(String(servoPin) + String("\t")+     String(angle) + String("\t")+     String(((angle + offset)*dir)) + String("\t") + String(" sent to writeAngleControlled"));
        // return;
    }
    else
    {
        writeAngleDirect(angle);
        // do control
        // srv.write((int)( (angle*(180.0/PI)*dir) + offset ));
    }
}












