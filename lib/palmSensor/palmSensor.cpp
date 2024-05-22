#include <palmSensor.h>




palmSensor::palmSensor(int _pin, double _forceMin, double _forceMax, int _signalMin, int _signalMax):
    pin(_pin),
    forceMin(_forceMin),
    forceMax(_forceMax),
    signalMin(_signalMin),
    signalMax(_signalMax)
{}

void palmSensor::init(){
    pinMode(pin, INPUT);
}

double palmSensor::getForce(){
    return MAP(analogRead(pin),     signalMin,    signalMax,    forceMin,    forceMax  );
}







