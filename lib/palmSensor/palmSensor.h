#ifndef PALMSENSOR_H
#define PALMSENSOR_H

#include <Arduino.h>
#include <utilities.h>






typedef enum{
    fr=0,
    fl,
    br,
    bl,
} palmIndex;









class palmSensor{
public:
    palmSensor(int _pin=A0, double _forceMin=0, double _forceMax=1023, int _signalMin=0, int _signalMax=1023);
    void init();
    double getForce();
    int pin;
    int forceMin;
    int forceMax;
    int signalMin;
    int signalMax;



// private:

};





#endif