#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>


// for the old dog, else, must be commented
// #define OLD_DOG

// #define PRINT_ANGLES

#define MAP(_val, _oldL, _oldH, _newL, _newH)	( ((double)_newH - _newL)*((double)_val - _oldL)/( (double)_oldH - _oldL) + _newL )
#define M_G 9.806




// To represent a data point corresponding to x and y = f(x)
struct Data {
    double x, y;
};


double interpolate_custom(Data f[], int dataSize, double xi);
bool isInRange(double in, double range,  double mean=0);





class Convolver {
public:
    Convolver(unsigned int size, double &externalVar);
    ~Convolver();
    void addSample(int sample);

private:
    int* buffer;
    unsigned int bufferSize;
    unsigned int index;
    long sum;
    double& externalVariable;
};






#endif