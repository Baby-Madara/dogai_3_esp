#include <utilities.h>


double interpolate_custom(Data f[], int dataSize, double xi)
{

    if(xi < f[0].x)
    {        return MAP(xi, f[0].x, f[1].x,  f[0].y, f[1].y);    }
    else if(xi > f[dataSize-1].x)
    {        return MAP(xi, f[dataSize-2].x, f[dataSize-1].x,  f[dataSize-2].y, f[dataSize-1].y);    }

    for(int i=0; i<dataSize-1; i++)
    {
        if(xi>=f[i].x && xi<=f[i+1].x)
        {   return MAP(xi, f[i].x, f[i+1].x,  f[i].y, f[i+1].y);   }

    }

}
bool isInRange(double in, double range,  double mean)
{
    if ((in< (mean+abs(range)))  &&  (in> (mean-abs(range))))
    {return true;}
    else
    {return false;}
}

















////////////////////////////////////////////////////   Convolver   ////////////////////////////////////////////////////

// Constructor: initializes the buffer, variables, and binds the reference
Convolver::Convolver(unsigned int size, double &externalVar) : bufferSize(size), externalVariable(externalVar)
{
    buffer = new int[bufferSize];
    for (unsigned int i = 0; i < bufferSize; i++)
    {
        buffer[i] = 0;
    }
    index = 0;
    sum = 0;
}

// Destructor: frees the dynamically allocated buffer
Convolver::~Convolver()
{
    delete[] buffer;
}

// Adds a new sample and updates the external variable with the moving average
void Convolver::addSample(int sample)
{
    // Subtract the oldest sample from sum and replace it with the new sample
    sum -= buffer[index];
    buffer[index] = sample;
    sum += sample;

    // Move to the next buffer position, wrapping around if necessary
    index = (index + 1) % bufferSize;

    // Update the external variable with the average
    externalVariable = (double)sum / bufferSize;
}






