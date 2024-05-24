#ifndef OBJECTS_H
#define OBJECTS_H

#include <Arduino.h>
#include <ArduinoJson.h>

#include <palmSensor.h>
#include <MPU_Custom.h>
#include <JointServo.h>
#include <utilities.h>
// #include <SoftwareSerial.h>





#define  BUFFER_SIZE  256 
// #define  BUFFER_SIZE  1024 

#define MAX_RECEIVED_NUMBERS 18












/////////////////////////////////////////////      arduino-jetson interface     /////////////////////////////////////////////


class interface{
public:
    interface() : 
    myMPU(interpolDataOmegaX, interpolDataOmegaY, interpolDataOmegaZ, interpolDataAccX, interpolDataAccY, interpolDataAccZ),
    joints  {           
                JointServo(servoPinList[frS], potPinList[frS], 0, -70,  555,  131, jointDirList[frS],  jointOffsetList[frS], jointLimitsLow[frS], jointLimitsHigh[frS], jointDefault[frS]),
                JointServo(servoPinList[frL], potPinList[frL], 0, -70,  473,   65, jointDirList[frL],  jointOffsetList[frL], jointLimitsLow[frL], jointLimitsHigh[frL], jointDefault[frL]),
                JointServo(servoPinList[frF], potPinList[frF], 0,  90, 1018,  446, jointDirList[frF],  jointOffsetList[frF], jointLimitsLow[frF], jointLimitsHigh[frF], jointDefault[frF]),

                JointServo(servoPinList[flS], potPinList[flS], 0,  70,  578,  157, jointDirList[flS],  jointOffsetList[flS], jointLimitsLow[flS], jointLimitsHigh[flS], jointDefault[flS]),
                JointServo(servoPinList[flL], potPinList[flL], 0, -70,  542,  687, jointDirList[flL],  jointOffsetList[flL], jointLimitsLow[flL], jointLimitsHigh[flL], jointDefault[flL]),
                JointServo(servoPinList[flF], potPinList[flF], 0,  90, 1013,  524, jointDirList[flF],  jointOffsetList[flF], jointLimitsLow[flF], jointLimitsHigh[flF], jointDefault[flF]),

                JointServo(servoPinList[brS], potPinList[brS], 0, -70,  523,  985, jointDirList[brS],  jointOffsetList[brS], jointLimitsLow[brS], jointLimitsHigh[brS], jointDefault[brS]),
                JointServo(servoPinList[brL], potPinList[brL], 0, -70,  485,  947, jointDirList[brL],  jointOffsetList[brL], jointLimitsLow[brL], jointLimitsHigh[brL], jointDefault[brL]),
                JointServo(servoPinList[brF], potPinList[brF], 0,  90,  920,  527, jointDirList[brF],  jointOffsetList[brF], jointLimitsLow[brF], jointLimitsHigh[brF], jointDefault[brF]),

                JointServo(servoPinList[blS], potPinList[blS], 0,  70,  514,  942, jointDirList[blS],  jointOffsetList[blS], jointLimitsLow[blS], jointLimitsHigh[blS], jointDefault[blS]),
                JointServo(servoPinList[blL], potPinList[blL], 0, -70,  529,  932, jointDirList[blL],  jointOffsetList[blL], jointLimitsLow[blL], jointLimitsHigh[blL], jointDefault[blL]),
                JointServo(servoPinList[blF], potPinList[blF], 0,  90,    0,  554, jointDirList[blF],  jointOffsetList[blF], jointLimitsLow[blF], jointLimitsHigh[blF], jointDefault[blF]),

                JointServo(servoPinList[12],  potPinList[12],  0,  180,   0, 1023, jointDirList[12],   jointOffsetList[12], jointLimitsLow[12], jointLimitsHigh[12], jointDefault[12]),
                JointServo(servoPinList[13],  potPinList[13],  0,  180,   0, 1023, jointDirList[13],   jointOffsetList[13], jointLimitsLow[13], jointLimitsHigh[13], jointDefault[13]),
            },
    palms   {
                palmSensor(A12, 0, 2.14*M_G, 0, 445),       //2.14*M_G
                palmSensor(A13, 0, 2.14*M_G, 0, 445),       //2.14*M_G
                palmSensor(A14, 0, 2.14*M_G, 0, 445),       //2.14*M_G
                palmSensor(A15, 0, 2.14*M_G, 0, 445),       //2.14*M_G
            },
    mySerial(15, 14)  //rx tx

    
    {}

    void Init();
    void updateSensors();

    void jointCmdExec();
    void StatesSender();
    bool dataReceiver();
    
// private:

    SoftwareSerial mySerial; //rx tx
    float data[MAX_RECEIVED_NUMBERS]; // Array to store the received data

    palmSensor palms[4];

    char buffer[BUFFER_SIZE];
    // double SEND_PERIOD = 50;
    double SEND_PERIOD = 0;
    double senderPrevTime = 0;


    /////////////////////////////////////////////      mpu     /////////////////////////////////////////////
    // in deg/sec:
    //omegaX:  98.97159   ,    -0.0986408747 ,   -98.87974       |      100 ,  0,   -100 
    //omegaY: 100.24887   ,     1.1573488335 ,   -97.85251       |      100 ,  0,   -100 
    //omegaZ:  97.01528   ,    -1.8363988338 ,  -100.44398       |      100 ,  0,   -100 
    Data interpolDataOmegaX[3] = 	{
                                        { -98.88974      * M_PI/180.0, -100 * M_PI/180.0 },
                                        {  -0.0986408747 * M_PI/180.0,    0 * M_PI/180.0 },
                                        {  98.97759      * M_PI/180.0,  100 * M_PI/180.0 }
                                    };
    Data interpolDataOmegaY[3] = 	{
                                        { -97.85251      * M_PI/180.0, -100 * M_PI/180.0 },
                                        {   1.1573483350 * M_PI/180.0,    0 * M_PI/180.0 },
                                        { 100.26587      * M_PI/180.0,  100 * M_PI/180.0 }
                                    };
    Data interpolDataOmegaZ[3] = 	{
                                        {-100.44398      * M_PI/180.0, -100 * M_PI/180.0},
                                        {  -1.8363988338 * M_PI/180.0,    0 * M_PI/180.0},
                                        {  97.02428      * M_PI/180.0,  100 * M_PI/180.0}
                                    };

    //////////////////////////////////////////////////////////////////////////////////////////
    // in m/sec^2
    //accX:    -9.68587   ,     0.05806     ,    10.0116064      |    -9.806 ,  0,      9.806
    //accY:    -9.87989   ,    -0.05928     ,     9.8234674      |    -9.806 ,  0,      9.806
    //accZ:    -9.9856136 ,    -0.05580     ,     9.78571        |    -9.806 ,  0,      9.806
    Data interpolDataAccX[3] = 	{
                                    {    -9.68587    , -M_G  },
                                    {     0.05806    ,    0  },
                                    {    10.0116064  ,  M_G  }
                                };
    Data interpolDataAccY[3] = 	{
                                    {    -9.87989    , -M_G  },
                                    {    -0.05928    ,    0  },
                                    {     9.8234674  ,  M_G  }
                                };
    Data interpolDataAccZ[3] = 	{
                                    {    -9.9856136  , -M_G  },
                                    {    -0.05580    ,    0  },
                                    {     9.79634214 ,  M_G  }
                                };

    MPU6050_Custom myMPU;
    // MPU6050_Custom myMPU(interpolDataOmegaX, interpolDataOmegaY, interpolDataOmegaZ, interpolDataAccX, interpolDataAccY, interpolDataAccZ);
    // MPU6050_Custom myMPU(3, MPU6050_RANGE_250_DEG, MPU6050_RANGE_2_G,  MPU6050_BAND_44_HZ,  NULL,  NULL,  100,  0,  0.98,  3);

    Quaternion OrientationQuaternion;
    Quaternion OmegaQuaternion;
    Quaternion alphaQuaternion;
    Quaternion accelQuaternion;







    /////////////////////////////////////////////      servos     /////////////////////////////////////////////



    double palmStates[4]= {0, 0, 0, 0};

    double jointStates[14] =   {
                                            0.001,   0.002,   0.003, 
                                            0.004,   0.005,   0.006, 
                                            0.007,   0.008,   0.009, 
                                            0.010,   0.011,   0.012,
                                            0,       0
                                        };

    double jointCmd[14]    =   { 
                                            0,   0,   0, 
                                            0,   0,   0, 
                                            0,   0,   0, 
                                            0,   0,   0,
                                            0,   0
                                        };

    int servoPinList[14] =  {
                                2,   3,   4, 
                                5,   6,   7, 
                                8,   9,  10, 
                                11, 12,  13,
                                44, 45
                            };

    int potPinList[14] =    {
                                -1,   -1,   -1, 
                                -1,   -1,   -1, 
                                -1,   -1,   -1, 
                                -1,   -1,   -1,
                                -1,   -1
                            };
    // int potPinList[14] =    {
    //                             A0,   A1,   A2, 
    //                             A3,   A4,   A5, 
    //                             A6,   A7,   A8, 
    //                             A9,   A10,  A11,
    //                             -1,   -1
    //                         };


#ifdef OLD_DOG

double jointOffsetList[14]     = {90,  80,  55,     79, 93, 119,     73, 168,  63,      65, 55, 110,      98,  -85};
double jointDirList[14]        = {-1,   1,   1,     -1, -1,  -1,      1,   1,   1,       1, -1,  -1,       1,   -1};

#else

    double jointDirList[14] =  {
                                1 ,     1,  -2/3.0, 
                                1 ,    -1,   2/3.0, 
                                -1,     1,  -2/3.0,
                                -1,    -1,   2/3.0,
                                1 ,     -1,   
                            };

    double jointOffsetList[14] =  {
                                96,     90,   -190, 
                                87,  -97.2,     76,
                                -81,    93,   -214, 
                             // -81,    93,   -194, 
                                -89, -94.9,     85,
                                98,    -85                 //-97 --> -7
                            };

#endif
    
    double jointLimitsLow[14] = {
                                -90,  -70,   0, 
                                -90,  -70,   0, 
                                -90,  -70,   0, 
                                -90,  -70,   0,
                                -90,  -90
                            };
    
    double jointLimitsHigh[14] = {
                                90,    85,   155, 
                                90,    85,   155, 
                                90,    85,   155, 
                                90,    85,   155,
                                90,    90
                            };
    
    double jointDefault[14] = {
                                0,    -30,   60, 
                                0,    -30,   60, 
                                0,    -30,   60, 
                                0,    -30,   60,
                                0,     0
                            };

    JointServo joints[14];
    




    
};




#endif