#ifndef SYSTEM_H
#define SYSTEM_H

#include <Arduino.h>


#include <utilities.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_PWMServoDriver.h>

#include <MPU_Custom.h>
#include <JointServo.h>
#include <palmSensor.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>




// sda1 21     -     scl1 22


#define SDA_2    18 
#define SCL_2    19





class IK{

};

class tf{

};









// Function prototypes
void Task1_WiFi_comm  (void *pvParameters);
void Task2_controller (void *pvParameters);
void Task3_gait_IK    (void *pvParameters);
void Task4_observer   (void *pvParameters);
void Task5_Serial     (void *pvParameters);
void Task6_LiDAR      (void *pvParameters);

void taskInitializers();








// Task handles
extern TaskHandle_t Task1_Handle;
extern TaskHandle_t Task2_Handle;
extern TaskHandle_t Task3_Handle;
extern TaskHandle_t Task4_Handle;
extern TaskHandle_t Task5_Handle;
extern TaskHandle_t Task6_Handle;



extern Data interpolDataOmegaX[3] ;
extern Data interpolDataOmegaY[3] ;
extern Data interpolDataOmegaZ[3] ;

//////////////////////////////////////////////////////////////////////////////////////////
// in m/sec^2
//accX:    -9.68587   ,     0.05806     ,    10.0116064      |    -9.806 ,  0,      9.806
//accY:    -9.87989   ,    -0.05928     ,     9.8234674      |    -9.806 ,  0,      9.806
//accZ:    -9.9856136 ,    -0.05580     ,     9.78571        |    -9.806 ,  0,      9.806
extern Data interpolDataAccX[3] ;
extern Data interpolDataAccY[3] ;
extern Data interpolDataAccZ[3] ;

extern MPU6050_Custom myMPU;
// MPU6050_Custom myMPU(interpolDataOmegaX, interpolDataOmegaY, interpolDataOmegaZ, interpolDataAccX, interpolDataAccY, interpolDataAccZ);
// MPU6050_Custom myMPU(3, MPU6050_RANGE_250_DEG, MPU6050_RANGE_2_G,  MPU6050_BAND_44_HZ,  NULL,  NULL,  100,  0,  0.98,  3);

extern Quaternion OrientationQuaternion;
extern Quaternion OmegaQuaternion;
extern Quaternion alphaQuaternion;
extern Quaternion accelQuaternion;





extern double palmStates[4];
extern double jointStates[14];
extern double jointCmd[14];
extern int servoPinList[14];
extern int potPinList[14];
extern double jointDirList[14];
extern double jointOffsetList[14];
extern double jointLimitsLow[14];
extern double jointLimitsHigh[14];
extern double jointDefault[14];






extern JointServo joints[14];
extern palmSensor palms[4];
extern double cmd_angles[14];







void wifiTask(void *parameter);
void randomImageTask(void *parameter);
void handleRoot();
void webSocketEventPhone(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void webSocketEventPython(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

















#endif  //SYSTEM_H