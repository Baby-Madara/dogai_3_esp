#include <system.h>











// Task handles
TaskHandle_t Task1_Handle;
TaskHandle_t Task2_Handle;
TaskHandle_t Task3_Handle;
TaskHandle_t Task4_Handle;
TaskHandle_t Task5_Handle;
TaskHandle_t Task6_Handle;




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

MPU6050_Custom    myMPU(interpolDataOmegaX, interpolDataOmegaY, interpolDataOmegaZ, interpolDataAccX, interpolDataAccY, interpolDataAccZ);
// MPU6050_Custom myMPU(interpolDataOmegaX, interpolDataOmegaY, interpolDataOmegaZ, interpolDataAccX, interpolDataAccY, interpolDataAccZ);
// MPU6050_Custom myMPU(3, MPU6050_RANGE_250_DEG, MPU6050_RANGE_2_G,  MPU6050_BAND_44_HZ,  NULL,  NULL,  100,  0,  0.98,  3);

Quaternion OrientationQuaternion;
Quaternion OmegaQuaternion;
Quaternion alphaQuaternion;
Quaternion accelQuaternion;




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
                            0,   1,   2, 
                            3,   4,   5,
                            6,   7,   8,
                            9,  10,  11, 
                            12,  13
                        };

// int servoPinList[14] =  {
//                             2,   3,   4, 
//                             5,   6,   7, 
//                             8,   9,  10, 
//                             11, 12,  13,
//                             44, 45
//                         };

int potPinList[14] =    {
                            -1,   -1,   -1, 
                            -1,   -1,   -1, 
                            -1,   -1,   -1, 
                            -1,   -1,   -1,
                            -1,   -1
                        };

// int potPinList[14] =    {
//                              4,    2,   15,
//                              0,   32,   33, 
//                             25,   26,   27, 
//                             14,   12,  13,
//                             -1,   -1
//                         };
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
                            1 ,    -1,   
                        };

double jointOffsetList[14] =  {
                            96,     90,   -190, 
                            87,  -97.2,     76,
                            -81,    93,   -194, 
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



#define SDA_2    18 
#define SCL_2    19


TwoWire servoDriverWire = TwoWire(1); // Use I2C bus 0


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, servoDriverWire);


JointServo joints[14] =  {           
                JointServo(servoPinList[frS], pwm, potPinList[frS], 0, -70, 2220,  524, jointDirList[frS],  jointOffsetList[frS], jointLimitsLow[frS], jointLimitsHigh[frS], jointDefault[frS]),
                JointServo(servoPinList[frL], pwm, potPinList[frL], 0, -70, 1892,  260, jointDirList[frL],  jointOffsetList[frL], jointLimitsLow[frL], jointLimitsHigh[frL], jointDefault[frL]),
                JointServo(servoPinList[frF], pwm, potPinList[frF], 0,  90, 4072, 1784, jointDirList[frF],  jointOffsetList[frF], jointLimitsLow[frF], jointLimitsHigh[frF], jointDefault[frF]),

                JointServo(servoPinList[flS], pwm, potPinList[flS], 0,  70, 2312,  628, jointDirList[flS],  jointOffsetList[flS], jointLimitsLow[flS], jointLimitsHigh[flS], jointDefault[flS]),
                JointServo(servoPinList[flL], pwm, potPinList[flL], 0, -70, 2168, 2748, jointDirList[flL],  jointOffsetList[flL], jointLimitsLow[flL], jointLimitsHigh[flL], jointDefault[flL]),
                JointServo(servoPinList[flF], pwm, potPinList[flF], 0,  90, 4052, 2096, jointDirList[flF],  jointOffsetList[flF], jointLimitsLow[flF], jointLimitsHigh[flF], jointDefault[flF]),

                JointServo(servoPinList[brS], pwm, potPinList[brS], 0, -70, 2092, 3940, jointDirList[brS],  jointOffsetList[brS], jointLimitsLow[brS], jointLimitsHigh[brS], jointDefault[brS]),
                JointServo(servoPinList[brL], pwm, potPinList[brL], 0, -70, 1940, 3788, jointDirList[brL],  jointOffsetList[brL], jointLimitsLow[brL], jointLimitsHigh[brL], jointDefault[brL]),
                JointServo(servoPinList[brF], pwm, potPinList[brF], 0,  90, 3680, 2108, jointDirList[brF],  jointOffsetList[brF], jointLimitsLow[brF], jointLimitsHigh[brF], jointDefault[brF]),

                JointServo(servoPinList[blS], pwm, potPinList[blS], 0,  70, 2056, 3768, jointDirList[blS],  jointOffsetList[blS], jointLimitsLow[blS], jointLimitsHigh[blS], jointDefault[blS]),
                JointServo(servoPinList[blL], pwm, potPinList[blL], 0, -70, 2116, 3728, jointDirList[blL],  jointOffsetList[blL], jointLimitsLow[blL], jointLimitsHigh[blL], jointDefault[blL]),
                JointServo(servoPinList[blF], pwm, potPinList[blF], 0,  90,    0, 2216, jointDirList[blF],  jointOffsetList[blF], jointLimitsLow[blF], jointLimitsHigh[blF], jointDefault[blF]),

                JointServo(servoPinList[12], pwm,  potPinList[12],  0,  180,   0, 4065, jointDirList[12],   jointOffsetList[12], jointLimitsLow[12], jointLimitsHigh[12], jointDefault[12]),
                JointServo(servoPinList[13], pwm,  potPinList[13],  0,  180,   0, 4065, jointDirList[13],   jointOffsetList[13], jointLimitsLow[13], jointLimitsHigh[13], jointDefault[13]),
            };

palmSensor palms[4] =   {
                palmSensor(36, 0, 2.14*M_G, 0, 1780),       //2.14*M_G
                palmSensor(39, 0, 2.14*M_G, 0, 1780),       //2.14*M_G
                palmSensor(34, 0, 2.14*M_G, 0, 1780),       //2.14*M_G
                palmSensor(35, 0, 2.14*M_G, 0, 1780),       //2.14*M_G
            };

double cmd_angles[14]={0};


















void taskInitializers(){
    
    // xTaskCreatePinnedToCore(
    //      Task_function,
    //      "Name of task",
    //      Stack_size (bytes),
    //      Task input parameter  (NULL usually),
    //      Priority of the task  (0 low - 4 high...),
    //      &Task_handle,
    //      core_number
    // );
    
    
    // Create Task1 WiFi_comm
    xTaskCreatePinnedToCore(
        Task1_WiFi_comm,
        "WiFi_comm",
        16384,
        NULL,
        1,
        &Task1_Handle,
        0
    );
    
    
    
    // Create Task2 controller
    xTaskCreatePinnedToCore(
        Task2_controller,
        "controller",
        1024,
        NULL,
        2,
        &Task2_Handle,
        0
    );
    
    
    
    // Create Task3 gait_IK
    xTaskCreatePinnedToCore(
        Task3_gait_IK,
        "gait_IK",
        81920,
        NULL,
        8,
        &Task3_Handle,
        0
    );
    
    
    
    // Create Task4 observer
    xTaskCreatePinnedToCore(
        Task4_observer,
        "observer",
        16384,
        NULL,
        4,
        &Task4_Handle,
        0
    );
    
    
    
    // Create Task5 Serial
    xTaskCreatePinnedToCore(
        Task5_Serial,
        "Serial",
        16384,
        NULL,
        1,
        &Task5_Handle,
        0
    );
    
    
    
    // Create Task6 LiDAR
    xTaskCreatePinnedToCore(
        Task6_LiDAR,
        "LiDAR",
        16384,
        NULL,
        1,
        &Task6_Handle,
        0
    );
    
    
    
}









void Task1_WiFi_comm  (void *pvParameters)
{
    pinMode(LED_BUILTIN, OUTPUT);
    
    
    while(1)
    {
        digitalWrite(LED_BUILTIN, HIGH);      // Turn the LED on
        vTaskDelay(500); // Wait for 500ms
        digitalWrite(LED_BUILTIN, LOW);       // Turn the LED off
        vTaskDelay(500); // Wait for 500ms
    }
}









void Task2_controller (void *pvParameters)
{
    // 
    
    while(1)
    {
        
        
        vTaskDelay(1);
        
    }
    
    
}









void Task3_gait_IK    (void *pvParameters)
{
// init:
    servoDriverWire.begin(SDA_2, SCL_2);
    pwm.begin();
    pwm.setOscillatorFrequency(26600000);   // it is something between 23 to 27MHz (needed to be calibrated  --> calibration 26600000Hz)
    pwm.setPWMFreq(50);                     // of the output PWM signal from channels (for each servo)
    
    for (int servo_num = 0; servo_num < 14; servo_num++)
    {       
        joints[servo_num].writeAngleDirect(jointDefault[servo_num]);           
        // vTaskDelay(100);
    }
    
    
    
// loop:
    while(1)
    {
        
        
        
        cmd_angles[0]=-90;
        for (int servo_num = 0; servo_num < 14; servo_num++)
        {            
            joints[servo_num].writeAngleDirect(cmd_angles[servo_num]);       
            Serial.print(joints[0].angleCmd);       
            Serial.print("\t");
            Serial.println(joints[0].calibratedSignal);       
        }
        vTaskDelay(500);

        cmd_angles[0]=0;
        for (int servo_num = 0; servo_num < 14; servo_num++)
        {            
            joints[servo_num].writeAngleDirect(cmd_angles[servo_num]);       
            Serial.print(joints[0].angleCmd);       
            Serial.print("\t");
            Serial.println(joints[0].calibratedSignal);       
        }
        vTaskDelay(500);

        cmd_angles[0]=80;
        for (int servo_num = 0; servo_num < 14; servo_num++)
        {            
            joints[servo_num].writeAngleDirect(cmd_angles[servo_num]);       
            Serial.print(joints[0].angleCmd);       
            Serial.print("\t");
            Serial.println(joints[0].calibratedSignal);       
        }
        vTaskDelay(500);
    }
}









void Task4_observer   (void *pvParameters)
{
// init:
    if(myMPU.begin())               
    {           myMPU.calculate_IMU_error();        }
    else                            
    {           
        Serial.println("MPU m4 48ala");    
    }
    
// loop:
    while(1)
    {
        // myMPU.updateOrientation();
        OrientationQuaternion = myMPU.getOrientationQuaternion();
        
        
        vTaskDelay(1);
        
        
    }
}









void Task5_Serial     (void *pvParameters)
{
    int i=0;
    Serial.begin(115200);
    
    while(1)
    {
        // Print remaining stack size
        UBaseType_t remainingStack = uxTaskGetStackHighWaterMark(NULL);
        Serial.print("Remaining stack size: ");
        Serial.println(remainingStack);
        Serial.println("Hello "+String(i));
        
        i++;
        vTaskDelay(200); // Wait for 1 second
        Serial.println(String(OrientationQuaternion.w)+String(" , ")+String(OrientationQuaternion.x)+String(" , ")+String(OrientationQuaternion.y)+String(" , ")+String(OrientationQuaternion.z));
        // vTaskDelay(1000); // Wait for 1 second
    }
}









void Task6_LiDAR      (void *pvParameters)
{
    while (1)
    {
        
        vTaskDelay(1);
    }
    
}



