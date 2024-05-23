#ifndef CUSTOM_MPU6050_H
#define CUSTOM_MPU6050_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <utilities.h>




void matrixMultiplier(double m1[4][4], double m2[4][4], double mResult[4][4]);
void matrix_transpose(int m, int n, double matrix[4][4], double transpose[4][4]);


class Quaternion_ {
public:
    double w, x, y, z;

    Quaternion_(double w = 1, double x = 0, double y = 0, double z = 0);

    void normalize();
    static Quaternion_ fromEuler(double roll, double pitch, double yaw);
    static Quaternion_ slerp(Quaternion_ &q1, Quaternion_ &q2, double alpha);
    static Quaternion_ quat_calc(double dx, double dy, double dz);
    void toEuler(double &roll, double &pitch, double &yaw);
    void toMatrix(double matrix[4][4], double tx = 0, double ty = 0, double tz = 0);
    Quaternion_ conjugate();
    Quaternion_ operator%(const Quaternion_ &q) const;
    Quaternion_ operator+(const Quaternion_ &q) const;
    Quaternion_ operator*(double scalar) const;
};


class MPU6050_Custom {
public:
    Adafruit_MPU6050 mpu;
    sensors_event_t a, g, temp;
	
    double alpha;
    double tolerance;
    Quaternion_ quatD, quatA, quatC, quatG, accelQuat, velocitioyQuat, positionQuat;
	
    
	//////////////////////////////////////////////////////////////////////////////////////////
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



    MPU6050_Custom(
        int listsSize = 3, 
        mpu6050_gyro_range_t gyro_range = MPU6050_RANGE_250_DEG, 
        mpu6050_accel_range_t accel_range = MPU6050_RANGE_2_G, 
        mpu6050_bandwidth_t bandwidth_range = MPU6050_BAND_44_HZ, 
        int sclPin = NULL, 
        int sdaPin = NULL, 
        int MPU_ERR_ITERATIONS = 100, 
        int MPU_PERIODIC = 0, 
        double alpha = 0.98, 
        double tolerance = 3
    );
    MPU6050_Custom(
        Data *_interpolDataOmegaX, 
        Data *_interpolDataOmegaY, 
        Data *_interpolDataOmegaZ, 
        Data *_interpolDataAccX, 
        Data *_interpolDataAccY, 
        Data *_interpolDataAccZ, 
        int listsSize = 3, 
        mpu6050_gyro_range_t gyro_range = MPU6050_RANGE_250_DEG, 
        mpu6050_accel_range_t accel_range = MPU6050_RANGE_2_G, 
        mpu6050_bandwidth_t bandwidth_range = MPU6050_BAND_44_HZ, 
        int sclPin = NULL, 
        int sdaPin = NULL, 
        int MPU_ERR_ITERATIONS = 100, 
        int MPU_PERIODIC = 0, 
        double alpha = 0.98, 
        double tolerance = 3
    );

    bool begin();
    void updateOrientation();
    void toEuler(double &r, double &p, double &y);
    // Quaternion_ gyroToQuaternion_(double gx, double gy, double gz);
    Quaternion_ anglesToQuaternion(double roll, double pitch, double yaw);
    void getOrientationAxisGlobal(double &x, double &y, double &z);
    Quaternion_ getOrientationQuaternion();
    void getAngularVelocityGlobal(double &gx, double &gy, double &gz);
    void getAngularAccelerationGlobal(double &ax, double &ay, double &az);
    void getLinearAccelerationGlobal(double &ax, double &ay, double &az);
    void getLinearAccelerationQuaternionGlobal(Quaternion_ &resultQuat);
    void getLinearAccelerationLocal(double &ax, double &ay, double &az);
    void setOrientationQuaternion(Quaternion_ _quatC);
    void calculate_IMU_error();

// private:
    mpu6050_gyro_range_t gyro_range;
    mpu6050_accel_range_t accel_range;
    mpu6050_bandwidth_t bandwidth_range;
    int MPU_ERR_ITERATIONS;
    int MPU_PERIODIC;
    double currentTime;
    double prevTime;
    double dt;
	double _alpha;
    int listsSize=3;
    
    double accErrRange=0;
    double gyroErrRange=0;
    
    
    double accErrX=0;           double accErrY=0;               double accErrZ=0;
    double omegaErrX=0;         double omegaErrY=0;             double omegaErrZ=0;
    double accErrXOffset=0;     double accErrYOffset=0;         double accErrZOffset=0;
    double omegaErrXOffset=0;   double omegaErrYOffset=0;       double omegaErrZOffset=0;
    
    double accX=0;              double accY=0;                  double accZ=0;
    double accXprev=0;          double accYprev=0;              double accZprev=0;
    double omegaX=0;            double omegaY=0;                double omegaZ=0;
    double omega2X=0;           double omega2Y=0;               double omega2Z=0;
    double omega1X=0;           double omega1Y=0;               double omega1Z=0;
    double omegaXd=0;           double omegaYd=0;               double omegaZd=0;
    double thetaXdelta;         double thetaYdelta;             double thetaZdelta;
    double rollg=0;             double pitchg=0;                double yawg=0;
    double rolla=0;             double pitcha=0;                double yawa=0;


    double TF      [4][4]={
                        {1, 0, 0, 0},
                        {0, 1, 0, 0},
                        {0, 0, 1, 0},
                        {0, 0, 0, 1},
                    };

    double TF_trans[4][4]={
                        {1, 0, 0, 0},
                        {0, 1, 0, 0},
                        {0, 0, 1, 0},
                        {0, 0, 0, 1},
                    };

    Convolver convAx;
    Convolver convAy;
    Convolver convAz;
};





#endif // CUSTOM_MPU6050_H
