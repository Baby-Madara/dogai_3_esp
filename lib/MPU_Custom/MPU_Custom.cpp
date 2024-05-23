#include <MPU_Custom.h>
#include <utilities.h>

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>


// α = τ/(τ + Δt)

//////////////////////////////////////////////////// Matrices Funcs////////////////////////////////////////////////////


void matrixMultiplier(double  m1[4][4], double  m2[4][4], double mResult[4][4])
{
	int i, j, k;

	// Initialize all elements of mResult to zero
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			mResult[i][j] = 0;
		}
	}

	// Multiply matrices m1 and m2
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			for (k = 0; k < 4; k++)
			{
				mResult[i][j] += m1[i][k] * m2[k][j];
			}
		}
	}
}
void matrix_transpose(int m, int n, double matrix[4][4], double transpose[4][4])
{
	// Transpose logic
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			transpose[j][i] = matrix[i][j];
		}
	}
}





////////////////////////////////////////////////////  Quaternion   ////////////////////////////////////////////////////


// Constructor to initialize the quaternion
Quaternion_::Quaternion_(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

// Normalize the quaternion_
void Quaternion_::normalize() {
    double norm = sqrt(w * w + x * x + y * y + z * z);
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
}

// Function to convert Euler angles to Quaternion_
Quaternion_ Quaternion_::fromEuler(double roll, double pitch, double yaw) {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    return Quaternion_(
        cr * cp * cy + sr * sp * sy, // w
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy  // z
    );
}

// Static method for Spherical Linear Interpolation (SLERP)
Quaternion_ Quaternion_::slerp(Quaternion_ &q1, Quaternion_ &q2, double alpha=0.98) {
    // Normalize the quaternion_s
    q1.normalize();
    q2.normalize();

    // Compute the cosine of the angle between the quaternion_s
    double dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

    // If the dot product is negative, the quaternion_s have opposite handed-ness
    // and slerp won't take the shorter path. Fix by reversing one quaternion_.
    if (dot < 0.0) {
        q2.w = -q2.w;
        q2.x = -q2.x;
        q2.y = -q2.y;
        q2.z = -q2.z;
        dot = -dot;
    }

    // const double DOT_THRESHOLD = 0.9995;
    const double DOT_THRESHOLD = 0.9995;
    if (dot > DOT_THRESHOLD) {
        // If the inputs are too close for comfort, linearly interpolate
        Quaternion_ result = q1 * (1 - alpha) + q2 * alpha;
        result.normalize();
        return result;
    }

    // Calculate the angle and its sine
    double theta_0 = acos(dot);
    double theta = theta_0 * alpha;
    double sin_theta = sin(theta);
    double sin_theta_0 = sin(theta_0);

    // Compute the two quaternion_s
    double s0 = cos(theta) - dot * sin_theta / sin_theta_0;
    double s1 = sin_theta / sin_theta_0;

    return Quaternion_(
        (s0 * q1.w) + (s1 * q2.w),
        (s0 * q1.x) + (s1 * q2.x),
        (s0 * q1.y) + (s1 * q2.y),
        (s0 * q1.z) + (s1 * q2.z)
    );
}

Quaternion_ Quaternion_::quat_calc(double dx, double dy, double dz)
{
    double angle;
    double kx;
    double ky;
    double kz;
    
    angle = sqrt( (dx*dx)  +  (dy*dy)  +  (dz*dz) );
    if(angle != 0.0){
        kx = dx / angle;
        ky = dy / angle;
        kz = dz / angle;
    }
    
    else{
        kx = 0;
        ky = 0;
        kz = 0;
    }
    
    return Quaternion_(
        cos(angle/2),
        kx * sin(angle/2),
        ky * sin(angle/2),
        kz * sin(angle/2)
    );
}

// Function to convert Quaternion_ to Euler angles
void Quaternion_::toEuler(double &roll, double &pitch, double &yaw) {
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (abs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

// Function to convert Quaternion_ to a 4x4 transformation matrix
void Quaternion_::toMatrix(double matrix[4][4], double tx, double ty, double tz) {
    matrix[0][0] = 1 - 2 * y * y - 2 * z * z;
    matrix[0][1] = 2 * x * y - 2 * w * z;
    matrix[0][2] = 2 * x * z + 2 * w * y;
    matrix[0][3] = tx;

    matrix[1][0] = 2 * x * y + 2 * w * z;
    matrix[1][1] = 1 - 2 * x * x - 2 * z * z;
    matrix[1][2] = 2 * y * z - 2 * w * x;
    matrix[1][3] = ty;

    matrix[2][0] = 2 * x * z - 2 * w * y;
    matrix[2][1] = 2 * y * z + 2 * w * x;
    matrix[2][2] = 1 - 2 * x * x - 2 * y * y;
    matrix[2][3] = tz;

    matrix[3][0] = 0;
    matrix[3][1] = 0;
    matrix[3][2] = 0;
    matrix[3][3] = 1;
}

// Add a method to get the conjugate of a quaternion_
Quaternion_ Quaternion_::conjugate() {
    return Quaternion_(w, -x, -y, -z);
}

// Operator overloading for quaternion_ multiplication
Quaternion_ Quaternion_::operator%(const Quaternion_ &q) const {
    return Quaternion_(
        w*q.w - x*q.x - y*q.y - z*q.z,
        w*q.x + x*q.w + y*q.z - z*q.y,
        w*q.y - x*q.z + y*q.w + z*q.x,
        w*q.z + x*q.y - y*q.x + z*q.w
    );
}

// Operator overloading for addition
Quaternion_ Quaternion_::operator+(const Quaternion_ &q) const {
    return Quaternion_(w + q.w, x + q.x, y + q.y, z + q.z);
}

// Operator overloading for scalar multiplication
Quaternion_ Quaternion_::operator*(double scalar) const {
    return Quaternion_(w*scalar, x*scalar, y*scalar, z*scalar);
}










////////////////////////////////////////////////////  Custom MPU   ////////////////////////////////////////////////////


MPU6050_Custom::MPU6050_Custom(
    int listsSize, 
    mpu6050_gyro_range_t gyro_range ,
    mpu6050_accel_range_t accel_range ,
    mpu6050_bandwidth_t bandwidth_range ,
    int sclPin,
    int sdaPin,
    int MPU_ERR_ITERATIONS,
    int MPU_PERIODIC,
    double alpha,
    double tolerance
) :listsSize(listsSize), gyro_range(gyro_range), accel_range(accel_range), bandwidth_range(bandwidth_range), MPU_ERR_ITERATIONS(MPU_ERR_ITERATIONS), MPU_PERIODIC(MPU_PERIODIC), alpha(alpha), tolerance(tolerance), convAx(5, accX), convAy(5, accY), convAz(5, accZ)
{
    _alpha =alpha;

    if((!sdaPin) || !(sclPin))
    {
        // try
        // {
        //     gpio_pad_select_gpio(I2C_SCL_PIN);
        //     gpio_set_direction(GPIO_NUM_14, GPIO_MODE_INPUT); //where GPIO_NUM_14 is I2C_SCL_PIN
        //     gpio_pad_select_gpio(I2C_SDA_PIN);
        //     gpio_set_direction(GPIO_NUM_15, GPIO_MODE_INPUT); //where GPIO_NUM_15 is I2C_SDA_PIN
        //     Wire1.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000); // sda, scl
        // }
        // catch(int x)
        // {;}
    }

}

MPU6050_Custom::MPU6050_Custom(
    Data *_interpolDataOmegaX, 
    Data *_interpolDataOmegaY, 
    Data *_interpolDataOmegaZ, 
    Data *_interpolDataAccX, 
    Data *_interpolDataAccY, 
    Data *_interpolDataAccZ,
    int listsSize, 
    mpu6050_gyro_range_t gyro_range,
    mpu6050_accel_range_t accel_range,
    mpu6050_bandwidth_t bandwidth_range,
    int sclPin, 
    int sdaPin, 
    int MPU_ERR_ITERATIONS,
    int MPU_PERIODIC,
    double alpha,
    double tolerance
) : listsSize(listsSize), gyro_range(gyro_range), accel_range(accel_range), bandwidth_range(bandwidth_range), MPU_ERR_ITERATIONS(MPU_ERR_ITERATIONS), MPU_PERIODIC(MPU_PERIODIC), alpha(alpha), tolerance(tolerance), convAx(5, accX), convAy(5, accY), convAz(5, accZ)
{
    _alpha =alpha;
    // Copy array data
    memcpy(interpolDataOmegaX, _interpolDataOmegaX, sizeof(interpolDataOmegaX));
    memcpy(interpolDataOmegaY, _interpolDataOmegaY, sizeof(interpolDataOmegaY));
    memcpy(interpolDataOmegaZ, _interpolDataOmegaZ, sizeof(interpolDataOmegaZ));
    memcpy(interpolDataAccX, _interpolDataAccX, sizeof(interpolDataAccX));
    memcpy(interpolDataAccY, _interpolDataAccY, sizeof(interpolDataAccY));
    memcpy(interpolDataAccZ, _interpolDataAccZ, sizeof(interpolDataAccZ));

}

// Initialize MPU6050
bool MPU6050_Custom::begin() {
    if (!mpu.begin()) {
        // Serial2.println("no MPU custom");
        return false;
    }
    mpu.setGyroRange(gyro_range);
    mpu.setAccelerometerRange(accel_range);
    mpu.setFilterBandwidth(bandwidth_range);

    calculate_IMU_error();
    
    accelQuat      = Quaternion_(0, 0, 0, 0);
    velocitioyQuat = Quaternion_(0, 0, 0, 0);
    positionQuat   = Quaternion_(0, 0, 0, 0);
    
    return true;
}

bool firstTime= true;
// Read sensor data and update orientation  (MUST BE IN THE FIRST OF 'LOOP()')
void MPU6050_Custom::updateOrientation() {
    currentTime = micros();
    if(currentTime - prevTime > MPU_PERIODIC)
    {
        dt = currentTime - prevTime;

        mpu.getEvent(&a, &g, &temp);

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////        GET clean Data        ////////////////////////////////////////
        // Get new sensor events with the readings 
        
        
        // get & clean omega (rad/s), acc (mm/s^2):
        accX =    0.8*( interpolate_custom(interpolDataAccX,   listsSize, a.acceleration.x) - accErrXOffset) + (1-0.8)*accX;
        accY =    0.8*( interpolate_custom(interpolDataAccY,   listsSize, a.acceleration.y) - accErrYOffset) + (1-0.8)*accY;
        accZ =    0.8*( interpolate_custom(interpolDataAccZ,   listsSize, a.acceleration.z) - accErrZOffset) + (1-0.8)*accZ;
        
        if(firstTime==true){
            accXprev =accX;
            accYprev =accY;
            accZprev =accZ;
            firstTime= false;
        }

        
        // omega2X = omegaX = (interpolate_custom(interpolDataOmegaX, listsSize, g.gyro.x)) -(0.0016*(interpolate_custom(interpolDataOmegaX, listsSize, g.gyro.x)         - omegaErrXOffset) + (1-0.0016)*omegaX);
        // omega2Y = omegaY = (interpolate_custom(interpolDataOmegaY, listsSize, g.gyro.y)) -(0.0016*(interpolate_custom(interpolDataOmegaY, listsSize, g.gyro.y)         - omegaErrYOffset) + (1-0.0016)*omegaY);
        // omega2Z = omegaZ = (interpolate_custom(interpolDataOmegaZ, listsSize, g.gyro.z)) -(0.0016*(interpolate_custom(interpolDataOmegaZ, listsSize, g.gyro.z)         - omegaErrZOffset) + (1-0.0016)*omegaZ);
        
        omega2X = omegaX = 0.9998*(interpolate_custom(interpolDataOmegaX, listsSize, g.gyro.x)         - omegaErrXOffset) + (1-0.9998)*omegaX;
        omega2Y = omegaY = 0.9998*(interpolate_custom(interpolDataOmegaY, listsSize, g.gyro.y)         - omegaErrYOffset) + (1-0.9998)*omegaY;
        omega2Z = omegaZ = 0.9998*(interpolate_custom(interpolDataOmegaZ, listsSize, g.gyro.z)         - omegaErrZOffset) + (1-0.9998)*omegaZ;
        if( isInRange(omegaX, gyroErrRange, 0)){    omega2X = omegaX=0;    }
        if( isInRange(omegaY, gyroErrRange, 0)){    omega2Y = omegaY=0;    }
        if( isInRange(omegaZ, gyroErrRange, 0)){    omega2Z = omegaZ=0;    }
        

        // smoothing using convolving
        // convAx.addSample(accX);
        // convAy.addSample(accY);
        // convAz.addSample(accZ);
        
        omegaXd = omega2X - omega1X;
        omegaYd = omega2Y - omega1Y;
        omegaZd = omega2Z - omega1Z;
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////         delta theta          ////////////////////////////////////////
        
        // find delta thetas, necessary to find TF after each `loop()` iteration
        thetaXdelta = (omega2X + omega1X) * (dt) /(2.0 * 1000000.0);
        thetaYdelta = (omega2Y + omega1Y) * (dt) /(2.0 * 1000000.0);
        thetaZdelta = (omega2Z + omega1Z) * (dt) /(2.0 * 1000000.0);
        
        omega1X = omega2X;
        omega1Y = omega2Y;
        omega1Z = omega2Z;
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////         R_ob (gyro)          ////////////////////////////////////////
        quatD = Quaternion_::quat_calc(thetaXdelta, thetaYdelta, thetaZdelta);    				
        quatG = quatG%quatD;
        quatG.toEuler(rollg, pitchg, yawg);
        // quatG.toMatrix(TF);
        // matrix_transpose(4, 4, TF, TF_trans);
        
        if(isInRange(rollg*180.0/M_PI, tolerance, 90) || isInRange(rollg*180.0/M_PI, tolerance, -90) || isInRange(pitchg*180.0/M_PI, tolerance, 90) || isInRange(pitchg*180.0/M_PI, tolerance, -90))
        {	alpha = 1.0;	}
        else
        {	alpha = _alpha;	}
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////         GET PR_accel         ////////////////////////////////////////

        // Convert accelerometer data to roll and pitch angles
        rolla  = atan2(accY, accZ);
        pitcha = -asin(accX/sqrt(accX*accX + accY*accY + accZ*accZ));
        // pitcha =  -atan2(accX, sqrt(accY*accY + accZ*accZ));
        yawa   = yawg;
        quatA = quatA.fromEuler(rolla, pitcha, yawa);
        
        quatC = quatC.slerp(quatA, quatG, alpha);
        quatG = quatC;
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////        Linear Motion         ////////////////////////////////////////

        getLinearAccelerationQuaternionGlobal(accelQuat);
        
        // velocitioyQuat.x = 1*(velocitioyQuat.x +(accX + accXprev)/2.0*dt/1000000.0) ;//+ (1-0.7)*velocitioyQuat.x ;
        // velocitioyQuat.y = 1*(velocitioyQuat.y +(accY + accYprev)/2.0*dt/1000000.0) ;//+ (1-0.7)*velocitioyQuat.y ;
        // velocitioyQuat.z = 1*(velocitioyQuat.z +(accZ + accZprev)/2.0*dt/1000000.0) ;//+ (1-0.7)*velocitioyQuat.z ;
        velocitioyQuat.x = 1*(velocitioyQuat.x +accelQuat.x*dt/1000000.0) ;//+ (1-0.7)*velocitioyQuat.x ;
        velocitioyQuat.y = 1*(velocitioyQuat.y +accelQuat.y*dt/1000000.0) ;//+ (1-0.7)*velocitioyQuat.y ;
        velocitioyQuat.z = 1*(velocitioyQuat.z +accelQuat.z*dt/1000000.0) ;//+ (1-0.7)*velocitioyQuat.z ;

        if(accelQuat.x==0.0){     velocitioyQuat.x = 0.996*(velocitioyQuat.x ) + (1-0.996)*0 ;    }
        if(accelQuat.y==0.0){     velocitioyQuat.y = 0.996*(velocitioyQuat.y ) + (1-0.996)*0 ;    }
        if(accelQuat.z==0.0){     velocitioyQuat.z = 0.996*(velocitioyQuat.z ) + (1-0.996)*0 ;    }
        
        positionQuat.x = 1*(positionQuat.x +velocitioyQuat.x*dt/1000000.0) ;//+ (1-0.7)*positionQuat.x ;
        positionQuat.y = 1*(positionQuat.y +velocitioyQuat.y*dt/1000000.0) ;//+ (1-0.7)*positionQuat.y ;
        positionQuat.z = 1*(positionQuat.z +velocitioyQuat.z*dt/1000000.0) ;//+ (1-0.7)*positionQuat.z ;
        
        
        
        
        
        accXprev = accX;
        accYprev = accY;
        accZprev = accZ;
        


        prevTime = currentTime;
    }
}

void MPU6050_Custom::toEuler(double &r, double &p, double &y)
{   quatC.toEuler(r, p, y);   }

// // Convert gyroscope data to quaternion_ (rad/sec)
// Quaternion_ MPU6050_Custom::gyroToQuaternion_(double gx, double gy, double gz) {
//     // Convert gyroscope data from degrees per second to radians per second
//     gx = gx * DEG_TO_RAD;
//     gy = gy * DEG_TO_RAD;
//     gz = gz * DEG_TO_RAD;
//     // Quaternion_ integration (simple approximation)
//     double qw = 1;
//     double qx = gx * dt / 2;
//     double qy = gy * dt / 2;
//     double qz = gz * dt / 2;
//     return Quaternion_(qw, qx, qy, qz);
// }

// Convert roll and pitch angles to quaternion_ (in rad)
Quaternion_ MPU6050_Custom::anglesToQuaternion(double roll, double pitch, double yaw) {
    // Assuming yaw is zero
    double cy = cos(yaw / 2);
    double sy = sin(yaw / 2);
    double cp = cos(pitch / 2);
    double sp = sin(pitch / 2);
    double cr = cos(roll / 2);
    double sr = sin(roll / 2);

    return Quaternion_(
        cr * cp * cy + sr * sp * sy, // w
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy  // z
    );
}

// Method to get orientation axis in global frame (not used)
void MPU6050_Custom::getOrientationAxisGlobal(double &x, double &y, double &z) {
    Quaternion_ axisX(0, 1, 0, 0);
    
    axisX = quatC % axisX % quatC.conjugate();
    x = axisX.x;
    y = axisX.y;
    z = axisX.z;
}

Quaternion_ MPU6050_Custom::getOrientationQuaternion()
{
    return quatC;
}

// Method to get global angular velocity (rad/sec)
void MPU6050_Custom::getAngularVelocityGlobal(double &gx, double &gy, double &gz) {
    // Assuming omegaX, omegaY, omegaZ are the local angular velocities
    Quaternion_ localOmega(0, omegaX, omegaY, omegaZ);
    Quaternion_ globalOmega = quatC % localOmega % quatC.conjugate();

    gx = globalOmega.x;
    gy = globalOmega.y;
    gz = globalOmega.z;
}

// Method to calculate and get global angular acceleration (rad/sec^2)
void MPU6050_Custom::getAngularAccelerationGlobal(double &ax, double &ay, double &az) {
    // Calculate local angular acceleration
    double localAx = omegaXd / dt;
    double localAy = omegaYd / dt;
    double localAz = omegaZd / dt;

    // Transform local angular acceleration to global frame
    Quaternion_ localAngularAcc(0, localAx, localAy, localAz);
    Quaternion_ globalAngularAcc = quatC % localAngularAcc % quatC.conjugate();

    ax = globalAngularAcc.x;
    ay = globalAngularAcc.y;
    az = globalAngularAcc.z;
}

// Method to get global linear acceleration (m/sec^2)
void MPU6050_Custom::getLinearAccelerationQuaternionGlobal(Quaternion_ &resultQuat) {
    // Assuming accX, accY, accZ are the local linear accelerations
    Quaternion_ localAcc(0, accX, accY, accZ);

    // Rotate the local acceleration vector to the global frame
    // Quaternion_ globalAcc = quatC.conjugate() % localAcc % quatC;
    Quaternion_ globalAcc = quatC % localAcc % quatC.conjugate();
    // Serial.println(accErrRange);
    


    // Gravity vector in the local frame (assuming Z-axis is up)
    Quaternion_ globalGravity(0, 0, 0, M_G);
    // Quaternion_ globalGravity(0, 0, 0, 0);

    // Rotate the gravity vector to the global frame
    // Quaternion_ globalGravity = quatC % localGravity % quatC.conjugate();

    // Subtract the gravity component from the global acceleration
    resultQuat = Quaternion_(
        0,
        (globalAcc.x - globalGravity.x),
        (globalAcc.y - globalGravity.y),
        (globalAcc.z - globalGravity.z)
    );
    // Serial.println(accErrRange);
    if(firstTime==false){
        if( isInRange(resultQuat.x, accErrRange, 0)){    resultQuat.x=0;    }
        if( isInRange(resultQuat.y, accErrRange, 0)){    resultQuat.y=0;    }
        if( isInRange(resultQuat.z, accErrRange, 0)){    resultQuat.z=0;    }
    }
    
}

// Method to get global linear acceleration (m/sec^2)
void MPU6050_Custom::getLinearAccelerationGlobal(double &ax, double &ay, double &az) {
    // Assuming accX, accY, accZ are the local linear accelerations
    Quaternion_ localAcc(0, accX, accY, accZ);

    // Rotate the local acceleration vector to the global frame
    Quaternion_ globalAcc = quatC % localAcc % quatC.conjugate();

    // Gravity vector in the local frame (assuming Z-axis is up)
    Quaternion_ globalGravity(0, 0, 0, M_G);

    // Rotate the gravity vector to the global frame
    // Quaternion_ globalGravity = quatC % localGravity % quatC.conjugate();

    // Subtract the gravity component from the global acceleration
    ax = (globalAcc.x - globalGravity.x);
    ay = (globalAcc.y - globalGravity.y);
    az = (globalAcc.z - globalGravity.z);
}

// Method to get local linear acceleration (m/sec^2)
void MPU6050_Custom::getLinearAccelerationLocal(double &ax, double &ay, double &az) {
    // Assuming accX, accY, accZ are the local linear accelerations
    Quaternion_ localAcc(0, accX, accY, accZ);

    // Gravity vector in the local frame (assuming Z-axis is up)
    Quaternion_ globalGravity(0, 0, 0, M_G);

    // Rotate the gravity vector to the global frame
    Quaternion_ localGravity = quatC.conjugate() % globalGravity % quatC;

    // Subtract the gravity component from the global acceleration
    ax = (localAcc.x - localGravity.x);
    ay = (localAcc.y - localGravity.y);
    az = (localAcc.z - localGravity.z);
}

void MPU6050_Custom::setOrientationQuaternion(Quaternion_ _quatC)
{
    quatC = _quatC;
}

void MPU6050_Custom::calculate_IMU_error()
{
    int i = 0;
    while (i < MPU_ERR_ITERATIONS)
    {
        mpu.getEvent(&a, &g, &temp);
        ////////////////////////////////////////////////////////
        {
            accErrX +=   a.acceleration.x;
            accErrY +=   a.acceleration.y;
            accErrZ +=   a.acceleration.z;
            omegaErrX += g.gyro.x;
            omegaErrY += g.gyro.y;
            omegaErrZ += g.gyro.z;

        }
        i++;
    }
    
    omegaErrX /= MPU_ERR_ITERATIONS;
    omegaErrY /= MPU_ERR_ITERATIONS;
    omegaErrZ /= MPU_ERR_ITERATIONS;
    
    accErrX /= MPU_ERR_ITERATIONS;
    accErrY /= MPU_ERR_ITERATIONS;
    accErrZ /= MPU_ERR_ITERATIONS;

    // the collected data were in mm/sec^2, and rad/sec
    // but in interpolation data lists, should be in m/s^2
    for(int listIndex=0; listIndex<listsSize; listIndex++)   {   if(interpolDataOmegaX[listIndex].y == 0.0){        interpolDataOmegaX[listIndex].x =  omegaErrX;     break; }    }
    for(int listIndex=0; listIndex<listsSize; listIndex++)   {   if(interpolDataOmegaY[listIndex].y == 0.0){        interpolDataOmegaY[listIndex].x =  omegaErrY;     break; }    }
    for(int listIndex=0; listIndex<listsSize; listIndex++)   {   if(interpolDataOmegaZ[listIndex].y == 0.0){        interpolDataOmegaZ[listIndex].x =  omegaErrZ;     break; }    }
    
    for(int listIndex=0; listIndex<listsSize; listIndex++)   {   if(interpolDataAccX[listIndex].y == 0.0){          interpolDataAccX[listIndex].x =  accErrX;  break; }    }
    for(int listIndex=0; listIndex<listsSize; listIndex++)   {   if(interpolDataAccY[listIndex].y == 0.0){          interpolDataAccY[listIndex].x =  accErrY;  break; }    }
    for(int listIndex=0; listIndex<listsSize; listIndex++)   {   if(interpolDataAccZ[listIndex].y == M_PI){         interpolDataAccZ[listIndex].x =  accErrZ;  break; }    }
    

    // interpolDataOmegaX[1].x =  omegaErrX;   
    // interpolDataOmegaY[1].x =  omegaErrY;   
    // interpolDataOmegaZ[1].x =  omegaErrZ;   
    // interpolDataAccX[1].x = accErrX/1000;  	
    // interpolDataAccY[1].x = accErrY/1000;
    // interpolDataAccZ[2].x = accErrZ/1000;
    //////////////////////////////////////////////////////////////////////////////////////////
    i = 0;
    while (i < MPU_ERR_ITERATIONS)
    {
        mpu.getEvent(&a, &g, &temp);
        ////////////////////////////////////////////////////////
        {
            // accErrXOffset   += 1000.0 *  interpolate_custom(interpolDataAccX,   listsSize, a.acceleration.x);
            // accErrYOffset   += 1000.0 *  interpolate_custom(interpolDataAccY,   listsSize, a.acceleration.y);
            // accErrZOffset   += 1000.0 *  interpolate_custom(interpolDataAccZ,   listsSize, a.acceleration.z);
            accErrXOffset   +=           interpolate_custom(interpolDataAccX,   listsSize, a.acceleration.x);
            accErrYOffset   +=           interpolate_custom(interpolDataAccY,   listsSize, a.acceleration.y);
            accErrZOffset   +=           interpolate_custom(interpolDataAccZ,   listsSize, a.acceleration.z);
            omegaErrXOffset +=           interpolate_custom(interpolDataOmegaX, listsSize, g.gyro.x);
            omegaErrYOffset +=           interpolate_custom(interpolDataOmegaY, listsSize, g.gyro.y);
            omegaErrZOffset +=           interpolate_custom(interpolDataOmegaZ, listsSize, g.gyro.z);

        }
        i++;
    }
    
    omegaErrXOffset /= MPU_ERR_ITERATIONS;
    omegaErrYOffset /= MPU_ERR_ITERATIONS;
    omegaErrZOffset /= MPU_ERR_ITERATIONS;
    accErrXOffset   /= MPU_ERR_ITERATIONS;
    accErrYOffset   /= MPU_ERR_ITERATIONS;
    accErrZOffset   /= MPU_ERR_ITERATIONS;
    accErrZOffset -= (M_G );
    //////////////////////////////////////////////////////////////////////////////////////////

    accX    =0;
    accY    =0;
    accZ    =0;
    omegaX  =0;
    omegaY  =0;
    omegaZ  =0;

    i = 0;
    while (i < MPU_ERR_ITERATIONS)
    {
        mpu.getEvent(&a, &g, &temp);
        ////////////////////////////////////////////////////////
        {
            // get & clean omega (rad/s), acc (mm/s^2):
            accX +=    ( interpolate_custom(interpolDataAccX,   listsSize, a.acceleration.x) - accErrXOffset);
            accY +=    ( interpolate_custom(interpolDataAccY,   listsSize, a.acceleration.y) - accErrYOffset);
            accZ +=    ( interpolate_custom(interpolDataAccZ,   listsSize, a.acceleration.z) - accErrZOffset);
            omegaX += (interpolate_custom(interpolDataOmegaX, listsSize, g.gyro.x)         - omegaErrXOffset);
            omegaY += (interpolate_custom(interpolDataOmegaY, listsSize, g.gyro.y)         - omegaErrYOffset);
            omegaZ += (interpolate_custom(interpolDataOmegaZ, listsSize, g.gyro.z)         - omegaErrZOffset);
        
        }
        i++;
    }
    
    accX /= MPU_ERR_ITERATIONS;
    accY /= MPU_ERR_ITERATIONS;
    accZ /= MPU_ERR_ITERATIONS;
    omegaX   /= MPU_ERR_ITERATIONS;
    omegaY   /= MPU_ERR_ITERATIONS;
    omegaZ   /= MPU_ERR_ITERATIONS;
    accZ -= (M_G );
    
    accErrXOffset += accX;
    accErrYOffset += accY;
    accErrZOffset += accZ;
    omegaErrXOffset += omegaX;
    omegaErrYOffset += omegaY;
    omegaErrZOffset += omegaZ;

    accX=0;
    accY=0;
    accZ=0;
    omegaX=0;
    omegaY=0;
    omegaZ=0;
    //////////////////////////////////////////////////////////////////////////////////////////
    
    i = 0;
    while (i < MPU_ERR_ITERATIONS)
    {
        mpu.getEvent(&a, &g, &temp);
        ////////////////////////////////////////////////////////
        {

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////        GET clean Data        ////////////////////////////////////////
        // Get new sensor events with the readings 
        
        
            accX =    1*( interpolate_custom(interpolDataAccX,   listsSize, a.acceleration.x) - accErrXOffset) + (1-1)*accX;
            accY =    1*( interpolate_custom(interpolDataAccY,   listsSize, a.acceleration.y) - accErrYOffset) + (1-1)*accY;
            accZ =    1*( interpolate_custom(interpolDataAccZ,   listsSize, a.acceleration.z) - accErrZOffset) + (1-1)*accZ;
            omegaX = 0.99*(interpolate_custom(interpolDataOmegaX, listsSize, g.gyro.x)         - omegaErrXOffset)+ (1-0.99)*omegaX;
            omegaY = 0.99*(interpolate_custom(interpolDataOmegaY, listsSize, g.gyro.y)         - omegaErrYOffset)+ (1-0.99)*omegaY;
            omegaZ = 0.99*(interpolate_custom(interpolDataOmegaZ, listsSize, g.gyro.z)         - omegaErrZOffset)+ (1-0.99)*omegaZ;

            getLinearAccelerationQuaternionGlobal(accelQuat);
            
            if(abs(accelQuat.x) > accErrRange){      accErrRange=abs(accelQuat.x);      }
            if(abs(accelQuat.y) > accErrRange){      accErrRange=abs(accelQuat.y);      }
            if(abs(accelQuat.z) > accErrRange){      accErrRange=abs(accelQuat.z);      }
            
            if(abs(omegaX) > gyroErrRange){      gyroErrRange=abs(omegaX);      }
            if(abs(omegaY) > gyroErrRange){      gyroErrRange=abs(omegaY);      }
            if(abs(omegaZ) > gyroErrRange){      gyroErrRange=abs(omegaZ);      }

        }
        i++;
    }
    
    
    accX=0;
    accY=0;
    accZ=M_G;
    omegaX=0;
    omegaY=0;
    omegaZ=0;
    
    


}





