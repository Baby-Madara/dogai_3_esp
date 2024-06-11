#ifndef LEG_ODOMETRY_H
#define LEG_ODOMETRY_H


#include <Arduino.h>
#include <utilities.h>
#include <MPU_Custom.h>

#include <iostream>
#include <cmath>
#include <vector>
#include <iostream>
#include <utility>
#include <stdexcept>
#include <array>
#include <ArduinoEigenDense.h>
// #include <Eigen.h>
// #include <Eigen/Dense>
#include <time.h>


using namespace Eigen;








class FK_BP
{
private:
    float Xbs, Ybs, Zbs, Ysl, Xlf, Xfp;

public:
    FK_BP(float Xbs, float Ybs, float Zbs, float Ysl, float Xlf, float Xfp);
    Matrix4f calc_xyz(float thetaS, float thetaL, float thetaF);

};




class Pybullet_Interface {
private:
    std::vector<FK_BP> fk_objects;
    std::vector<int> motor_joint_indices;
    Vector3f P_odomtery_to_palm;
    Vector3f P_odomtery_to_body;
    Vector3f P_body_to_palm;
    Matrix4f tf_o_b;
    Vector4i is_stance;
    std::vector<std::pair<std::string, int>> palm_indices;
    std::string last_tracking_palm;
    unsigned long lastTime;


    int get_palm_index(const std::string& palm_name);



public:
    Pybullet_Interface();
    void publish_transform_odometry(double jointPositions[14], int palmStates[4],           float &pos_x, float &pos_y, float &pos_z,        float w=1, float x=0, float y=0, float z=0);
    // void publish_current_sensor_data();


};




#endif  // LEG_ODOMETRY_H