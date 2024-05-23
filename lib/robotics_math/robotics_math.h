#ifndef ROBOTICS_MATH_H
#define ROBOTICS_MATH_H


#include <Arduino.h>
#include <utilities.h>



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
using namespace std;







class Palm {
public:
    Palm(float Xbs, float Ybs, float Zbs, float Ysl, float Xlf, float Xfp);

    std::vector<float> ikCalc(const std::vector<float>& goalVector);


private:
    float Xbs, Ybs, Zbs, Ysl, Xlf, Xfp;
};
























//////////////////////////////// foot bezier ////////////////////////////



Matrix4f translation_matrix(float tx, float ty, float tz);
Matrix4f rotation_x_matrix(float theta);
Matrix4f rotation_y_matrix(float theta);
Matrix4f rotation_z_matrix(float theta);
std::pair<Matrix<float, 3, 1>, bool> compound_bezier(
    float t, float step_time_quarter, float speed, 
    float terminating_stance_to_q_step_ratio = 0.2, float phase_shift = 0, 
    float z_via = 0.2, float curving_radius = 1000, float swing_curving_to_ssq_ratio = 1, 
    float penetration_depth = 0, float rotation_angle = 0, bool smooth = true
);















/////////////////////////////////// steady gait /////////////////////////////////





class GaitManager {
private:
    float leg_Xf, leg_Xb, leg_Y, z_height, total_step_time, publish_freq, publish_interval;
    float t, x_position, y_position;
    Vector4f legs_thetas, legs_curves_radii, legs_speeds;
    MatrixXf palms_cmd;
    Vector3f vx_vy_wz, horizontal_rotation_rpy, inclined_rotation_rpy, legs_Xf_Xb_Y, gait_center;
    Vector4i is_stance;
    unsigned long prev_instance;

public:







    GaitManager(float leg_Xf = 0.2044, float leg_Xb = -0.2044, float leg_Y = 0.171, float z_height = 0.3, float total_step_time = 2.5, float publish_freq = 20);
    void publishing_routine(float xyz_cmd_array[4][3]);
    void loop(float xyz_cmd_array[4][3]);
    void update_commands();

    




};





// GaitManager gaitManager;

// void setup() {
//     gaitManager.setup();
// }

// void loop() {
//     gaitManager.loop();
// }



























#endif