#include <robotics_math.h>


////////////////////////////////// ik //////////////////////////////////

Palm::Palm(float Xbs, float Ybs, float Zbs, float Ysl, float Xlf, float Xfp)
    :        Xbs(Xbs),  Ybs(Ybs),  Zbs(Zbs),  Ysl(Ysl),  Xlf(Xlf),  Xfp(Xfp) {}

// std::vector<float> Palm::ikCalc(const std::vector<float>& goalVector)
void Palm::ikCalc(float goalVector[3], float thetas_array[3])
{
    float X_goal = goalVector[0];
    float Y_goal = goalVector[1];
    float Z_goal = goalVector[2];

    float D, thetaS, thetaF, thetaL;
    
    
    
    
    
    if ( sqrt((pow(Y_goal - Ybs, 2) + pow(Z_goal - Zbs, 2)))   <= 0.071  )
    {
        // Serial.println("it is inside cylinder");
        
        if      ((X_goal - Xbs)>  0.4){X_goal = Xbs+0.39;}
        else if ((X_goal - Xbs)< -0.4){X_goal = Xbs-0.39;}
        
        
        
        float legLen_over_mag = 0.072 / sqrt(pow(Y_goal - Ybs, 2) + pow(Z_goal - Zbs, 2));
        
        Y_goal = Ybs + legLen_over_mag * (Y_goal - Ybs);
        Z_goal = Zbs + legLen_over_mag * (Z_goal - Zbs);
        
        
        
        D = ((pow(Y_goal - Ybs, 2) + pow(Z_goal - Zbs, 2) + pow(X_goal - Xbs, 2)) - (pow(Ysl, 2) + pow(Xlf, 2) + pow(Xfp, 2))) / (2 * Xlf * Xfp);
        
        thetaS = -atan2(-(Z_goal - Zbs), (Y_goal - Ybs)) + atan2(sqrt(pow(Y_goal - Ybs, 2) + pow(Z_goal - Zbs, 2) - pow(Ysl, 2)), Ysl);
        thetaF = atan2(sqrt(1 - pow(D, 2)), D);
        thetaL = atan2((X_goal - Xbs), sqrt(pow(Y_goal - Ybs, 2) + pow(Z_goal - Zbs, 2) - pow(Ysl, 2))) - atan2(Xfp * sin(thetaF), Xlf + Xfp * cos(thetaF));
        
    }
    else
    {
        // Serial.println("");
        try
        {
            D = ((pow(Y_goal - Ybs, 2) + pow(Z_goal - Zbs, 2) + pow(X_goal - Xbs, 2)) - (pow(Ysl, 2) + pow(Xlf, 2) + pow(Xfp, 2))) / (2 * Xlf * Xfp);

            thetaS = -atan2(-(Z_goal - Zbs), (Y_goal - Ybs)) + atan2(sqrt(pow(Y_goal - Ybs, 2) + pow(Z_goal - Zbs, 2) - pow(Ysl, 2)), Ysl);
            thetaF = atan2(sqrt(1 - pow(D, 2)), D);
            thetaL = atan2((X_goal - Xbs), sqrt(pow(Y_goal - Ybs, 2) + pow(Z_goal - Zbs, 2) - pow(Ysl, 2))) - atan2(Xfp * sin(thetaF), Xlf + Xfp * cos(thetaF));
        }
        catch (...)
        {
            float legLen_over_mag = 0.4057 / sqrt(pow(X_goal - Xbs, 2) + pow(Y_goal - Ybs, 2) + pow(Z_goal - Zbs, 2));

            X_goal = Xbs + legLen_over_mag * (X_goal - Xbs);
            Y_goal = Ybs + legLen_over_mag * (Y_goal - Ybs);
            Z_goal = Zbs + legLen_over_mag * (Z_goal - Zbs);

            D = ((pow(Y_goal - Ybs, 2) + pow(Z_goal - Zbs, 2) + pow(X_goal - Xbs, 2)) - (pow(Ysl, 2) + pow(Xlf, 2) + pow(Xfp, 2))) / (2 * Xlf * Xfp);
            
            thetaS = -atan2(-(Z_goal - Zbs), (Y_goal - Ybs)) + atan2(sqrt(pow(Y_goal - Ybs, 2) + pow(Z_goal - Zbs, 2) - pow(Ysl, 2)), Ysl);
            thetaF = atan2(sqrt(1 - pow(D, 2)), D);
            thetaL = atan2((X_goal - Xbs), sqrt(pow(Y_goal - Ybs, 2) + pow(Z_goal - Zbs, 2) - pow(Ysl, 2))) - atan2(Xfp * sin(thetaF), Xlf + Xfp * cos(thetaF));
        }

    }
    
    thetaS = thetaS*(180.0/PI);
    thetaL = thetaL*(180.0/PI);
    thetaF = thetaF*(180.0/PI);
    
    thetas_array[0] = thetaS;
    thetas_array[1] = thetaL;
    thetas_array[2] = thetaF;
    
    Serial.println(String("")+String(thetaS)+String(",")+String(thetaL)+String(",")+String(thetaF));
    

    // return {thetaS, thetaL, thetaF};
}













////////////////////////////////// foot bezier //////////////////////////////////




Matrix4f translation_matrix(float tx, float ty, float tz)
{
    Matrix4f mat = Matrix4f::Identity();
    mat(0, 3) = tx;
    mat(1, 3) = ty;
    mat(2, 3) = tz;
    return mat;
}

Matrix4f rotation_x_matrix(float theta)
{
    Matrix4f mat = Matrix4f::Identity();
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    mat(1, 1) = cos_theta;
    mat(1, 2) = -sin_theta;
    mat(2, 1) = sin_theta;
    mat(2, 2) = cos_theta;
    return mat;
}

Matrix4f rotation_y_matrix(float theta)
{
    Matrix4f mat = Matrix4f::Identity();
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    mat(0, 0) = cos_theta;
    mat(0, 2) = sin_theta;
    mat(2, 0) = -sin_theta;
    mat(2, 2) = cos_theta;
    return mat;
}

Matrix4f rotation_z_matrix(float theta)
{
    Matrix4f mat = Matrix4f::Identity();
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    mat(0, 0) = cos_theta;
    mat(0, 1) = -sin_theta;
    mat(1, 0) = sin_theta;
    mat(1, 1) = cos_theta;
    return mat;
}




std::pair<Matrix<float, 3, 1>, bool> compound_bezier(
    float t,
    float total_step_time,
    float speed,
    float stance_ratio,
    float phase_shift,
    float z_via,
    float curve_width_ratio,
    float penetration_depth,
    float curving_radius,
    float rotation_angle,
    float distance_shift
)
{
    Matrix<float, 3, 1> output(0, 0, 0);

    if (stance_ratio >= 0.95) {
        stance_ratio = 0.95;
    }

    float step_size = speed * (total_step_time * stance_ratio);
    if (step_size == 0) {
        z_via = 0;
    }

    float t_local = t + phase_shift;
    if (t_local >= 1) {
        t_local -= static_cast<int>(t_local);
    }

    bool stance_phase = false;

    float t_1, t_2, t_3, t_4;
    t_1 = t_2 = (total_step_time * stance_ratio) / 2;
    t_3 = t_4 = (total_step_time * (1 - stance_ratio)) / 2;

    float t_sum = t_1 + t_2 + t_3 + t_4;

    float s_1 = t_1 / t_sum;
    float s_2 = t_2 / t_sum;
    float s_3 = t_3 / t_sum;
    float s_4 = t_4 / t_sum;

    Matrix<float, 3, 1> p0(distance_shift + (step_size / 2),                            0.0, 0.0);
    Matrix<float, 3, 1> p1(distance_shift + (step_size / 4),                            0.0, -penetration_depth);
    Matrix<float, 3, 1> p2(distance_shift + 0,                                          0.0, -penetration_depth);
    Matrix<float, 3, 1> p3(distance_shift + (-step_size / 4),                           0.0, -penetration_depth);
    Matrix<float, 3, 1> p4(distance_shift + (-step_size / 2),                           0.0, 0.0);
    Matrix<float, 3, 1> p5 = (1.0 / 3.0) * (5 * p4 - 2 * p3);
    Matrix<float, 3, 1> p6(distance_shift + ((-step_size / 2) * curve_width_ratio),     0.0, z_via);
    Matrix<float, 3, 1> p7(distance_shift + 0,                                          0.0, z_via);
    Matrix<float, 3, 1> p8(distance_shift + ((step_size / 2) * curve_width_ratio),      0.0, z_via);
    Matrix<float, 3, 1> p9 = (1.0 / 3.0) * (5 * p0 - 2 * p1);

    float t_bezier;
    if (t_local < s_1)
    {
        t_bezier = (t_local - 0) / s_1;
        output = pow((1 - t_bezier), 2) * p0 + 2 * (1 - t_bezier) * t_bezier * p1 + pow(t_bezier, 2) * p2;
        stance_phase = true;
    }
    else if (t_local < s_1 + s_2)
    {
        t_bezier = (t_local - s_1) / s_2;
        output = pow((1 - t_bezier), 2) * p2 + 2 * (1 - t_bezier) * t_bezier * p3 + pow(t_bezier, 2) * p4;
        stance_phase = true;
    }
    else if (t_local < s_1 + s_2 + s_3)
    {
        t_bezier = (t_local - (s_1 + s_2)) / s_3;
        output = pow((1 - t_bezier), 3) * p4 + 3 * pow((1 - t_bezier), 2) * t_bezier * p5 + 3 * (1 - t_bezier) * pow(t_bezier, 2) * p6 + pow(t_bezier, 3) * p7;
        stance_phase = false;
    }
    else if (t_local < s_1 + s_2 + s_3 + s_4)
    {
        t_bezier = (t_local - (s_1 + s_2 + s_3)) / s_4;
        output = pow((1 - t_bezier), 3) * p7 + 3 * pow((1 - t_bezier), 2) * t_bezier * p8 + 3 * (1 - t_bezier) * pow(t_bezier, 2) * p9 + pow(t_bezier, 3) * p0;
        stance_phase = false;
    }
    
    if (abs(curving_radius) < 500)
    {
        Matrix<float, 3, 1> norm_radius  = (output - Matrix<float, 3, 1>(0, curving_radius, output(2))) / (output - Matrix<float, 3, 1>(0, curving_radius, output(2))).norm();
        output                           = curving_radius * norm_radius + Matrix<float, 3, 1>(0, curving_radius, output(2));
    }

    Matrix<float, 4, 1> output_homogeneous(output(0), output(1), output(2), 1.0);
    output_homogeneous   = rotation_z_matrix(rotation_angle) * output_homogeneous;
    output               = output_homogeneous.head<3>();


    return std::make_pair(output, stance_phase);
}















// std::pair<Matrix<float, 3, 1>, bool> compound_bezier(
//     float t,
//     float step_time_quarter,
//     float speed,
//     float terminating_stance_to_q_step_ratio,
//     float phase_shift,
//     float z_via,
//     float curving_radius,
//     float swing_curving_to_ssq_ratio,
//     float penetration_depth,
//     float rotation_angle,
//     bool smooth
// )
// {
//     Matrix<float, 3, 1> output(0, 0, 0);
//     float step_size_quarter = speed * step_time_quarter;
//     if (step_size_quarter == 0)
//     {
//         z_via = 0;
//     }

//     if (terminating_stance_to_q_step_ratio >= 0.4)
//     {
//         terminating_stance_to_q_step_ratio = 0.4;
//     }

//     float t_local = t + phase_shift;
//     if (t_local > 1) {
//         t_local -= static_cast<int>(t_local);
//     }

//     bool stance_phase = false;

//     float t_1  = step_time_quarter;
//     float t_2  = step_time_quarter;
//     float t_3  = step_time_quarter;
//     float t_4  = t_1 * terminating_stance_to_q_step_ratio;
//     float t_56 = t_1 * (0.5 - terminating_stance_to_q_step_ratio);
//     float t_78 = t_1 * (0.5 - terminating_stance_to_q_step_ratio);
//     float t_9  = t_1 * terminating_stance_to_q_step_ratio;

//     float t_sum = t_1 + t_2 + t_3 + t_4 + t_56 + t_78 + t_9;

//     float s_1  = t_1  / t_sum;
//     float s_2  = t_2  / t_sum;
//     float s_3  = t_3  / t_sum;
//     float s_4  = t_4  / t_sum;
//     float s_56 = t_56 / t_sum;
//     float s_78 = t_78 / t_sum;
//     float s_9  = t_9  / t_sum;

//     Matrix<float, 3, 1> p0(  1.5 * step_size_quarter,                                                                     0.0,    0.0                );
//     Matrix<float, 3, 1> p1(  0.5 * step_size_quarter,                                                                     0.0,    -penetration_depth );
//     Matrix<float, 3, 1> p2( -0.5 * step_size_quarter,                                                                     0.0,    -penetration_depth );
//     Matrix<float, 3, 1> p3( -1.5 * step_size_quarter,                                                                     0.0,    0.0                );
//     Matrix<float, 3, 1> p4( -step_size_quarter * (1.5 + terminating_stance_to_q_step_ratio),                              0.0,    0.0                );
//     Matrix<float, 3, 1> p5( -step_size_quarter * (1.5 + terminating_stance_to_q_step_ratio + swing_curving_to_ssq_ratio), 0.0,    z_via              );
//     Matrix<float, 3, 1> p6(  0.0,                                                                                         0.0,    z_via              );
//     Matrix<float, 3, 1> p7(  step_size_quarter * (1.5 + terminating_stance_to_q_step_ratio + swing_curving_to_ssq_ratio), 0.0,    z_via              );
//     Matrix<float, 3, 1> p8(  step_size_quarter * (1.5 + terminating_stance_to_q_step_ratio),                              0.0,    0.0                );
//     Matrix<float, 3, 1> p9 = p0;

//     float t_bezier;
//     if (t_local < s_1)
//     {
//         t_bezier     = (t_local) / s_1;
//         output       = (1 - t_bezier) * p0 + t_bezier * p1;
//         stance_phase = true;
//     }
//     else if (t_local < s_1 + s_2)
//     {
//         t_bezier = (t_local - s_1) / s_2;
//         output = (1 - t_bezier) * p1 + t_bezier * p2;
//         stance_phase = true;
//     }
//     else if (t_local < s_1 + s_2 + s_3)
//     {
//         t_bezier = (t_local - (s_1 + s_2)) / s_3;
//         output = (1 - t_bezier) * p2 + t_bezier * p3;
//         stance_phase = true;
//     }
//     else if (t_local < s_1 + s_2 + s_3 + s_4)
//     {
//         t_bezier = (t_local - (s_1 + s_2 + s_3)) / s_4;
//         output = (1 - t_bezier) * p3 + t_bezier * p4;
//         stance_phase = true;
//     }
//     else if (t_local < s_1 + s_2 + s_3 + s_4 + s_56)
//     {
//         t_bezier = (t_local - (s_1 + s_2 + s_3 + s_4)) / s_56;
//         if (smooth)
//         {
//             t_bezier /= 2;
//             t_bezier = 3 * pow(t_bezier, 2) - 2 * pow(t_bezier, 3);
//             t_bezier *= 2;
//         }
//         output = pow((1 - t_bezier), 2) * p4 + 2 * (1 - t_bezier) * t_bezier * p5 + pow(t_bezier, 2) * p6;
//         stance_phase = false;
//     }
//     else if (t_local < s_1 + s_2 + s_3 + s_4 + s_56 + s_78)
//     {
//         t_bezier = (t_local - (s_1 + s_2 + s_3 + s_4 + s_56)) / s_78;
//         if (smooth)
//         {
//             t_bezier = t_bezier / 2 + 0.5;
//             t_bezier = 3 * pow(t_bezier, 2) - 2 * pow(t_bezier, 3);
//             t_bezier = (t_bezier - 0.5) * 2;
//         }
//         output       = pow((1 - t_bezier), 2) * p6 + 2 * (1 - t_bezier) * t_bezier * p7 + pow(t_bezier, 2) * p8;
//         stance_phase = false;
//     }
//     else if (t_local <= s_1 + s_2 + s_3 + s_4 + s_56 + s_78 + s_9)
//     {
//         t_bezier     = (t_local - (s_1 + s_2 + s_3 + s_4 + s_56 + s_78)) / s_9;
//         output       = (1 - t_bezier) * p8 + t_bezier * p9;
//         stance_phase = true;
//     }

//     if (abs(curving_radius) < 500)
//     {
//         Matrix<float, 3, 1> norm_radius  = (output - Matrix<float, 3, 1>(0, curving_radius, output(2))) / (output - Matrix<float, 3, 1>(0, curving_radius, output(2))).norm();
//         output                           = curving_radius * norm_radius + Matrix<float, 3, 1>(0, curving_radius, output(2));
//     }

//     Matrix<float, 4, 1> output_homogeneous(output(0), output(1), output(2), 1.0);
//     output_homogeneous   = rotation_z_matrix(rotation_angle) * output_homogeneous;
//     output               = output_homogeneous.head<3>();

//     return std::make_pair(output, stance_phase);
// }






















///////////////////////////////// gait manager //////////////////////////////////////////


GaitManager::GaitManager(float leg_Xf,   float leg_Xb,  float leg_Y,     float z_height,            float total_step_time,         float publish_freq)
                    :   leg_Xf(leg_Xf), leg_Xb(leg_Xb), leg_Y(leg_Y), z_height(z_height), total_step_time(total_step_time), publish_freq(publish_freq), t(0), x_position(0), y_position(0), prev_instance(0) 
{
    publish_interval        = 1.0 / publish_freq;
    legs_thetas             = Vector4f::Zero();
    legs_curves_radii       = Vector4f::Zero();
    legs_speeds             = Vector4f::Zero();
    palms_cmd               = MatrixXf::Zero(4, 3);
    vx_vy_wz                = Vector3f(0.0, 0.0, 0.0);
    horizontal_rotation_rpy = Vector3f::Zero();
    inclined_rotation_rpy   = Vector3f::Zero();
    legs_Xf_Xb_Y            = Vector3f(leg_Xf, leg_Xb, leg_Y);
    gait_center             = Vector3f(x_position, y_position, z_height);
    is_stance               = Vector4i::Zero();
}


void GaitManager::update_commands(float vx_vy_wz_[3], float rpy1_[3], float rpy2_[3], float trans_xyz_[3], float z_via_)
{
    vx_vy_wz[0] = vx_vy_wz_[0];
    vx_vy_wz[1] = vx_vy_wz_[1];
    vx_vy_wz[2] = vx_vy_wz_[2];

    horizontal_rotation_rpy[0] = rpy1_[0];
    horizontal_rotation_rpy[1] = rpy1_[1];
    horizontal_rotation_rpy[2] = rpy1_[2];

    inclined_rotation_rpy[0] = rpy2_[0];
    inclined_rotation_rpy[1] = rpy2_[1];
    inclined_rotation_rpy[2] = rpy2_[2];
    
    x_position = trans_xyz_[0];
    y_position = trans_xyz_[1];
    z_height   = trans_xyz_[2];
    z_via = z_via_;
    
}


void GaitManager::publishing_routine(float xyz_cmd_array[4][3])
{
    float phase_shift[] = {0.75, 0.25, 0.0, 0.5};

    // Transformation matrices for the robot
    Matrix4f tf_gb_tr             = translation_matrix(x_position, y_position, z_height);
    Matrix4f tf_gb_horizontal_RPY = rotation_z_matrix(horizontal_rotation_rpy[2]) * rotation_y_matrix(horizontal_rotation_rpy[1]) * rotation_x_matrix(horizontal_rotation_rpy[0]);
    Matrix4f tf_gb_horizontal     = tf_gb_tr * tf_gb_horizontal_RPY;
    Matrix4f tf_bg_horizontal     = tf_gb_horizontal.inverse();
    Matrix4f tf_gn_inclined_RPY   = rotation_z_matrix(inclined_rotation_rpy[2]) * rotation_y_matrix(inclined_rotation_rpy[1]) * rotation_x_matrix(inclined_rotation_rpy[0]);
    Matrix4f tf_bg                = tf_bg_horizontal * tf_gn_inclined_RPY;

    // Leg translation matrices
    Matrix4f tf_gi_tr[4] = {
        translation_matrix(legs_Xf_Xb_Y[0], -legs_Xf_Xb_Y[2], 0),
        translation_matrix(legs_Xf_Xb_Y[0],  legs_Xf_Xb_Y[2], 0),
        translation_matrix(legs_Xf_Xb_Y[1], -legs_Xf_Xb_Y[2], 0),
        translation_matrix(legs_Xf_Xb_Y[1],  legs_Xf_Xb_Y[2], 0),
    };

    // Setting velocities and angles based on conditions
    if (vx_vy_wz[2] == 0)
    {
        Vector3f V_vector(vx_vy_wz[0], vx_vy_wz[1], 0);
        float V_mag              = V_vector.norm();
        float theta              = atan2(vx_vy_wz[1], vx_vy_wz[0]);
        
        for (int i = 0; i < 4; ++i) {
            legs_thetas[i]       = theta;
            legs_curves_radii[i] = 1000;  // Arbitrarily large radius to approximate straight line
            legs_speeds[i]       = V_mag;
        }
    }
    else if (vx_vy_wz[0] == 0 && vx_vy_wz[1] == 0)
    {
        for (int i = 0; i < 4; ++i) {
            legs_thetas[i]       = atan2(tf_gi_tr[i](0, 3), -tf_gi_tr[i](1, 3));
            legs_curves_radii[i] = sqrt(pow(tf_gi_tr[i](0, 3), 2) + pow(tf_gi_tr[i](1, 3), 2));
            legs_speeds[i]       = legs_curves_radii[i] * vx_vy_wz[2];
        }
    }
    else
    {
        Vector4f V_vector(vx_vy_wz[0], vx_vy_wz[1], 0, 1);
        float V_mag              = V_vector.norm();
        float R                  = abs(V_mag / vx_vy_wz[2]);
        Vector3f rotation_center = -R * (rotation_z_matrix(-PI / 2) * V_vector / V_mag).head<3>();

        for (int i = 0; i < 4; ++i) {
            legs_thetas[i]       = atan2(tf_gi_tr[i](0, 3) - rotation_center[0], rotation_center[1] - tf_gi_tr[i](1, 3));
            legs_curves_radii[i] = (tf_gi_tr[i].block<3, 1>(0, 3) - rotation_center).norm();
            legs_speeds[i]       = legs_curves_radii[i] * vx_vy_wz[2];
        }
    }

    for (int i = 0; i < 4; ++i) {
        // auto result = compound_bezier(t, total_step_time / 4.0, legs_speeds[i], 0.3, phase_shift[i], 0.1, legs_curves_radii[i], 0.2, 0.0, legs_thetas[i], true);
        auto result = compound_bezier(  
            t,                      // t
            total_step_time,        // total_step_time
            legs_speeds[i],         // speed
            0.85,                   // stance_ratio = 0.85
            phase_shift[i],         // phase_shift = 0
            // 0.2,                    // z_via = 0.2
            z_via,                     // z_via = 0.2
            1.0,                    // curve_width_ratio = 1.0
            0.0,                    // penetration_depth = 0
            legs_curves_radii[i],   // curving_radius = 1000
            legs_thetas[i],         // rotation_angle = 0
            0.0                     // distance_shift = 0
            
        );
        Vector3f p_i_out = result.first;
        is_stance[i] = result.second;
        palms_cmd.row(i) = (tf_bg * tf_gi_tr[i] * p_i_out.homogeneous()).head<3>();
    }

    t += (millis() - prev_instance) / 1000.0 / total_step_time;
    if (t > 1) t = 0;
    
    if(t<=0.1 || t>=0.9){ Serial.println("t:    ");   Serial.println(t);}
    
    
    prev_instance = millis();

    for (int i = 0; i < palms_cmd.rows(); ++i) {
        for (int j = 0; j < palms_cmd.cols(); ++j) {
            xyz_cmd_array[i][j] = palms_cmd(i, j);
            
            
            // Serial.print(palms_cmd(i, j));
            // Serial.print(j == palms_cmd.cols() - 1 ? "\n" : ", ");
        }
    }
    // Serial.println("");
}

void GaitManager::loop(float xyz_cmd_array[4][3])
{
    publishing_routine(xyz_cmd_array);
    vTaskDelay(1000 / publish_freq); // to match the publishing frequency
    // delay(1000 / publish_freq); // to match the publishing frequency
}



// void setup()
// {
    // Serial.begin(115200);
// }
// Matrix4f GaitManager::translation_matrix(float x, float y, float z) {
//     Matrix4f mat = Matrix4f::Identity();
//     mat(0, 3) = x;
//     mat(1, 3) = y;
//     mat(2, 3) = z;
//     return mat;
// }
// Matrix4f GaitManager::rotation_x_matrix(float angle) {
//     Matrix4f mat = Matrix4f::Identity();
//     mat(1, 1) = cos(angle);
//     mat(1, 2) = -sin(angle);
//     mat(2, 1) = sin(angle);
//     mat(2, 2) = cos(angle);
//     return mat;
// }
// Matrix4f GaitManager::rotation_y_matrix(float angle) {
//     Matrix4f mat = Matrix4f::Identity();
//     mat(0, 0) = cos(angle);
//     mat(0, 2) = sin(angle);
//     mat(2, 0) = -sin(angle);
//     mat(2, 2) = cos(angle);
//     return mat;
// }
// Matrix4f GaitManager::rotation_z_matrix(float angle) {
//     Matrix4f mat = Matrix4f::Identity();
//     mat(0, 0) = cos(angle);
//     mat(0, 1) = -sin(angle);
//     mat(1, 0) = sin(angle);
//     mat(1, 1) = cos(angle);
//     return mat;
// }

