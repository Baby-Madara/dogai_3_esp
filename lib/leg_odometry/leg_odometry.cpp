#include <leg_odometry.h>





FK_BP palm_fr(0.2044, -0.1, 0, -0.071, 0.2, 0.2);
FK_BP palm_fl(0.2044, 0.1, 0, 0.071, 0.2, 0.2);
FK_BP palm_br(-0.2044, -0.1, 0, -0.071, 0.2, 0.2);
FK_BP palm_bl(-0.2044, 0.1, 0, 0.071, 0.2, 0.2);




FK_BP::FK_BP(float Xbs, float Ybs, float Zbs, float Ysl, float Xlf, float Xfp): Xbs(Xbs), Ybs(Ybs), Zbs(Zbs), Ysl(Ysl), Xlf(Xlf), Xfp(Xfp){}


Matrix4f FK_BP::calc_xyz(float thetaS, float thetaL, float thetaF)
{
    Matrix4f result = Matrix4f::Zero();

    result(0, 0) = sin(thetaF + thetaL);
    result(0, 1) = cos(thetaF + thetaL);
    result(0, 2) = 0;
    result(0, 3) = Xbs + Xfp * sin(thetaF + thetaL) + Xlf * sin(thetaL);

    result(1, 0) = cos(thetaF + thetaL) * sin(thetaS);
    result(1, 1) = -sin(thetaF + thetaL) * sin(thetaS);
    result(1, 2) = -cos(thetaS);
    result(1, 3) = Ybs + Ysl * cos(thetaS) + Xfp * cos(thetaF + thetaL) * sin(thetaS) + Xlf * cos(thetaL) * sin(thetaS);

    result(2, 0) = -cos(thetaF + thetaL) * cos(thetaS);
    result(2, 1) = sin(thetaF + thetaL) * cos(thetaS);
    result(2, 2) = -sin(thetaS);
    result(2, 3) = Zbs + Ysl * sin(thetaS) - Xfp * cos(thetaF + thetaL) * cos(thetaS) - Xlf * cos(thetaL) * cos(thetaS);

    result(3, 0) = 0;
    result(3, 1) = 0;
    result(3, 2) = 0;
    result(3, 3) = 1;

    return result;
}








Matrix4f to_translation_matrix(float tx, float ty, float tz)
{
    Matrix4f mat = Matrix4f::Identity();
    mat(0, 3) = tx;
    mat(1, 3) = ty;
    mat(2, 3) = tz;
    return mat;
}











int Pybullet_Interface::get_palm_index(const std::string& palm_name) {
    for (const auto& pair : palm_indices) {
        if (pair.first == palm_name) {
            return pair.second;
        }
    }
    return -1; // Return an invalid index if not found
}

Pybullet_Interface::Pybullet_Interface():
    fk_objects({
        FK_BP(0.2044, -0.1, 0, -0.071, 0.2, 0.2),
        FK_BP(0.2044, 0.1, 0, 0.071, 0.2, 0.2),
        FK_BP(-0.2044, -0.1, 0, -0.071, 0.2, 0.2),
        FK_BP(-0.2044, 0.1, 0, 0.071, 0.2, 0.2)
    }),
    // motor_joint_indices({0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14}),
    P_odomtery_to_palm(0.2, -0.17, 0.0),
    P_body_to_palm(0.2, -0.17, -0.2),
    tf_o_b(Matrix4f::Identity()),
    is_stance({0, 0, 0, 0}),
    palm_indices({{"palm_fr", 0}, {"palm_fl", 1}, {"palm_br", 2}, {"palm_bl", 3}}),
    last_tracking_palm(""),
    lastTime(0) 
{        tf_o_b(2, 3) = 0.2;    }

void Pybullet_Interface::publish_transform_odometry(double jointPositions[14], int palmStates[4],           float &pos_x, float &pos_y, float &pos_z,         float w, float x, float y, float z)
{
    


    unsigned long currentTime = millis();
    if (currentTime - lastTime >= 100)
    {
        lastTime = currentTime;

        // Serial.println("Publishing transform odometry...");
        // Add the rest of the logic similar to the original function
        // This is where you would handle matrix calculations and transformations

        std::vector<float> joint_positions;
        for (int i=0; i<12; i++)
        {
            joint_positions.push_back(jointPositions[i]);
        }

        int loop_index = 0;
        for (const auto& pair : palm_indices)
        {
            const std::string& palm_name = pair.first;
            int palm_index = pair.second;

            // get stance
            // if (is_stance[loop_index] == 1)
            if (palmStates[loop_index] == 1)
            {
                try
                {
                    // Assuming p is a global variable of type pybullet
                    if (last_tracking_palm == palm_name)
                    {
                        // Placeholder for PyBullet call
                        // get orientation from IMU
                        // std::vector<float> rotation_odom_to_body = std::vector<float>(w, x, y, z);
                        // Placeholder for FK calculation
                        Matrix4f tf_b_p = fk_objects[palm_index].calc_xyz(joint_positions[loop_index*3+0], joint_positions[loop_index*3+1], joint_positions[loop_index*3+2]); //get joint sensors
                        P_body_to_palm = tf_b_p.block<3, 1>(0, 3);

                        // Placeholder for quaternion to rotation matrix conversion
                        Matrix3f rotation_matrix_odom_to_body = Quaternion_(w, x, y, z).toMatrix3f();
                        // rotation_matrix_odom_to_body = quaternion_to_rotation_matrix(rotation_odom_to_body);

                        P_odomtery_to_body = P_odomtery_to_palm - rotation_matrix_odom_to_body * P_body_to_palm;

                        // Placeholder for translation matrix and transformation matrix multiplication
                        tf_o_b = Matrix4f::Identity(); // Replace with actual matrix operations
                        tf_o_b = to_translation_matrix(P_odomtery_to_body[0], P_odomtery_to_body[1], P_odomtery_to_body[2]) * Quaternion_(w, x, y, z).toMatrix4f();
                        
                        
                        
                        // // Placeholder for setting the transform_stamped values
                        // Serial.print(String(P_odomtery_to_body[0])  +  String("\t"));
                        // Serial.print(String(P_odomtery_to_body[1])  +  String("\t"));
                        // Serial.print(String(P_odomtery_to_body[2])  +  String("\t|\t"));
                        // Serial.print(String(w)                      +  String("\t"));
                        // Serial.print(String(x)                      +  String("\t"));
                        // Serial.print(String(y)                      +  String("\t"));
                        // Serial.println(String(z));
                        
                        
                        pos_x = P_odomtery_to_body[0];
                        pos_y = P_odomtery_to_body[1];
                        pos_z = P_odomtery_to_body[2];
                        
                    }
                    else
                    {
                        last_tracking_palm = palm_name;

                        // Placeholder for PyBullet call
                        // get orientation from IMU
                        // std::vector<float> rotation_odom_to_body = ...;
                        Matrix3f rotation_matrix_odom_to_body = Quaternion_(w, x, y, z).toMatrix3f();


                        // Placeholder for FK calculation
                        Matrix4f tf_b_p = fk_objects[palm_index].calc_xyz(joint_positions[loop_index*3+0], joint_positions[loop_index*3+1], joint_positions[loop_index*3+2]);
                        P_body_to_palm = tf_b_p.block<3, 1>(0, 3);

                        // Placeholder for matrix operations
                        Matrix4f tf_o_p = Matrix4f::Identity();
                        tf_o_p = tf_o_b * tf_b_p;
                        P_odomtery_to_palm = tf_o_p.block<3, 1>(0, 3);
                        P_odomtery_to_palm[2]=0;
                        
                        
                        
                        pos_x = P_odomtery_to_body[0];
                        pos_y = P_odomtery_to_body[1];
                        pos_z = P_odomtery_to_body[2];
                        
                        
                        // // Placeholder for setting the transform_stamped values
                        // Serial.print(String(P_odomtery_to_body[0])  +  String("\t"));
                        // Serial.print(String(P_odomtery_to_body[1])  +  String("\t"));
                        // Serial.print(String(P_odomtery_to_body[2])  +  String("\t|\t"));
                        // Serial.print(String(w)                      +  String("\t"));
                        // Serial.print(String(x)                      +  String("\t"));
                        // Serial.print(String(y)                      +  String("\t"));
                        // Serial.println(String(z));
                        
                    }

                    break;
                }
                catch (const std::exception& e)
                {
                    Serial.print("Error: ");
                    Serial.println(e.what());
                }
                break;
            }
            else
            {
                loop_index++;
            }
        }

    }












}













