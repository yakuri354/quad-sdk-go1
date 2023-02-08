#include "robot_driver/hardware_interfaces/a1_interface.h"

/// Given a map from keys to values, creates a new map from values to keys

void fill_testLCMmsg_sample(LowState& msg);

template <typename K, typename V>
static std::map<V, K> reverse_map(const std::map<K, V> &m)
{
    std::map<V, K> r;
    for (const auto &kv : m)
        r[kv.second] = kv.first;
    return r;
}

void *update_loop(void *param)
{
    UNITREE_LEGGED_SDK::LCM *data = (UNITREE_LEGGED_SDK::LCM *)param;
    while (ros::ok)
    {
        data->Recv();
        usleep(2000);
    }
}

A1_Interface::A1_Interface() : roslcm(LOWLEVEL)
{

    roslcm.SubscribeState();
    uni2quad = reverse_map(quad2uni);
    pthread_create(&tid, NULL, update_loop, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for (int i = 0; i < 12; i++)
    {
        SendLowROS.motorCmd[i].mode = 0x0A; // motor switch to servo (PMSM) mode
    }
    uni2quad = reverse_map(quad2uni);
}

void A1_Interface::loadInterface(int argc, char **argv)
{
    sleep(3);
}

bool A1_Interface::send(
    const quad_msgs::LegCommandArray &last_leg_command_array_msg,
    const Eigen::VectorXd &user_tx_data)
{

    roslcm.Get(RecvLowLCM);
    
    RecvLowROS = ToRos(RecvLowLCM);
    
    SendLowROS.motorCmd[FR_0].tau = -0.65f;
    SendLowROS.motorCmd[FL_0].tau = +0.65f;
    SendLowROS.motorCmd[RR_0].tau = -0.65f;
    SendLowROS.motorCmd[RL_0].tau = +0.65f;

    float torque = 0.0f;

    SendLowROS.motorCmd[FL_1].q = PosStopF;
    SendLowROS.motorCmd[FL_1].dq = VelStopF;
    SendLowROS.motorCmd[FL_1].Kp = 0;
    SendLowROS.motorCmd[FL_1].Kd = 0;
    SendLowROS.motorCmd[FL_1].tau = torque;

    // quad_2unitree_data_transform(last_leg_command_array_msg, SendLowROS);
    SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
    roslcm.Send(SendLowLCM);

    return false;
}

bool A1_Interface::recv(
    sensor_msgs::JointState &joint_state_msg,
    sensor_msgs::Imu &imu_msg,
    Eigen::VectorXd &user_rx_data)
{
    // Getting LCM state msg
    roslcm.Get(RecvLowLCM);
    // Converting to ROS msg
    RecvLowROS = ToRos(RecvLowLCM);

    // Converting unitree joints positions and joints velocities

  

    uni2quad_data_transform(RecvLowROS, joint_state_msg);

    // Quaternion
    geometry_msgs::Quaternion orientation_msg;
    tf2::Quaternion quat_tf;
    Eigen::Vector3f rpy;
    quat_tf.setRPY(
        RecvLowLCM.imu.rpy[0],
        RecvLowLCM.imu.rpy[1],
        RecvLowLCM.imu.rpy[2]);
    tf2::convert(quat_tf, orientation_msg);
    imu_msg.orientation = orientation_msg;
    // // Quaternion
    // imu_msg.orientation.x = RecvLowLCM.imu.quaternion[1];
    // imu_msg.orientation.y = RecvLowLCM.imu.quaternion[2];
    // imu_msg.orientation.z = RecvLowLCM.imu.quaternion[3];
    // imu_msg.orientation.w = RecvLowLCM.imu.quaternion[0];
    // imu_msg.header.frame_id = "body";

    // Angular velocities from gyroscope
    imu_msg.angular_velocity.x = RecvLowLCM.imu.gyroscope[0];
    imu_msg.angular_velocity.y = RecvLowLCM.imu.gyroscope[1];
    imu_msg.angular_velocity.z = RecvLowLCM.imu.gyroscope[2];
    // Linear velocities form acceleremeter
    imu_msg.linear_acceleration.x = RecvLowLCM.imu.accelerometer[0];
    imu_msg.linear_acceleration.y = RecvLowLCM.imu.accelerometer[1];
    imu_msg.linear_acceleration.z = RecvLowLCM.imu.accelerometer[2];

    return true;
}

void A1_Interface::unloadInterface() {}

/* Returns vector with position, velocity and effort in quad frame*/
std::tuple<float, float, float> A1_Interface::convert_joint_state_uni2quad(
    const int quad_motor_idx,
    const unitree_legged_msgs::MotorState &motor_state_msg)
{
    int sign;
    float init_pose;
    if (quad_motor_idx < 8)
    {
        if (quad_motor_idx % 2 == 0)
        {
            sign = -1;
            init_pose = M_PI / 2;
        }
        else
        {
            sign = 1;
            init_pose = M_PI;
        }
    }
    else
    {
        sign = 1;
        init_pose = 0.;
    }
    float pose = sign * motor_state_msg.q + init_pose;
    float speed = sign * motor_state_msg.dq;
    float torgue = sign * motor_state_msg.tauEst;
    return std::tuple<float, float, float>{pose, speed, torgue};
}

void A1_Interface::uni2quad_data_transform(
    const unitree_legged_msgs::LowState &low_state_msg,
    sensor_msgs::JointState &joint_state_msg)
{
    for (int i = 0; i < 12; ++i)
    {
        joint_state_msg.name[i] = std::to_string(i);
        std::tie(joint_state_msg.position[i], joint_state_msg.velocity[i], joint_state_msg.effort[i]) = convert_joint_state_uni2quad(i, low_state_msg.motorState[quad2uni[i]]);
        // std::tie(joint_state_msg.position[i], joint_state_msg.velocity[i], joint_state_msg.effort[i]) = convert_joint_state_uni2quad(i, RecvLowROS.motorState[quad2uni[i]]);
    }
}

void A1_Interface::quad_2unitree_data_transform(
    const quad_msgs::LegCommandArray &last_leg_command_array_msg,
    unitree_legged_msgs::LowCmd &cmd)
{

    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    float torque = 1.0f;

    cmd.motorCmd[FL_1].q = PosStopF;
    cmd.motorCmd[FL_1].dq = VelStopF;
    cmd.motorCmd[FL_1].Kp = 0;
    cmd.motorCmd[FL_1].Kd = 0;
    cmd.motorCmd[FL_1].tau = torque;
}

void fill_testLCMmsg_sample(LowState& msg){
    float dx = 0.1;
    Eigen::VectorXd stand_joint_angles{3};
    stand_joint_angles << 0.0, 0.76, 1.52;
    for(int i =0; i < 12; i++){
        msg.motorState[i].q = stand_joint_angles[i%3];
        msg.motorState[i].dq = 0;
        msg.motorState[i].tauEst = 0;
    }
    Eigen::VectorXd rpy(3);
    rpy << 0,0,0;
    for (int i = 0; i < 3; i++)
        msg.imu.rpy[i] = rpy[i];
    
    for (int i = 0; i < 3; i++){
        msg.imu.gyroscope[i] = 0;
        msg.imu.accelerometer[i] = 0;
    }
}

