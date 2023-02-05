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
    fill_testLCMmsg_sample(RecvLowLCM);
    // Converting to ROS msg
    RecvLowROS = ToRos(RecvLowLCM);


    std::vector<std::string> joint_names_ = {"8",  "0", "1", "9",  "2", "3",
                                           "10", "4", "5", "11", "6", "7"};
    // Converting unitree joints positions and joints velocities
    for (int i = 0; i < joint_names_.size(); ++i)
    {
        joint_state_msg.name[i] = joint_names_[i];
        joint_state_msg.position[i] = RecvLowROS.motorState[quad2uni[i]].q;
        joint_state_msg.velocity[i] = RecvLowROS.motorState[quad2uni[i]].dq;
        joint_state_msg.effort[i] = RecvLowLCM.motorState[quad2uni[i]].tauEst; // No need extra converstion because it already done on robot side
    }
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

void A1_Interface::unitree_2quad_data_transform(
    const quad_msgs::LegCommandArray &last_leg_command_array_msg,
    unitree_legged_msgs::LowCmd &cmd)
{
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

