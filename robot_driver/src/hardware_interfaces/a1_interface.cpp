#include "robot_driver/hardware_interfaces/a1_interface.h"


using namespace UNITREE_LEGGED_SDK;

A1_Interface::A1_Interface(): safe(LeggedType::A1), udp(LOWLEVEL){
        udp.InitCmdData(cmd);
}


void A1_Interface::loadInterface(int argc, char** argv){}


bool A1_Interface::send(
    const quad_msgs::LegCommandArray& last_leg_command_array_msg,
    const Eigen::VectorXd& user_tx_data){
    
    udp.GetRecv(state);
    quad_2unitree_data_transform(last_leg_command_array_msg, cmd);
    safe.PowerProtect(cmd, state, 1);
    udp.SetSend(cmd);
    udp.Send();
    return true;
}


bool A1_Interface::recv(
    sensor_msgs::JointState& joint_state_msg,
    sensor_msgs::Imu& imu_msg,
    Eigen::VectorXd& user_rx_data)
    {
        udp.Recv();
        return true;
    }


void A1_Interface::unloadInterface() {}

void A1_Interface::unitree_2quad_data_transform(
    const quad_msgs::LegCommandArray& last_leg_command_array_msg,
    LowCmd& cmd){

}

void A1_Interface::quad_2unitree_data_transform(
    const quad_msgs::LegCommandArray& last_leg_command_array_msg,
    LowCmd& cmd){

    motiontime++;

    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    if( motiontime >= 500){
        float torque = (0 - state.motorState[FR_1].q)*10.0f + (0 - state.motorState[FR_1].dq)*1.0f;
        if(torque > 5.0f) torque = 5.0f;
        if(torque < -5.0f) torque = -5.0f;

        cmd.motorCmd[FR_1].q = PosStopF;
        cmd.motorCmd[FR_1].dq = VelStopF;
        cmd.motorCmd[FR_1].Kp = 0;
        cmd.motorCmd[FR_1].Kd = 0;
        cmd.motorCmd[FR_1].tau = torque;
    }
}