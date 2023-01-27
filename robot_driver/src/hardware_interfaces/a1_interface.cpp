#include "robot_driver/hardware_interfaces/a1_interface.h"

void* update_loop(void* param)
{
    UNITREE_LEGGED_SDK::LCM *data = (UNITREE_LEGGED_SDK::LCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

A1_Interface::A1_Interface():roslcm(LOWLEVEL){

    roslcm.SubscribeState();
    
    pthread_create(&tid, NULL, update_loop, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    }
    
}

void A1_Interface::loadInterface(int argc, char** argv){
    sleep(3);
}


bool A1_Interface::send(
    const quad_msgs::LegCommandArray& last_leg_command_array_msg,
    const Eigen::VectorXd& user_tx_data){
    
    roslcm.Get(RecvLowLCM);
    ROS_WARN_STREAM("RESIVED  " << RecvLowLCM.motorState[0].q);
    RecvLowROS = ToRos(RecvLowLCM);
    printf("FL_1 position: %f\n",  RecvLowROS.motorState[FL_1].q);
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
    
    //quad_2unitree_data_transform(last_leg_command_array_msg, SendLowROS);
    SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
    roslcm.Send(SendLowLCM);

    return false;
}


bool A1_Interface::recv(
    sensor_msgs::JointState& joint_state_msg,
    sensor_msgs::Imu& imu_msg,
    Eigen::VectorXd& user_rx_data)
    {
        //std::cout << "RECVESTRECVESTRECVESTRECVESTRECVEST" <<std::endl;
        //roslcm.Recv();
        
        return false;
    }


void A1_Interface::unloadInterface() {}

void A1_Interface::unitree_2quad_data_transform(
    const quad_msgs::LegCommandArray& last_leg_command_array_msg,
    unitree_legged_msgs::LowCmd& cmd){

}

void A1_Interface::quad_2unitree_data_transform(
    const quad_msgs::LegCommandArray& last_leg_command_array_msg,
    unitree_legged_msgs::LowCmd& cmd){

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

