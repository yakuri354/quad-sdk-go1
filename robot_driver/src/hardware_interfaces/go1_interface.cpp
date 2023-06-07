#include "robot_driver/hardware_interfaces/go1_interface.h"

// LegCommandArray: LegCommand[] leg_commands # FL, BL, FR, BR
// LegCommand: MotorCommand[] motor_commands # Stored as Abd, Hip, Knee -- 0, 1, 2 in Unitree SDK

template <typename K, typename V>
static std::map<V, K> reverse_map(const std::map<K, V> &m)
{
    std::map<V, K> r;
    for (const auto &kv : m)
        r[kv.second] = kv.first;
    return r;
}

Go1Interface::Go1Interface(ros::NodeHandle nh) :
    HardwareInterface(nh),
    udp(8080, "192.168.123.10", 8007, LOW_CMD_LENGTH, LOW_STATE_LENGTH, false, RecvEnum::block) {

  udp.InitCmdData(default_cmd);

  uni2quad = reverse_map(quad2uni);

  // Init the default LowCmd
  default_cmd.head = {0xFE, 0xEF};
  default_cmd.levelFlag = LOWLEVEL;

  // Copied from Unitree SDK:
  for (int i = 0; i < 12; i++) {
      default_cmd.motorCmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
      default_cmd.motorCmd[i].q = 0;
      default_cmd.motorCmd[i].Kp = 0;
      default_cmd.motorCmd[i].dq = 0;
      default_cmd.motorCmd[i].Kd = 0;
      default_cmd.motorCmd[i].tau = 0;
  }

  // Straighten the legs relative to the body

  default_cmd.motorCmd[FR_0].tau = -0.65f;
  default_cmd.motorCmd[FL_0].tau = +0.65f;
  default_cmd.motorCmd[RR_0].tau = -0.65f;
  default_cmd.motorCmd[RL_0].tau = +0.65f;
}

void Go1Interface::loadInterface(int argc, char **argv) {
  for (int i = 0; i < 10; i++) {
    udp.SetSend(default_cmd);
    udp.Send();
    usleep(20'000);
  }
}

std::map<int, string> dbg = {
     {FL_0, "FL_0"},
     {FL_1, "FL_1"},
     {FL_2, "FL_2"},
     {FR_0, "FR_0"},
     {FR_1, "FR_1"},
     {FR_2, "FR_2"},
     {RL_0, "RL_0"},
     {RL_1, "RL_1"},
     {RL_2, "RL_2"},
     {RR_0, "RR_0"},
     {RR_1, "RR_1"},
     {RR_2, "RR_2"},
};

std::map<int, float> offset_q2u = {
     {FL_0, 0},
     {FL_1, +MATH_PI/2},
     {FL_2, -2.8},
     {FR_0, 0},
     {FR_1, +MATH_PI/2},
     {FR_2, -2.8},
     {RL_0, 0},
     {RL_1, +MATH_PI/2},
     {RL_2, -2.8},
     {RR_0, 0},
     {RR_1, +MATH_PI/2},
     {RR_2, -2.8},
};

std::map<int, float> sign_q2u = {
     {FL_0, 1},
     {FL_1, -1},
     {FL_2, 1},
     {FR_0, 1},
     {FR_1, -1},
     {FR_2, 1},
     {RL_0, -1},
     {RL_1, -1},
     {RL_2, 1},
     {RR_0, -1},
     {RR_1, -1},
     {RR_2, 1},
};

// #define SEND_DBG

bool Go1Interface::send(const quad_msgs::LegCommandArray &leg_command_array_msg, const Eigen::VectorXd &user_tx_data) {
  LowCmd cmd = default_cmd;

  // FL, BL, FR, BR
  // Abd, Hip, Knee
  int quad_to_unitree[4][3] = {
    {FL_0, FL_1, FL_2},
    {RL_0, RL_1, RL_2},
    {FR_0, FR_1, FR_2},
    {RR_0, RR_1, RR_2},
  };

  #ifdef SEND_DBG
  cout.clear();
  cerr.clear();

  cerr << "\e[1;1H\e[2J" << flush;
  cerr << fixed << setprecision(4) << setfill(' ');

  LowState state;
  udp.Recv();
  udp.GetRecv(state);
  #endif

  for (int leg = 0; leg < 4; leg++) { // FL, BL, FR, BR
    for (int joint = 0; joint < 3; joint++) { // Abd, Hip, Knee
      const auto uni_ix = quad_to_unitree[leg][joint];
      const auto &quad_cmd = leg_command_array_msg.leg_commands[leg].motor_commands[joint];
      auto &uni_cmd = cmd.motorCmd[uni_ix];

      #ifdef SEND_DBG
      cerr << "At " << dbg[uni_ix] << ' ';

      cerr << "q " << setw(7) << quad_cmd.pos_setpoint << ' ';
      cerr << "dq " << setw(7) << quad_cmd.vel_setpoint << ' ';
      cerr << "tau " << setw(7) << quad_cmd.torque_ff << ' ';
      cerr << "kd " << setw(7) << quad_cmd.kd << ' ';
      cerr << "kp " << setw(7) << quad_cmd.kp << ' ';
      #endif

      uni_cmd.q = offset_q2u[uni_ix] + quad_cmd.pos_setpoint  * sign_q2u[uni_ix];
      uni_cmd.dq = quad_cmd.vel_setpoint * sign_q2u[uni_ix];
      uni_cmd.tau = quad_cmd.torque_ff * sign_q2u[uni_ix];
      uni_cmd.Kp = quad_cmd.kp;
      uni_cmd.Kd = quad_cmd.kd;

      #ifdef SEND_DBG
      cerr << " ---> ";

      cerr << "q " << setw(7) << uni_cmd.q << ' ';
      cerr << "dq " << setw(7) << uni_cmd.dq << ' ';
      cerr << "tau " << setw(7) << uni_cmd.tau << ' ';
      cerr << "kd " << setw(7) << uni_cmd.Kp << ' ';
      cerr << "kp " << setw(7) << uni_cmd.Kd << ' ';

      cerr << "    [ ";

      cerr << "q " << setw(7) << state.motorState[uni_ix].q << ' ';
      cerr << "dq " << setw(7) << state.motorState[uni_ix].dq << ' ';
      cerr << "tau " << setw(7) << state.motorState[uni_ix].tauEst << ' ';

      // cerr << " ] ---> [ ";
      //
      // cerr << "q " << setw(7) << (state.motorState[uni_ix].q - offset_q2u[uni_ix]) * sign_q2u[uni_ix] << ' ';
      // cerr << "dq " << setw(7) << state.motorState[uni_ix].dq * sign_q2u[uni_ix] << ' ';
      // cerr << "tau " << setw(7) << state.motorState[uni_ix].tauEst * sign_q2u[uni_ix ] << ' ';
      
      cerr << "]\n" << endl;

      #endif
    }
  }

  cout.setstate(std::ios_base::failbit);
  cerr.setstate(std::ios_base::failbit);

  udp.SetSend(cmd);
  udp.Send();

  // cerr << "Sent " << leg_command_array_msg << endl << endl;

  return true;
}

// #define RECV_DBG

bool Go1Interface::recv(sensor_msgs::JointState &joint_state_msg, sensor_msgs::Imu &imu_msg, Eigen::VectorXd &user_rx_data) {
  LowState state;
  udp.Recv(); // Blocking
  udp.GetRecv(state);

#ifdef RECV_DBG
    ios::sync_with_stdio(false);
    cerr.tie(nullptr);

    cerr << "Recv: " << endl;
#endif

  std::map<int, string> dbg = {
       {FL_0, "FL_0"},
       {FL_1, "FL_1"},
       {FL_2, "FL_2"},
       {FR_0, "FR_0"},
       {FR_1, "FR_1"},
       {FR_2, "FR_2"},
       {RL_0, "RL_0"},
       {RL_1, "RL_1"},
       {RL_2, "RL_2"},
       {RR_0, "RR_0"},
       {RR_1, "RR_1"},
       {RR_2, "RR_2"},
  };

#ifdef RECV_DBG
  cout.clear();
  cerr.clear();

  cerr << "\e[1;1H\e[2J" << flush;
  cerr << fixed << setprecision(4) << setfill(' ');

  for (int i = 0; i < 12; i++) {
    cerr << "Motor " << i << " --> " << dbg[i] << endl;
    cerr << "q " << setw(4) << state.motorState[i].q << ' ';
    cerr << "dq " << setw(4) << state.motorState[i].dq << ' ';
    cerr << "tau " << setw(4) << state.motorState[i].tauEst << ' ';
    cerr << '\n' << endl;
  }

  cout.setstate(std::ios_base::failbit);
  cerr.setstate(std::ios_base::failbit);
#endif

  if (state.head[0] == 0) {
    ROS_ERROR("Null state received");
    return false;
  }
  // Translate IMU data
  
  tf2::Quaternion quat;
  quat.setRPY(state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]);

  imu_msg.orientation.w = state.imu.quaternion[0];
  imu_msg.orientation.x = state.imu.quaternion[1];
  imu_msg.orientation.y = state.imu.quaternion[2];
  imu_msg.orientation.z = state.imu.quaternion[3];

  imu_msg.linear_acceleration.x = state.imu.accelerometer[0];
  imu_msg.linear_acceleration.y = state.imu.accelerometer[1];
  imu_msg.linear_acceleration.z = state.imu.accelerometer[2];

  imu_msg.angular_velocity.x = state.imu.gyroscope[0];
  imu_msg.angular_velocity.y = state.imu.gyroscope[1];
  imu_msg.angular_velocity.z = state.imu.gyroscope[2];

  // Translate joint data
  
  for (int i = 0; i < 12; i++) {
    joint_state_msg.name[i] = std::to_string(i);
    joint_state_msg.position[i] = (state.motorState[quad2uni[i]].q - offset_q2u[quad2uni[i]]) * sign_q2u[quad2uni[i]];
    joint_state_msg.velocity[i] = state.motorState[quad2uni[i]].dq * sign_q2u[quad2uni[i]];
    joint_state_msg.effort[i] = state.motorState[quad2uni[i]].tauEst * sign_q2u[quad2uni[i]];
  }

  return true;
}

void Go1Interface::unloadInterface() {}
