/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/IMU.h>


#include "unitree_legged_sdk/unitree_legged_sdk.h"

unitree_legged_msgs::IMU ToRos(UNITREE_LEGGED_SDK::IMU& lcm);

unitree_legged_msgs::MotorState ToRos(UNITREE_LEGGED_SDK::MotorState& lcm);

UNITREE_LEGGED_SDK::MotorCmd ToLcm(unitree_legged_msgs::MotorCmd& ros, UNITREE_LEGGED_SDK::MotorCmd lcmType);

unitree_legged_msgs::LowState ToRos(UNITREE_LEGGED_SDK::LowState& lcm);

UNITREE_LEGGED_SDK::LowCmd ToLcm(unitree_legged_msgs::LowCmd& ros, UNITREE_LEGGED_SDK::LowCmd lcmType);

unitree_legged_msgs::HighState ToRos(UNITREE_LEGGED_SDK::HighState& lcm);

UNITREE_LEGGED_SDK::HighCmd ToLcm(unitree_legged_msgs::HighCmd& ros, UNITREE_LEGGED_SDK::HighCmd lcmType);


#endif  // _CONVERT_H_