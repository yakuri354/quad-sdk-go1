#ifndef A1_INTERFACE_H
#define A1_INTERFACE_H

#include <robot_driver/hardware_interfaces/hardware_interface.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
using namespace UNITREE_LEGGED_SDK;

class A1_Interface : public HardwareInterface {
    public:
        /**
         * @brief Constructor for A1_Interface
         * @return Constructed object of type A1_Interface
         */
        A1_Interface();

        /**
         * @brief Load the hardware interface
         * @param[in] argc Argument count
         * @param[in] argv Argument vector
         */
        virtual void loadInterface(int argc, char** argv);

        /**
         * @brief Unload the hardware interface
         */
        virtual void unloadInterface();

        /**
         * @brief Send commands to the robot via the mblink protocol
         * @param[in] leg_command_array_msg Message containing leg commands
         * @param[in] user_data Vector containing user data
         * @return boolean indicating success of transmission
         */
        virtual bool send(const quad_msgs::LegCommandArray& leg_command_array_msg,
                        const Eigen::VectorXd& user_tx_data);

        /**
         * @brief Recieve data from the robot via the mblink protocol
         * @param[out] joint_state_msg Message containing joint state information
         * @param[out] imu_msg Message containing imu information
         * @param[out] user_data Vector containing user data
         * @return Boolean for whether data was successfully received
         */
        virtual bool recv(sensor_msgs::JointState& joint_state_msg,
                        sensor_msgs::Imu& imu_msg, Eigen::VectorXd& user_rx_data);
    private:
        void unitree_2quad_data_transform(
            const quad_msgs::LegCommandArray& last_leg_command_array_msg,
            LowCmd& cmd);

        void quad_2unitree_data_transform(
            const quad_msgs::LegCommandArray& last_leg_command_array_msg,
            LowCmd& cmd);

        Safety safe;
        UDP udp;
        LowCmd cmd = {0};
        LowState state = {0};
        int motiontime = 0;
};


#endif  // A1_INTERFACE_H