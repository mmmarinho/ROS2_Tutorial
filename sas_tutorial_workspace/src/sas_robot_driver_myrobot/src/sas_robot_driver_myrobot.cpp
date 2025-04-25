/*
# Copyright (c) 2016-2025 Murilo Marques Marinho
#
#    This file is part of sas_robot_driver_myrobot.
#
#    sas_robot_driver_myrobot is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_myrobot is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_myrobot.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#   Based on sas_robot_driver_ur.cpp
#
# ################################################################*/


#include "sas_robot_driver_myrobot/sas_robot_driver_myrobot.hpp"
#include <iostream>
#include <memory>
#include <sas_core/eigen3_std_conversions.hpp>

namespace sas
{


class RobotDriverMyrobot::Impl
{

public:
    bool connected{false};
    bool motor_on{false};

    VectorXd joint_positions_;


    Impl()
        {

        }

};

RobotDriverMyrobot::RobotDriverMyrobot(const RobotDriverMyrobotConfiguration& configuration, std::atomic_bool* break_loops):
    RobotDriver(break_loops),
    configuration_(configuration)
{
    joint_limits_ = configuration.joint_limits; //for the superclass
    impl_ = std::make_unique<RobotDriverMyrobot::Impl>();
}

RobotDriverMyrobot::~RobotDriverMyrobot()
{

}

/**
 * @brief RobotDriverMyrobot::get_joint_positions
 * This method should always throw an exception if the user
 * tries to obtain the joint positions in an invalid state.
 *
 * One useful way of defining that is with a VectorXd(), which
 * has by default size zero until it is initialized.
 *
 * @return a VectorXd representing the configuration space in radians.
 */
VectorXd RobotDriverMyrobot::get_joint_positions()
{
    if(impl_->joint_positions_.size()==0)
        throw std::runtime_error("Tried to obtain invalid joint positions");

    return impl_->joint_positions_;
}

/**
 * @brief RobotDriverMyrobot::set_target_joint_positions
 * This method expects the desired joint positions in radians. The most basic
 * check is for the correct
 *
 * @param desired_joint_positions_rad
 */
void RobotDriverMyrobot::set_target_joint_positions(const VectorXd &desired_joint_positions_rad)
{
    if(desired_joint_positions_rad.size() != 6)
        throw std::runtime_error("Incorrect vector size in RobotDriverMyrobot::set_target_joint_positions");

    impl_->joint_positions_ = desired_joint_positions_rad;
}

/**
 * @brief RobotDriverMyrobot::connect
 *
 * Usually this method will connect to a given ip address. It is also common
 * for this part of the code to stop running programs or turn the robot off.
 * This function is expected to throw an exception of something goes wrong.
 * For instance, if the connection is not established an exception MUST
 * be thrown.
 */
void RobotDriverMyrobot::connect()
{
    //An example of exception to throw.
    if(impl_->connected)
        throw std::runtime_error("Already connected.");

    impl_->connected = true;

    impl_->motor_on = false;

    //Usually after the connection is established we can read joint positions
    //but not all drivers work like this
    impl_->joint_positions_ = (VectorXd(6) << 0, 0, 0, 0, 0, 0).finished();
}

/**
 * @brief RobotDriverMyrobot::initialize
 *
 * This method is expected to turn the robot on and initialize the internal joint control loop.
 * If there are any issues, this MUST throw an exception so that the program will finish
 * cleanly.
 *
 * After this method finishes target joint states can be received.
 */
void RobotDriverMyrobot::initialize()
{
    impl_->motor_on = true;


}

/**
 * @brief RobotDriverMyrobot::deinitialize.
 * For safety reasons, this MUST NOT throw exceptions.
 */
void RobotDriverMyrobot::deinitialize()
{
    impl_->motor_on = false;

}

/**
 * @brief RobotDriverMyrobot::disconnect
 * For safety reasons, this MUST NOT throw exceptions.
 */
void RobotDriverMyrobot::disconnect()
{
    impl_->connected = false;
    impl_->joint_positions_ = VectorXd();
}

}
