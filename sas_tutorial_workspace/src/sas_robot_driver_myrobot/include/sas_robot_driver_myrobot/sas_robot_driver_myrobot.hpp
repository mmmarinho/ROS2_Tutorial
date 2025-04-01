#pragma once
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
#   Based on sas_robot_driver_ur.hpp
#
# ################################################################*/

#include <atomic>
#include <memory>
#include <sas_core/sas_robot_driver.hpp>

using namespace Eigen;

namespace sas
{

struct RobotDriverMyrobotConfiguration
{
    std::string ip;
    std::tuple<VectorXd,VectorXd> joint_limits;
};

class RobotDriverMyrobot: public RobotDriver
{
private:
    RobotDriverMyrobotConfiguration configuration_;

    //Use the Impl idiom to "hide" internal driver sources
    class Impl;
    std::unique_ptr<Impl> impl_;
public:

    // Prevent copies as usually drivers have threads
    RobotDriverMyrobot(const RobotDriverMyrobot&)=delete;
    RobotDriverMyrobot()=delete;
    ~RobotDriverMyrobot();

    // This boilderplate constructor usually does the job well and prevent big changes when
    // parameters change
    RobotDriverMyrobot(const RobotDriverMyrobotConfiguration &configuration, std::atomic_bool* break_loops);

    /// Everything below this line is an override
    /// the concrete implementations are needed
    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

};
}
