/**
(C) Copyright 2024 DQ Dynamics Developers

This file is part of DQ Dynamics.

    DQ Dynamics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Dynamics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Dynamics.  If not, see <http://www.gnu.org/licenses/>.

Contributors to this file:
    Frederico Fernandes Afonso Silva - frederico.silva@ieee.org
        - Adapted from DQ Robotic's LBR4pVrepRobot class.
            https://github.com/dqrobotics/cpp-interface-vrep/blob/master/include/dqrobotics/interfaces/vrep/robots/LBR4pVrepRobot.h
*/

#pragma once

#include <dqdynamics/interfaces/coppeliasim/zmq/DQ_BranchedCoppeliaSimZMQRobot.h>

#include <dqdynamics/robot_modeling/DQ_FreeFlyingRobotDynamics.h>

namespace DQ_dynamics
{

class FreeFlyingCoppeliaSimZMQRobot: public DQ_BranchedCoppeliaSimZMQRobot
{
protected:
    std::string force_sensor_name_;
public:
    FreeFlyingCoppeliaSimZMQRobot() = delete;
    FreeFlyingCoppeliaSimZMQRobot(const std::string& robot_name,
        const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental>&
                                      vrep_interface_sptr);
    virtual ~FreeFlyingCoppeliaSimZMQRobot() = default;

    // void set_configuration_space(const VectorXd& q) override;
    VectorXd get_configuration_space() override;

    // void set_target_configuration_space(const VectorXd& q_target) override;

    VectorXd get_configuration_space_velocities() override;
    // void set_target_configuration_space_velocities(const VectorXd& v_target) override;

    // void set_configuration_space_torques(const VectorXd& t) override;
    VectorXd get_configuration_space_torques() override;

    DQ_FreeFlyingRobotDynamics dynamics();
};

}//namespace DQ_dynamics



