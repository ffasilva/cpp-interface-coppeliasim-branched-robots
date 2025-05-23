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
*/

#pragma once

#include <dqdynamics/robot_modeling/DQ_Dynamics.h>
#include <dqdynamics/robot_modeling/DQ_SerialManipulatorDynamics.h>
#include <dqdynamics/robot_modeling/DQ_BranchedWholeBody.h>

#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobotZMQ.h>

#include<memory>

namespace DQ_dynamics
{

class DQ_BranchedCoppeliaSimZMQRobot: public DQ_robotics::DQ_CoppeliaSimRobotZMQ
{
protected:
    std::vector<std::string> link_names_;
    VectorXd joint_armatures_;

    void update_base_dynamic_parameters(DQ_Dynamics& robot_dynamics);
    void update_branch_dynamic_parameters(DQ_Dynamics& robot_dynamics,
                                          const int&starting_name_index);
    void update_dynamic_parameters(DQ_SerialManipulatorDynamics& robot_dynamics,
                                   const int&starting_name_index = 0);
    void update_dynamic_parameters(DQ_BranchedWholeBody& robot_dynamics,
                                   const int&starting_name_index = 0);
public:
    DQ_BranchedCoppeliaSimZMQRobot() = delete;
    DQ_BranchedCoppeliaSimZMQRobot(const std::string& robot_name,
        const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental>& interface_sptr);
    virtual ~DQ_BranchedCoppeliaSimZMQRobot() = default;

    std::vector<std::string> get_link_names() const;

    void set_joint_operation_modes(
        const DQ_CoppeliaSimInterfaceZMQExperimental::JOINT_MODE& joint_mode,
        const DQ_CoppeliaSimInterfaceZMQExperimental::JOINT_CONTROL_MODE& joint_control_mode);

    void set_joint_armatures(const double& joint_armatures);
    void set_joint_dampings(const int& joint_dampings);
    void set_link_frictions(const std::vector<double>& link_frictions);

    VectorXd remove_joint_armature_effects(const VectorXd& tau,
                                           const VectorXd& ddq);
};

}//namespace DQ_dynamics



