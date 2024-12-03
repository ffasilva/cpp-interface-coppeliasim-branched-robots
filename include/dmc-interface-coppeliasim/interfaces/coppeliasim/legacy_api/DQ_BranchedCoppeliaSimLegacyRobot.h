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

#include "dqdynamics/robot_modeling/DQ_BranchedWholeBody.h"
#include <dqdynamics/robot_modeling/DQ_Dynamics.h>

#include <dqrobotics/interfaces/vrep/DQ_SerialVrepRobot.h>

namespace DQ_dynamics
{

class DQ_BranchedCoppeliaSimLegacyRobot: public DQ_SerialVrepRobot
{
protected:
    std::vector<std::string> link_names_;

    // void update_base_dynamic_parameters(std::shared_ptr<DQ_Dynamics> robot_dynamics);
    // void update_branch_dynamic_parameters(std::shared_ptr<DQ_Dynamics> robot_dynamics,
    //                                       const int&starting_name_index);
    // void update_dynamic_parameters(std::shared_ptr<DQ_BranchedWholeBody> robot_dynamics,
    //                                const int&starting_name_index = 1);
public:
    // DQ_BranchedCoppeliaSimLegacyRobot() = delete;
    DQ_BranchedCoppeliaSimLegacyRobot(const std::string& base_robot_name,
                            const int& robot_dof,
                            const std::string& robot_name,
                            const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr);
    // virtual ~DQ_BranchedCoppeliaSimLegacyRobot() = default;
    // ~DQ_BranchedCoppeliaSimLegacyRobot();
};

}//namespace DQ_dynamics



