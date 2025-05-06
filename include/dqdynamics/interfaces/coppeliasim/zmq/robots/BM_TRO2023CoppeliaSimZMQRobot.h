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

#include <dqdynamics/interfaces/coppeliasim/zmq/DQ_BranchedCoppeliaSimZMQRobot.h>

#include <dqdynamics/robot_modeling/DQ_BranchedWholeBody.h>

#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>

namespace DQ_dynamics
{

class BM_TRO2023CoppeliaSimZMQRobot: public DQ_BranchedCoppeliaSimZMQRobot
{
public:
    BM_TRO2023CoppeliaSimZMQRobot() = delete;
    BM_TRO2023CoppeliaSimZMQRobot(const std::string& robot_name,
        const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental>& vrep_interface_sptr);
    virtual ~BM_TRO2023CoppeliaSimZMQRobot() = default;

    DQ_BranchedWholeBody dynamics();
};

}//namespace DQ_dynamics



