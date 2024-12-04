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

#include <dqrobotics/interfaces/vrep/DQ_SerialVrepRobot.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>

namespace DQ_dynamics
{

class LBR4pCoppeliaSimLegacyRobot: public DQ_SerialVrepRobot
{
public:
    // LBR4pCoppeliaSimLegacyRobot() = delete;
    LBR4pCoppeliaSimLegacyRobot(const std::string& robot_name,
                           const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr);
    // virtual ~LBR4pCoppeliaSimLegacyRobot() = default;
    // ~LBR4pCoppeliaSimLegacyRobot();

    DQ_SerialManipulatorDH kinematics();
};

}//namespace DQ_dynamics



