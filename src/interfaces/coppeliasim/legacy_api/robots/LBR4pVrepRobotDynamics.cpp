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

#include <dmc-interface-coppeliasim/interfaces/coppeliasim/legacy_api/robots/LBR4pVrepRobotDynamics.h>

#include <dqrobotics/utils/DQ_Constants.h>

namespace DQ_dynamics
{

/**
 * @brief Constructor of the LBR4pVrepRobotDynamics class
 *
 * @param robot_name The name of robot used on the vrep scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 *
 *               Example:
 *               auto vi = std::make_shared<DQ_VrepInterface>(DQ_VrepInterface());
 *               vi->connect(19997,100,5);
 *               vi->start_simulation();
 *               LBR4pVrepRobot lbr4p_vreprobot("LBR4p", vi);
 *               auto q = lbr4p_vreprobot.get_q_from_vrep();
 *               vi->stop_simulation();
 *               vi->disconnect();
 */
LBR4pVrepRobotDynamics::LBR4pVrepRobotDynamics(const std::string& robot_name,
                            const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr):
    DQ_SerialVrepRobot("LBR4p",
                       7,
                       robot_name,
                       vrep_interface_sptr)
{

}


DQ_SerialManipulatorDH LBR4pVrepRobotDynamics::kinematics()
{
    const double pi2 = pi/2.0;

    Matrix<double,5,7> dh(5,7);
    dh <<  0,     0,     0,   0,   0,    0,   0,
            0.200, 0,     0.4, 0,   0.39, 0,   0,
            0,     0,     0,   0,   0,    0,   0,
            pi2,   -pi2,  pi2,-pi2, pi2, -pi2, 0,
            0, 0, 0, 0, 0, 0, 0;
    DQ_SerialManipulatorDH kin(dh);
    kin.set_reference_frame(_get_interface_ptr()->get_object_pose(base_frame_name_));
    kin.set_base_frame(_get_interface_ptr()->get_object_pose(base_frame_name_));
    kin.set_effector(1+0.5*E_*k_*0.07);
    return kin;
}

}//namespace DQ_dynamics
