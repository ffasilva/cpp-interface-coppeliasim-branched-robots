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
            https://github.com/dqrobotics/cpp-interface-vrep/blob/master/src/dqrobotics/interfaces/vrep/robots/LBR4pVrepRobot.cpp
*/

#include <dmc-interface-coppeliasim/interfaces/coppeliasim/legacy_api/robots/LBR4pCoppeliaSimLegacyRobot.h>

#include<dqdynamics/robots/LBR4pRobot.h>

#include <dqrobotics/utils/DQ_Constants.h>

namespace DQ_dynamics
{

/**
 * @brief Constructor of the LBR4pCoppeliaSimLegacyRobot class
 *
 * @param robot_name The name of robot used on the CoppeliaSim scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 *
 *               Example:
 *               auto vi = std::make_shared<DQ_VrepInterface>(DQ_VrepInterface());
 *               vi->connect(19997,100,5);
 *               vi->start_simulation();
 *               LBR4pVrepRobot lbr4p_coppeliasim_robot("LBR4p", vi);
 *               auto q = lbr4p_coppeliasim_robot.get_q_from_vrep();
 *               vi->stop_simulation();
 *               vi->disconnect();
 */
LBR4pCoppeliaSimLegacyRobot::LBR4pCoppeliaSimLegacyRobot(const std::string& robot_name,
                            const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr):
    DQ_BranchedCoppeliaSimLegacyRobot("LBR4p", 7, robot_name, vrep_interface_sptr)
{
}


DQ_SerialManipulatorDH LBR4pCoppeliaSimLegacyRobot::kinematics()
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

/**
 * @brief This method constructs an instance of a DQ_SerialManipulatorDynamics.
 *
 * @return A DQ_SerialManipulatorDynamics representing a KUKA LBR+ robot.
 *
 *  Example:
 *      Recommended:
 *          auto vi = std::make_shared<DQ_VrepInterface>(DQ_VrepInterface());
 *          vi->connect(19997,100,5);
 *          vi->start_simulation();
 *          LBR4pCoppeliaSimLegacyRobot lbr4p_coppeliasim_robot("LBR4p", vi);
 *          DQ_SerialManipulatorDynamics lbr4p_robot =
 *              lbr4p_coppeliasim_robot.dynamics();
 *      Advanced:
 *          auto vi = std::make_shared<DQ_VrepInterface>(DQ_VrepInterface());
 *          vi->connect(19997,100,5);
 *          vi->start_simulation();
 *          LBR4pCoppeliaSimLegacyRobot lbr4p_coppeliasim_robot("LBR4p", vi);
 *          DQ_SerialManipulatorDynamics lbr4p_robot =
 *              lbr4p_coppeliasim_robot.dynamics('MyLuaScript');
 */
DQ_SerialManipulatorDynamics LBR4pCoppeliaSimLegacyRobot::dynamics()
{
    // Create a DQ_SerialManipulatorDynamics object
    DQ_SerialManipulatorDynamics dyn = LBR4pRobot::dynamics();

    // Set the robot configuration with CoppeliaSim values
    VectorXd q_read = this->get_configuration_space_positions();
    VectorXd dq_read = this->get_configuration_space_velocities();
    VectorXd ddq_read = VectorXd::Zero(7,1);

    dyn.set_configurations(q_read, dq_read, ddq_read);

    // Update base and reference frame with CoppeliaSim values
    dyn.set_reference_frame(this->vrep_interface_sptr_->get_object_pose(
                                                            this->base_frame_name_));
    dyn.set_base_frame(this->vrep_interface_sptr_->get_object_pose(
                                                            this->base_frame_name_));

    // Update dynamic parameters with CoppeliaSim information
    auto dyn_sptr = std::make_shared<DQ_SerialManipulatorDynamics>(dyn);
    this->update_dynamic_parameters(dyn_sptr);

    return dyn;
}

}//namespace DQ_dynamics
