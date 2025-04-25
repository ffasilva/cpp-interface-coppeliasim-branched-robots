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

#include <dqdynamics/interfaces/coppeliasim/zmq/robots/FreeFlyingCoppeliaSimZMQRobot.h>

// #include <dqrobotics/utils/DQ_Constants.h>

namespace DQ_dynamics
{

/**
 * @brief Constructor of the FreeFlyingCoppeliaSimZMQRobot class
 *
 * @param robot_name The name of robot used on the CoppeliaSim scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 *
 *  Example:
 *          auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *          vi->connect();
 *          vi->start_simulation();
 *          vi->load_from_model_browser("/robots/non-mobile/KUKA LBR4+.ttm",
 *                                      "/LBR4p");
 *          FreeFlyingCoppeliaSimZMQRobot lbr4p_coppeliasim_robot("/LBR4p", vi);
 *          auto q = lbr4p_coppeliasim_robot.get_joint_positions();
 *          vi->stop_simulation();
 */
FreeFlyingCoppeliaSimZMQRobot::FreeFlyingCoppeliaSimZMQRobot(
    const std::string& robot_name,
    const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental>& vrep_interface_sptr):
    DQ_BranchedCoppeliaSimZMQRobot(robot_name, vrep_interface_sptr)
{
}

/**
 * @brief This method constructs an instance of a DQ_FreeFlyingRobotDynamics.
 *
 * @return A DQ_FreeFlyingRobotDynamics representing a free-flying robot.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      FreeFlyingCoppeliaSimZMQRobot free_flying_coppeliasim_robot(
 *          "/free_flying",
 *          vi);
 *      DQ_FreeFlyingRobotDynamics free_flying_robot =
 *          free_flying_coppeliasim_robot.dynamics();
 *      vi->stop_simulation();
 */
DQ_FreeFlyingRobotDynamics FreeFlyingCoppeliaSimZMQRobot::dynamics()
{
    // Create a DQ_FreeFlyingRobotDynamics object
    DQ_FreeFlyingRobotDynamics dyn = FreeFlyingRobotDynamics::dynamics();

    // // Set the robot configuration with CoppeliaSim values
    // VectorXd q_read = this->get_configuration_space();
    // VectorXd dq_read = this->get_configuration_space_velocities();
    // VectorXd ddq_read = VectorXd::Zero(7,1);

    // dyn.set_configurations(q_read, dq_read, ddq_read);

    // // Update base and reference frame with CoppeliaSim values
    // const DQ& base_frame = this->_get_interface_sptr()->get_object_pose(
    //     this->base_frame_name_);
    // dyn.set_reference_frame(base_frame);
    // dyn.set_base_frame(base_frame);

    // // Update dynamic parameters with CoppeliaSim information
    // update_dynamic_parameters(dyn);

    return dyn;
}

}//namespace DQ_dynamics
