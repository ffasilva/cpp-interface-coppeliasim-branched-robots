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

#include <dqdynamics/interfaces/coppeliasim/zmq/robots/HexapodLegCoppeliaSimZMQ.h>

#include <dqdynamics/robots/HexapodLeg.h>

#include <dqrobotics/utils/DQ_Constants.h>

namespace DQ_dynamics
{

/**
 * @brief Constructor of the HexapodLegCoppeliaSimZMQ class
 *
 * @param robot_name The name of robot used on the CoppeliaSim scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 *
 *  Example:
 *          auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *          vi->connect();
 *          vi->start_simulation();
 *          HexapodLegCoppeliaSimZMQ robot_coppeliasim("/hexapod_leg", vi);
 *          auto q = robot_coppeliasim.get_joint_positions();
 *          vi->stop_simulation();
 */
HexapodLegCoppeliaSimZMQ::HexapodLegCoppeliaSimZMQ(
    const std::string& robot_name,
    const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental>& vrep_interface_sptr):
    DQ_BranchedCoppeliaSimZMQRobot(robot_name, vrep_interface_sptr)
{
}

/**
 * @brief This method constructs an instance of a DQ_SerialManipulatorDH.
 *
 * @return A DQ_SerialManipulatorDH representing a leg from Unitree's B1
 *        robot.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      HexapodLegCoppeliaSimZMQ robot_coppeliasim("/hexapod_leg", vi);
 *      DQ_SerialManipulatorDH robot_dynamics = robot_coppeliasim.kinematics();
 *      vi->stop_simulation();
 */
DQ_SerialManipulatorDH HexapodLegCoppeliaSimZMQ::kinematics()
{
    // Create a DQ_SerialManipulatorDH object
    DQ_SerialManipulatorDH kin = HexapodLeg::kinematics();

    // Update base and reference frame with CoppeliaSim values
    const DQ& base_frame = this->_get_interface_sptr()->get_object_pose(
        this->base_frame_name_);
    kin.set_reference_frame(base_frame);
    kin.set_base_frame(base_frame);

    return kin;
}

/**
 * @brief This method constructs an instance of a DQ_SerialManipulatorDynamics.
 *
 * @return A DQ_SerialManipulatorDynamics representing a KUKA LBR+ robot.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      HexapodLegCoppeliaSimZMQ robot_coppeliasim("/hexapod_leg", vi);
 *      DQ_SerialManipulatorDynamics robot_dynamics = robot_coppeliasim.dynamics();
 *      vi->stop_simulation();
 */
DQ_SerialManipulatorDynamics HexapodLegCoppeliaSimZMQ::dynamics()
{
    // Create a DQ_SerialManipulatorDynamics object
    DQ_SerialManipulatorDynamics dyn = HexapodLeg::dynamics();

    // Set the robot configuration with CoppeliaSim values
    VectorXd q_read = this->get_configuration_space();
    VectorXd dq_read = this->get_configuration_space_velocities();
    VectorXd ddq_read = VectorXd::Zero(q_read.size(),1);

    dyn.set_configurations(q_read, dq_read, ddq_read);

    // Update base and reference frame with CoppeliaSim values
    const DQ& base_frame = this->_get_interface_sptr()->get_object_pose(
        this->base_frame_name_);
    dyn.set_reference_frame(base_frame);
    dyn.set_base_frame(base_frame);

    // Update dynamic parameters with CoppeliaSim information
    update_dynamic_parameters(dyn);

    return dyn;
}

}//namespace DQ_dynamics
