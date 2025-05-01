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

#include <dqdynamics/interfaces/coppeliasim/zmq/robots/B1LegCoppeliaSimZMQ.h>

#include <dqdynamics/robots/B1Leg.h>

#include <dqrobotics/utils/DQ_Constants.h>

namespace DQ_dynamics
{

/**
 * @brief Constructor of the B1LegCoppeliaSimZMQ class
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
 *          B1LegCoppeliaSimZMQ lbr4p_coppeliasim_robot("/LBR4p", vi);
 *          auto q = lbr4p_coppeliasim_robot.get_joint_positions();
 *          vi->stop_simulation();
 */
B1LegCoppeliaSimZMQ::B1LegCoppeliaSimZMQ(
    const std::string& robot_name,
    const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental>& vrep_interface_sptr):
    DQ_BranchedCoppeliaSimZMQRobot(robot_name, vrep_interface_sptr)
{
    // Fix the joint names automatically assigned by the
    // DQ_CoppeliaSimRobotZMQ class
    std::vector<std::string> joint_names{"/UnitreeB1/FR_hip_joint",
                                         "/UnitreeB1/FR_thigh_joint",
                                         "/UnitreeB1/FR_calf_joint"};
    this->jointnames_ = joint_names;

    // Fix the joint names automatically assigned by the
    // DQ_CoppeliaSimRobotZMQ class
    std::vector<std::string> link_names{"/UnitreeB1/FR_hip_respondable",
                                        "/UnitreeB1/FR_thigh_respondable",
                                        "/UnitreeB1/FR_calf_respondable"};
    link_names_ = link_names;

    // Fix base frame name automatically assigned by the
    // DQ_CoppeliaSimRobotZMQ class
    base_frame_name_ = jointnames_.at(0);
}

/**
 * @brief This method constructs an instance of a DQ_SerialManipulatorDH.
 *
 * @return A DQ_SerialManipulatorDH representing a KUKA LBR+ robot.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      vi->load_from_model_browser("/robots/non-mobile/KUKA LBR4+.ttm",
 *                                  "/LBR4p");
 *      B1LegCoppeliaSimZMQ lbr4p_coppeliasim_robot("/LBR4p", vi);
 *      DQ_SerialManipulatorDH lbr4p_robot =
 *                                  lbr4p_coppeliasim_robot.kinematics();
 *      vi->stop_simulation();
 */
DQ_SerialManipulatorDH B1LegCoppeliaSimZMQ::kinematics()
{
    // Create a DQ_SerialManipulatorDH object
    DQ_SerialManipulatorDH kin = B1Leg::kinematics();

    // Update base and reference frame with CoppeliaSim values
    kin.set_reference_frame(this->_get_interface_sptr()->get_object_pose(
                                                            this->base_frame_name_));
    kin.set_base_frame(this->_get_interface_sptr()->get_object_pose(
                                                            this->base_frame_name_));
    kin.set_effector(1+0.5*E_*k_*0.07);
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
 *      vi->load_from_model_browser("/robots/non-mobile/KUKA LBR4+.ttm",
 *                                  "/LBR4p");
 *      B1LegCoppeliaSimZMQ lbr4p_coppeliasim_robot("/LBR4p", vi);
 *      DQ_SerialManipulatorDynamics lbr4p_robot =
 *                                      lbr4p_coppeliasim_robot.dynamics();
 *      vi->stop_simulation();
 */
DQ_SerialManipulatorDynamics B1LegCoppeliaSimZMQ::dynamics()
{
    // Create a DQ_SerialManipulatorDynamics object
    DQ_SerialManipulatorDynamics dyn = B1Leg::dynamics();

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
