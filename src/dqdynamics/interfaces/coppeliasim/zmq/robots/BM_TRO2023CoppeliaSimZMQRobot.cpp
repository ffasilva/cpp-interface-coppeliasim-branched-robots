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

#include <dqdynamics/interfaces/coppeliasim/zmq/robots/BM_TRO2023CoppeliaSimZMQRobot.h>

#include <dqdynamics/robots/BM_TRO2023Robot.h>

#include <dqrobotics/DQ.h>

#include <memory.h>
#include <string>

namespace DQ_dynamics
{

/**
 * @brief Constructor of the BM_TRO2023CoppeliaSimZMQRobot class
 *
 * @param robot_name The name of robot used on the CoppeliaSim scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 *
 *  Example:
 *          auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *          vi->connect();
 *          vi->start_simulation();
 *          vi->load_from_model_browser("/robots/non-mobile/BM_TRO2023.ttm",
 *                                      "/BM_TRO2023");
 *          BM_TRO2023CoppeliaSimZMQRobot robot_coppeliasim("/BM_TRO2023", vi);
 *          auto q = robot_coppeliasim.get_joint_positions();
 *          vi->stop_simulation();
 */
BM_TRO2023CoppeliaSimZMQRobot::BM_TRO2023CoppeliaSimZMQRobot(
    const std::string& robot_name,
    const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental>& vrep_interface_sptr):
    DQ_BranchedCoppeliaSimZMQRobot(robot_name, vrep_interface_sptr)
{
    // Fix the joint names automatically assigned by the
    // DQ_CoppeliaSimRobotZMQ class
    for (unsigned long i=0; i<this->jointnames_.size(); i++){
        this->jointnames_.at(i) = robot_name + "/joint" + std::to_string(i+1);
    }

    // Fix the link names automatically assigned by the
    // DQ_BranchedCoppeliaSimZMQRobot class
    for (unsigned long i=0; i<this->link_names_.size(); i++){
        this->link_names_.at(i) = robot_name + "/link" + std::to_string(i+2);
    }
}

/**
 * @brief This method constructs an instance of a DQ_BranchedWholeBody.
 *
 * @return A DQ_BranchedWholeBody representing a BM_TRO2023 robot.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      vi->load_from_model_browser("/robots/non-mobile/BM_TRO2023.ttm",
 *                                  "/BM_TRO2023");
 *      BM_TRO2023CoppeliaSimZMQRobot robot_coppeliasim("/BM_TRO2023", vi);
 *      DQ_BranchedWholeBody robot_dynamics = robot_coppeliasim.dynamics();
 *      vi->stop_simulation();
 */
DQ_BranchedWholeBody BM_TRO2023CoppeliaSimZMQRobot::dynamics()
{
    // Create a DQ_BranchedWholeBody object
    DQ_BranchedWholeBody dyn = BM_TRO2023Robot::dynamics();

    // Set the robot configuration with CoppeliaSim values
    VectorXd q_read = this->get_configuration_space();
    VectorXd dq_read = this->get_configuration_space_velocities();
    VectorXd ddq_read = VectorXd::Zero(q_read.rows());

    dyn.set_configurations(q_read, dq_read, ddq_read);

    // Update base and reference frame of the root subsystem with
    // CoppeliaSim values
    const auto coppeliasim_sptr = this->_get_interface_sptr();
    const DQ& base_frame = coppeliasim_sptr->get_object_pose(
        this->base_frame_name_);

    std::shared_ptr<DQ_Dynamics> root_subsystem = dyn.get_chain().at(0);
    root_subsystem->set_reference_frame(base_frame);
    root_subsystem->set_base_frame(base_frame);

    // Set the base frame of the remaining subsystems with CoppeliaSim values
    std::vector<int> index_beginning_i;
    std::tie(index_beginning_i, std::ignore) = dyn.get_indexes_subsystems();
    const std::vector<std::string>& joint_names = this->jointnames_;
    for (unsigned long i=1; i<dyn.get_chain().size(); i++){
        DQ_robotics::DQ branch_base_frame_in_0 = coppeliasim_sptr->
            get_object_pose(joint_names.at(index_beginning_i.at(i)));

        std::shared_ptr<DQ_Dynamics> my_root_subsystem = dyn.get_chain().at(
            dyn.get_connection_branch_of_ith_branch(i));
        DQ_robotics::DQ x_root_link_in_0 = my_root_subsystem->fkm(
            my_root_subsystem->get_configuration_space_positions(),
            dyn.get_connection_link_of_ith_branch(i));

        DQ_robotics::DQ branch_base_frame =
            (x_root_link_in_0.conj())*branch_base_frame_in_0;

        dyn.set_branch_base_frame_of_ith_branch(branch_base_frame, i);
    }

    // // Update dynamic parameters with CoppeliaSim information
    update_dynamic_parameters(dyn);

    return dyn;
}

}//namespace DQ_dynamics
