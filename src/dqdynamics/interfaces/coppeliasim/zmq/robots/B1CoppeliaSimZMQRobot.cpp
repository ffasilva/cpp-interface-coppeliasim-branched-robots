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

#include <string>

#include <dqdynamics/interfaces/coppeliasim/zmq/robots/B1CoppeliaSimZMQRobot.h>

#include <dqdynamics/robots/B1Robot.h>

#include <dqrobotics/DQ.h>

namespace DQ_dynamics
{

/**
 * @brief Constructor of the B1CoppeliaSimZMQRobot class
 *
 * @param robot_name The name of robot used on the CoppeliaSim scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 * @return A B1CoppeliaSimZMQRobot object representing Unitree's B1
 *         robot in the CoppeliaSim scene.
 *  Example:
 *          auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *          vi->connect();
 *          vi->start_simulation();
 *          B1CoppeliaSimZMQRobot robot_coppeliasim("/UnitreeB1", vi);
 *          vi->stop_simulation();
 */
B1CoppeliaSimZMQRobot::B1CoppeliaSimZMQRobot(
    const std::string& robot_name,
    const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental>&
        vrep_interface_sptr):
    DQ_BranchedCoppeliaSimZMQRobot(robot_name, vrep_interface_sptr)
{
    // Fix the joint names automatically assigned by the
    // DQ_CoppeliaSimRobotZMQ class
    std::vector<std::string> joint_names{"/UnitreeB1/FR_hip_joint",
                                         "/UnitreeB1/FR_thigh_joint",
                                         "/UnitreeB1/FR_calf_joint",
                                         "/UnitreeB1/FL_hip_joint",
                                         "/UnitreeB1/FL_thigh_joint",
                                         "/UnitreeB1/FL_calf_joint",
                                         "/UnitreeB1/RR_hip_joint",
                                         "/UnitreeB1/RR_thigh_joint",
                                         "/UnitreeB1/RR_calf_joint",
                                         "/UnitreeB1/RL_hip_joint",
                                         "/UnitreeB1/RL_thigh_joint",
                                         "/UnitreeB1/RL_calf_joint"};
    this->jointnames_ = joint_names;

    // Fix the link names automatically assigned by the
    // DQ_BranchedCoppeliaSimZMQRobot class
    std::vector<std::string> link_names{"/UnitreeB1/FR_hip_respondable",
                                        "/UnitreeB1/FR_thigh_respondable",
                                        "/UnitreeB1/FR_calf_respondable",
                                        "/UnitreeB1/FL_hip_respondable",
                                        "/UnitreeB1/FL_thigh_respondable",
                                        "/UnitreeB1/FL_calf_respondable",
                                        "/UnitreeB1/RR_hip_respondable",
                                        "/UnitreeB1/RR_thigh_respondable",
                                        "/UnitreeB1/RR_calf_respondable",
                                        "/UnitreeB1/RL_hip_respondable",
                                        "/UnitreeB1/RL_thigh_respondable",
                                        "/UnitreeB1/RL_calf_respondable"};
    link_names_ = link_names;

    // Fix base frame name automatically assigned by the
    // DQ_CoppeliaSimRobotZMQ class
    base_frame_name_ = "/UnitreeB1/trunk_respondable";
}

/**
 * @brief Returns the vector of joint positions of B1's front rigth leg, front
 *        left leg, rear right leg, and rear left leg. In other words,
 *          q_joints = [q_fr_leg; q_fl_leg; q_rr_leg; q_rl_leg;]
 *
 * @return A VectorXd with B1's joint positions in the CoppeliaSim scene.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      B1CoppeliaSimZMQRobot robot_coppeliasim("/UnitreeB1", vi);
 *      VectorXd q = robot_coppeliasim.get_joint_positions();
 *      vi->stop_simulation();
 */
VectorXd B1CoppeliaSimZMQRobot::get_joint_positions()
{
    return DQ_BranchedCoppeliaSimZMQRobot::get_configuration_space();
}

/**
 * @brief Returns B1's vector of configuration space positions in the CoppeliaSim
 *        scene, which is composed by the vec8() of the unit dual quaternion
 *        representing the pose of B1's trunk concatenated with the joint positions
 *        of its front rigth leg, front left leg, rear right leg, and rear left leg.
 *        In other words,
 *          q = [vec8(x_trunk); q_fr_leg; q_fl_leg; q_rr_leg; q_rl_leg;]
 *
 * @return A VectorXd with B1's vector of configuration space positions in the
 *         CoppeliaSim scene.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      B1CoppeliaSimZMQRobot robot_coppeliasim("/UnitreeB1", vi);
 *      VectorXd q = robot_coppeliasim.get_configuration_space();
 *      vi->stop_simulation();
 */
VectorXd B1CoppeliaSimZMQRobot::get_configuration_space()
{
    // Read the trunk's pose from CoppeliaSim
    const auto coppeliasim_sptr = this->_get_interface_sptr();
    DQ_robotics::DQ x_trunk = DQ_robotics::normalize(
        coppeliasim_sptr->get_object_pose(base_frame_name_));

    // Read the leg's joint positions from CoppeliaSim
    VectorXd q_legs = DQ_BranchedCoppeliaSimZMQRobot::get_configuration_space();

    // Concatenate the vec8() of the trunk's pose with the leg's joint positions
    const int& n_joints = this->get_joint_names().size();
    VectorXd q(8 + n_joints, 1);
    q << x_trunk.vec8(),
         q_legs;

    return q;
}

/**
 * @brief Returns B1's vector of configuration space velocities in the
 *        CoppeliaSim scene, which is composed by the vec8() of pure dual
 *        quaternion representing the twist of B1's trunk concatenated with
 *        the joint velocities of its front rigth leg, front left leg, rear
 *        right leg, and rear left leg. In other words,
 *          dq = [vec8(xi_trunk); dq_fr_leg; dq_fl_leg; dq_rr_leg; dq_rl_leg;]
 *
 * @return A VectorXd with B1's vector of configuration space velocities in the
 *         CoppeliaSim scene.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      B1CoppeliaSimZMQRobot robot_coppeliasim("/free_flying", vi);
 *      VectorXd dq =
 *          robot_coppeliasim.get_configuration_space_velocities();
 *      vi->stop_simulation();
 */
VectorXd B1CoppeliaSimZMQRobot::get_configuration_space_velocities()
{
    // Read the trunk's twist from CoppeliaSim
    const auto coppeliasim_sptr = this->_get_interface_sptr();
    DQ_robotics::DQ xi_trunk = coppeliasim_sptr->get_twist(base_frame_name_);

    // Read the leg's joint positions from CoppeliaSim
    VectorXd dq_legs =
        DQ_BranchedCoppeliaSimZMQRobot::get_configuration_space_velocities();

    // Concatenate the vec8() of the trunk's pose with the leg's joint positions
    const int& n_joints = this->get_joint_names().size();
    VectorXd dq(8 + n_joints, 1);
    dq << xi_trunk.vec8(),
          dq_legs;

    return dq;
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
 *      B1CoppeliaSimZMQRobot robot_coppeliasim("/BM_TRO2023", vi);
 *      DQ_BranchedWholeBody robot_dynamics = robot_coppeliasim.dynamics();
 *      vi->stop_simulation();
 */
DQ_BranchedWholeBody B1CoppeliaSimZMQRobot::dynamics()
{
    // Create a DQ_BranchedWholeBody object
    DQ_BranchedWholeBody dyn = B1Robot::dynamics();

    // Set the robot configuration with CoppeliaSim values
    VectorXd q_read = this->get_configuration_space();
    VectorXd dq_read = this->get_configuration_space_velocities();
    VectorXd ddq_read = VectorXd::Zero(q_read.rows());

    dyn.set_configurations(q_read, dq_read, ddq_read);

    // Update base and reference frame of the root subsystem with
    // CoppeliaSim values
    const auto coppeliasim_sptr = this->_get_interface_sptr();
    const DQ_robotics::DQ& base_frame = DQ_robotics::normalize(
        coppeliasim_sptr->get_object_pose(this->base_frame_name_));

    std::shared_ptr<DQ_Dynamics> root_subsystem = dyn.get_chain().at(0);
    root_subsystem->set_reference_frame(base_frame);
    root_subsystem->set_base_frame(base_frame);

    // Set the base frame of the remaining subsystems with CoppeliaSim values
    int index_beginning = 0;
    const std::vector<std::string>& joint_names = this->jointnames_;
    for (size_t i=1; i<dyn.get_chain().size(); i++){
        DQ_robotics::DQ branch_base_frame_in_0 = DQ_robotics::normalize(
            coppeliasim_sptr->get_object_pose(
                joint_names.at(index_beginning)));

        std::shared_ptr<DQ_Dynamics> my_root_subsystem = dyn.get_chain().at(
            dyn.get_connection_branch_of_ith_branch(i));
        DQ_robotics::DQ x_root_link_in_0 = my_root_subsystem->fkm(
            my_root_subsystem->get_configuration_space_positions(),
            dyn.get_connection_link_of_ith_branch(i));

        DQ_robotics::DQ branch_base_frame =
            (x_root_link_in_0.conj())*branch_base_frame_in_0;

        dyn.set_branch_base_frame_of_ith_branch(branch_base_frame, i);

        index_beginning = index_beginning +
            dyn.get_chain().at(i)->get_configuration_space_positions().size();
    }

    // Update dynamic parameters with CoppeliaSim information
    update_dynamic_parameters(dyn);

    return dyn;
}

}//namespace DQ_dynamics
