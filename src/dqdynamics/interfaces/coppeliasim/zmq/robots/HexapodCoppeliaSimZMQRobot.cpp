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

#include <dqdynamics/interfaces/coppeliasim/zmq/robots/HexapodCoppeliaSimZMQRobot.h>

#include <dqdynamics/robots/HexapodRobot.h>

#include <dqrobotics/DQ.h>

namespace DQ_dynamics
{

/**
 * @brief Constructor of the HexapodCoppeliaSimZMQRobot class
 *
 * @param robot_name The name of robot used on the CoppeliaSim scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 * @return A HexapodCoppeliaSimZMQRobot object representing the hexapod
 *         robot in the CoppeliaSim scene.
 *  Example:
 *          auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *          vi->connect();
 *          vi->start_simulation();
 *          HexapodCoppeliaSimZMQRobot robot_coppeliasim("/hexapod", vi);
 *          vi->stop_simulation();
 */
HexapodCoppeliaSimZMQRobot::HexapodCoppeliaSimZMQRobot(
    const std::string& robot_name,
    const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental>&
        vrep_interface_sptr):
    DQ_BranchedCoppeliaSimZMQRobot(robot_name, vrep_interface_sptr)
{
    // Fix the joint names automatically assigned by the
    // DQ_CoppeliaSimRobotZMQ class
    std::vector<std::string> joint_names{"/hexapod/joint1_[0]",
                                         "/hexapod/joint1_[0]/joint2_",
                                         "/hexapod/joint1_[0]/joint3_",
                                         "/hexapod/joint1_[1]",
                                         "/hexapod/joint1_[1]/joint2_",
                                         "/hexapod/joint1_[1]/joint3_",
                                         "/hexapod/joint1_[2]",
                                         "/hexapod/joint1_[2]/joint2_",
                                         "/hexapod/joint1_[2]/joint3_",
                                         "/hexapod/joint1_[3]",
                                         "/hexapod/joint1_[3]/joint2_",
                                         "/hexapod/joint1_[3]/joint3_",
                                         "/hexapod/joint1_[4]",
                                         "/hexapod/joint1_[4]/joint2_",
                                         "/hexapod/joint1_[4]/joint3_",
                                         "/hexapod/joint1_[5]",
                                         "/hexapod/joint1_[5]/joint2_",
                                         "/hexapod/joint1_[5]/joint3_"};
    this->jointnames_ = joint_names;

    // Fix the link names automatically assigned by the
    // DQ_BranchedCoppeliaSimZMQRobot class
    std::vector<std::string> link_names{"/hexapod/joint1_[0]/link1Respondable_",
                                        "/hexapod/joint1_[0]/link2Respondable_",
                                        "/hexapod/joint1_[0]/link3Respondable_",
                                        "/hexapod/joint1_[1]/link1Respondable_",
                                        "/hexapod/joint1_[1]/link2Respondable_",
                                        "/hexapod/joint1_[1]/link3Respondable_",
                                        "/hexapod/joint1_[2]/link1Respondable_",
                                        "/hexapod/joint1_[2]/link2Respondable_",
                                        "/hexapod/joint1_[2]/link3Respondable_",
                                        "/hexapod/joint1_[3]/link1Respondable_",
                                        "/hexapod/joint1_[3]/link2Respondable_",
                                        "/hexapod/joint1_[3]/link3Respondable_",
                                        "/hexapod/joint1_[4]/link1Respondable_",
                                        "/hexapod/joint1_[4]/link2Respondable_",
                                        "/hexapod/joint1_[4]/link3Respondable_",
                                        "/hexapod/joint1_[5]/link1Respondable_",
                                        "/hexapod/joint1_[5]/link2Respondable_",
                                        "/hexapod/joint1_[5]/link3Respondable_"};
    link_names_ = link_names;

    // Fix base frame name automatically assigned by the
    // DQ_CoppeliaSimRobotZMQ class
    base_frame_name_ = "/hexapod/body";
}

/**
 * @brief Returns the vector of joint positions of hexapod's legs.
 *
 * @return A VectorXd with hexapod's joint positions in the CoppeliaSim scene.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      HexapodCoppeliaSimZMQRobot robot_coppeliasim("/hexapod", vi);
 *      VectorXd q = robot_coppeliasim.get_joint_positions();
 *      vi->stop_simulation();
 */
VectorXd HexapodCoppeliaSimZMQRobot::get_joint_positions()
{
    return DQ_BranchedCoppeliaSimZMQRobot::get_configuration_space();
}

/**
 * @brief Returns hexapod's vector of configuration space positions in the
 *        CoppeliaSim scene, which is composed by the vec8() of the unit dual
 *        quaternion representing the pose of hexapod's body concatenated with
 *        the joint positions of its legs. In other words,
 *          q = [vec8(x_body); q_legs]
 *
 * @return A VectorXd with hexapod's vector of configuration space positions
 *         in the CoppeliaSim scene.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      HexapodCoppeliaSimZMQRobot robot_coppeliasim("/hexapod", vi);
 *      VectorXd q = robot_coppeliasim.get_configuration_space();
 *      vi->stop_simulation();
 */
VectorXd HexapodCoppeliaSimZMQRobot::get_configuration_space()
{
    // Read the body's pose from CoppeliaSim
    const auto coppeliasim_sptr = this->_get_interface_sptr();
    DQ_robotics::DQ x_body = DQ_robotics::normalize(
        coppeliasim_sptr->get_object_pose(base_frame_name_));

    // Read the leg's joint positions from CoppeliaSim
    VectorXd q_legs = DQ_BranchedCoppeliaSimZMQRobot::get_configuration_space();

    // Concatenate the vec8() of the body's pose with the leg's joint positions
    const int& n_joints = this->get_joint_names().size();
    VectorXd q(8 + n_joints, 1);
    q << x_body.vec8(),
         q_legs;

    return q;
}

/**
 * @brief Returns hexapod's vector of configuration space velocities in the
 *        CoppeliaSim scene, which is composed by the vec8() of pure dual
 *        quaternion representing the twist of hexapod's body concatenated
 *        with the joint velocities of its legs. In other words,
 *          dq = [vec8(xi_body); dq_legs]
 *
 * @return A VectorXd with hexapod's vector of configuration space velocities
 *         in the CoppeliaSim scene.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      HexapodCoppeliaSimZMQRobot robot_coppeliasim("/hexapod", vi);
 *      VectorXd dq =
 *          robot_coppeliasim.get_configuration_space_velocities();
 *      vi->stop_simulation();
 */
VectorXd HexapodCoppeliaSimZMQRobot::get_configuration_space_velocities()
{
    // Read the body's twist from CoppeliaSim
    const auto coppeliasim_sptr = this->_get_interface_sptr();
    DQ_robotics::DQ xi_body = coppeliasim_sptr->get_twist(base_frame_name_);

    // Read the leg's joint positions from CoppeliaSim
    VectorXd dq_legs =
        DQ_BranchedCoppeliaSimZMQRobot::get_configuration_space_velocities();

    // Concatenate the vec8() of the body's pose with the leg's joint positions
    const int& n_joints = this->get_joint_names().size();
    VectorXd dq(8 + n_joints, 1);
    dq << xi_body.vec8(),
          dq_legs;

    return dq;
}

/**
 * @brief This method constructs an instance of a DQ_BranchedWholeBody.
 *
 * @return A DQ_BranchedWholeBody representing a hexapod robot.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      HexapodCoppeliaSimZMQRobot robot_coppeliasim("/hexapod", vi);
 *      DQ_BranchedWholeBody robot_dynamics = robot_coppeliasim.dynamics();
 *      vi->stop_simulation();
 */
DQ_BranchedWholeBody HexapodCoppeliaSimZMQRobot::dynamics()
{
    // Create a DQ_BranchedWholeBody object
    DQ_BranchedWholeBody dyn = HexapodRobot::dynamics();

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
