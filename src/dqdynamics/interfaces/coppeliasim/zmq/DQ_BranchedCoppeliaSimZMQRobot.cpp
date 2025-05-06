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

#include <dqdynamics/interfaces/coppeliasim/zmq/DQ_BranchedCoppeliaSimZMQRobot.h>

#include <dqdynamics/DQ_Extended.h>
#include <dqdynamics/robots/FreeFlyingRobotDynamics.h>
#include <dqdynamics/utils/DQ_Conversions.h>

namespace DQ_dynamics
{
/****************************************************************
****DQ_BranchedCoppeliaSimZMQRobot PROTECTED CLASS METHODS****
*****************************************************************/

/**
 * @brief This method updates the dynamic parameters of the branched robot's
 *        base in the CoppeliaSim scene.
 *
 * @param robot_dynamics A DQ_Dynamics object representing the robot's base
 *        in the CoppeliaSim scene.
 */
void DQ_BranchedCoppeliaSimZMQRobot::update_base_dynamic_parameters(
    DQ_Dynamics& robot_dynamics)
{
    // Update the mobile base's mass
    double mass = this->_get_interface_sptr()->get_mass(this->base_frame_name_);
    VectorXd masses = VectorXd::Zero(1, 1);
    masses(0) = mass;

    // Update the mobile base's center of mass position
    DQ_robotics::DQ center_of_mass =
        this->_get_interface_sptr()->get_center_of_mass(this->base_frame_name_,
            DQ_CoppeliaSimInterfaceZMQExperimental::REFERENCE::BODY_FRAME);
    VectorXdq position_CoMs = VectorXdq::Zero(1, 1);
    position_CoMs(0)= center_of_mass;

    // Update the mobile base's inertia tensor
    MatrixXd inertia_tensor =
        this->_get_interface_sptr()->get_inertia_matrix(this->base_frame_name_,
            DQ_CoppeliaSimInterfaceZMQExperimental::REFERENCE::BODY_FRAME);

    // Check if the simulation is using the MuJoCo engine and include
    // the effects of joint armature in the link's inertia
    if (this->_get_interface_sptr()->get_engine() == "MUJOCO"){
        const double joint_armature =
            this->_get_interface_sptr()->get_mujoco_joint_armature(
                this->base_frame_name_);
        if (joint_armature != 0){
            inertia_tensor = inertia_tensor + pow(joint_armature,2.)*inertia_tensor;
        }
    }

    std::vector<Matrix3d> inertia_tensors{inertia_tensor};

    robot_dynamics.set_masses(masses);
    robot_dynamics.set_position_CoMs(position_CoMs);
    robot_dynamics.set_inertia_tensors(inertia_tensors);
}

/**
 * @brief This method updates the dynamic parameters of a branch from the
 *        branched robot in the CoppeliaSim scene.
 *
 * @param robot_dynamics A DQ_Dynamics object representing a branch from
 *        the robot in the CoppeliaSim scene.
 * @param starting_name_index An int representing the index indicating
 *        in which range from the properties 'joint_names'/'link_names'
 *        the link/joint names of this branch are stored.
 */
void DQ_BranchedCoppeliaSimZMQRobot::update_branch_dynamic_parameters(
    DQ_Dynamics& robot_dynamics,
    const int&starting_name_index)
{
    const int& num_bodies = robot_dynamics.get_num_bodies();
    VectorXd masses = VectorXd::Zero(num_bodies, 1);
    VectorXdq position_CoMs = VectorXdq::Zero(num_bodies, 1);
    std::vector<Matrix3d> inertia_tensors;

    int name_index = starting_name_index;
    VectorXd q_read = robot_dynamics.get_configuration_space_positions();
    for (int i=0; i<num_bodies; i++){
        std::string link_name = this->link_names_.at(name_index);

        // Get the link's mass
        masses(i) = this->_get_interface_sptr()->get_mass(link_name);

        // Get the link's center of mass position with respect to the end-link frame
        DQ_robotics::DQ pcm_in_0 = this->_get_interface_sptr()->get_center_of_mass(
            link_name,
            DQ_CoppeliaSimInterfaceZMQExperimental::REFERENCE::ABSOLUTE_FRAME);
        DQ_robotics::DQ rcm_in_0 = this->_get_interface_sptr()->get_object_rotation(
            link_name);
        DQ_robotics::DQ xcm_in_0 = rcm_in_0 + 0.5*DQ_robotics::E_*pcm_in_0*rcm_in_0;
        DQ_robotics::DQ xl_in_0 = robot_dynamics.fkm(q_read, i);
        DQ_robotics::DQ xcm_in_xl = (xl_in_0.conj())*xcm_in_0;
        position_CoMs(i) = xcm_in_xl.translation();

        // Get the link's inertia tensor with respect to the end-link frame
        MatrixXd inertia_tensor_in_0 = this->_get_interface_sptr()->
                                       get_inertia_matrix(link_name,
            DQ_CoppeliaSimInterfaceZMQExperimental::REFERENCE::ABSOLUTE_FRAME);
        Matrix3d Rcm_in_0 =
            DQ_Conversions::rotation_matrix_from_quaternion(xcm_in_0.P());
        Matrix3d inertia_tensor_ith_link =
            (Rcm_in_0.transpose())*inertia_tensor_in_0*Rcm_in_0;

        // Check if the simulation is using the MuJoCo engine and include
        // the effects of joint armature in the link's inertia
        if (this->_get_interface_sptr()->get_engine() == "MUJOCO"){
            const double joint_armature =
                this->_get_interface_sptr()->get_mujoco_joint_armature(
                    jointnames_.at(name_index));
            if (joint_armature != 0){
                inertia_tensor_ith_link = inertia_tensor_ith_link +
                    pow(joint_armature,2.)*inertia_tensor_ith_link;
            }
        }

        inertia_tensors.push_back(inertia_tensor_ith_link);

        // Move to the next name index
        ++name_index;
    }

    robot_dynamics.set_masses(masses);
    robot_dynamics.set_position_CoMs(position_CoMs);
    robot_dynamics.set_inertia_tensors(inertia_tensors);
}

/**
 * @brief This method updates the dynamic parameters of the robotic
 *        manipulator in the CoppeliaSim scene.
 *
 * @param robot_dynamics A DQ_SerialManipulatorDynamics object representing
 *        the robot in the CoppeliaSim scene.
 * @param starting_name_index An int representing the index indicating
 *        in which range from the properties 'joint_names'/'link_names'
 *        the link/joint names of this serial manipulator are stored.
 *        (Default: 1)
 */
void DQ_BranchedCoppeliaSimZMQRobot::update_dynamic_parameters(
    DQ_SerialManipulatorDynamics& robot_dynamics,
    const int &starting_name_index)
{
    std::cout << "Updating dynamic parameters of the robotic manipulator..."
              << std::endl;
    std::cout << std::endl;

    update_branch_dynamic_parameters(robot_dynamics, starting_name_index);

}

/**
 * @brief This method updates the dynamic parameters of the branched robot
 *        in the CoppeliaSim scene.
 *
 * @param robot_dynamics A DQ_BranchedWholeBody object representing the
 *        robot in the CoppeliaSim scene.
 * @param starting_name_index An int representing the index indicating
 *        in which range from the properties 'joint_names'/'link_names'
 *        the link/joint names of this branch are stored. (Default: 1)
 */
void DQ_BranchedCoppeliaSimZMQRobot::update_dynamic_parameters(
    DQ_BranchedWholeBody &robot_dynamics,
    const int &starting_name_index)
{
    std::vector<std::shared_ptr<DQ_Dynamics>> my_chain =
        robot_dynamics.get_chain();

    DQ_FreeFlyingRobotDynamics* floating_base_prt;
    floating_base_prt = dynamic_cast<DQ_FreeFlyingRobotDynamics*>(
        my_chain.at(0).get());
    int start_from = 0;
    if (floating_base_prt){ // the base is a free-flying robot
        std::cout << "Updating dynamic parameters of the root subsystem..."
                  << std::endl;

        this->update_base_dynamic_parameters(*my_chain.at(0));

        start_from = 1;
    }

    int chain_size = robot_dynamics.get_chain().size();
    int name_index = starting_name_index;
    for (int i=start_from; i<chain_size; i++){
        std::cout << "Updating dynamic parameters of subsystem "
                  << i+1 << "..." << std::endl;

        this->update_branch_dynamic_parameters(*my_chain.at(i), name_index);

        name_index = name_index +
                     my_chain.at(i)->get_configuration_space_positions().size();
    }
    std::cout << std::endl;
}

/****************************************************************
******DQ_BranchedCoppeliaSimZMQRobot PUBLIC CLASS METHODS*****
*****************************************************************/

/**
 * @brief Creates a DQ_BranchedCoppeliaSimZMQRobot object with set joint
 *        and link names.
 *
 * @param robot_name The name of the robot in the CoppeliaSim scene.
 * @param interface_sptr The DQ_CoppeliaSimInterfaceZMQExperimental smart pointer.
 */
DQ_BranchedCoppeliaSimZMQRobot::DQ_BranchedCoppeliaSimZMQRobot(
    const std::string& robot_name,
    const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental> &interface_sptr):
    DQ_CoppeliaSimRobotZMQ(robot_name, interface_sptr)
{
    // Initialize link names
    link_names_ = this->_get_interface_sptr()->
                  get_shapenames_from_parent_object(robot_name,
                    DQ_CoppeliaSimInterfaceZMQExperimental::SHAPE_TYPE::DYNAMIC);
}

/**
 * @brief Returns the time robot's link names.
 * @return A vector of strings representing the robot's link names.
 */
std::vector<std::string> DQ_BranchedCoppeliaSimZMQRobot::get_link_names() const
{
    return link_names_;
}

/**
 * @brief Sets the robot's joint mode and joint control mode.
 *
 * @param joint_mode A valid JOINT_MODE.
 * @param interface_sptr A valid JOINT_CONTROL_MODE.
 */
void DQ_BranchedCoppeliaSimZMQRobot::set_joint_operation_modes(
    const DQ_CoppeliaSimInterfaceZMQExperimental::JOINT_MODE& joint_mode,
    const DQ_CoppeliaSimInterfaceZMQExperimental::JOINT_CONTROL_MODE& joint_control_mode)
{
    this->_get_interface_sptr()->set_joint_modes(this->jointnames_, joint_mode);
    this->_get_interface_sptr()->set_joint_control_modes(this->jointnames_,
                                                         joint_control_mode);
}

/**
 * @brief Sets the joint armatures. This results in the effect known as
 *        “reflected inertia,” which is equivalent to multiplying the inertia
 *        tensor of the body actuated by the joint by the square of the joint
 *        armature. For instance, if the body actuated by a joint has inertia
 *        tensor I, setting the joint armature to x is equivalent to setting
 *        the inertia tensor of the body to I + (x^2)*I. This parameter only
 *        applies to the MuJoCo engine. For more details, refer to:
 *          https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-armature
 *
 * @param joint_armatures An integer representing additional inertia associated
 *        with movement of the joint that is not due to body mass.
 */
void DQ_BranchedCoppeliaSimZMQRobot::set_joint_armatures(const int& joint_armatures)
{
    // Check if the simulation is using the MuJoCo engine
    if (this->_get_interface_sptr()->get_engine() != "MUJOCO"){
        throw std::runtime_error("This joint armatures only apply to the MuJoCo "
                                 "engine! Please change the engine of your "
                                 "simulation before using this method.");
    }

    // Set the joint armature of all the joints
    this->_get_interface_sptr()->set_mujoco_joint_armatures(jointnames_,
                                                            joint_armatures);
}

/**
 * @brief Sets the joint dampings. This parameter only applies to the MuJoCo
 *        engine. For more details, refer to:
 *          https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-damping
 *
 * @param joint_dampings An integer representing the damping applied to all
 *        degrees of freedom created by this joint.
 */
void DQ_BranchedCoppeliaSimZMQRobot::set_joint_dampings(const int& joint_dampings)
{
    // Check if the simulation is using the MuJoCo engine
    if (this->_get_interface_sptr()->get_engine() != "MUJOCO"){
        throw std::runtime_error("This joint armatures only apply to the MuJoCo "
                                 "engine! Please change the engine of your "
                                 "simulation before using this method.");
    }

    // Set the joint dampings of all the joints
    this->_get_interface_sptr()->set_mujoco_joint_dampings(jointnames_,
                                                           joint_dampings);
}

/**
 * @brief Sets the contact friction parameters of all the links. This parameter
 *        only applies to the MuJoCo engine. For more details, refer to:
 *          https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom-friction
 *
 * @param link_frictions A vector of integers in which each element represents
 *        the contact friction parameters of the link. The first number is the
 *        sliding friction, acting along both axes of the tangent plane. The
 *        second number is the torsional friction, acting around the contact
 *        normal. The third number is the rolling friction, acting around
 *        both axes of the tangent plane.
 */
void DQ_BranchedCoppeliaSimZMQRobot::set_link_frictions(
    const std::vector<double>& link_frictions)
{
    // Check if the simulation is using the MuJoCo engine
    if (this->_get_interface_sptr()->get_engine() != "MUJOCO"){
        throw std::runtime_error("This joint armatures only apply to the MuJoCo "
                                 "engine! Please change the engine of your "
                                 "simulation before using this method.");
    }

    // Set the friction coefficients of all the links
    this->_get_interface_sptr()->set_mujoco_body_frictions(link_names_,
                                                           link_frictions);
}

}//namespace DQ_dynamics
