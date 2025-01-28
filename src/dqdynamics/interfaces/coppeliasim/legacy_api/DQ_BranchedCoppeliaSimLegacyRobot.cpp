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

#include <dqdynamics/DQ_Extended.h>
#include <dqdynamics/utils/DQ_Conversions.h>
#include <dqdynamics/interfaces/coppeliasim/legacy_api/DQ_BranchedCoppeliaSimLegacyRobot.h>

namespace DQ_dynamics
{
/****************************************************************
****DQ_BRANCHEDCOPPELIASIMLEGACYROBOT PROTECTED CLASS METHODS****
*****************************************************************/

/**
 * @brief This method updates the dynamic parameters of the branched robot's
 *        base in the CoppeliaSim scene.
 *
 * @param robot_dynamics A DQ_Dynamics object representing the robot's base
 *        in the CoppeliaSim scene.
 */
void DQ_BranchedCoppeliaSimLegacyRobot::update_base_dynamic_parameters(
    std::shared_ptr<DQ_Dynamics> robot_dynamics)
{
    // Update the mobile base's mass
    double mass = this->_get_interface_sptr()->get_mass(this->base_frame_name_);
    VectorXd masses = VectorXd::Zero(1, 1);
    masses(0) = mass;
    robot_dynamics->set_masses(masses);

    // Update the mobile base's center of mass position
    DQ_robotics::DQ center_of_mass =
        this->_get_interface_sptr()->get_center_of_mass(this->base_frame_name_);
    VectorXdq position_CoMs = VectorXdq::Zero(1, 1);
    position_CoMs(0)= center_of_mass;
    robot_dynamics->set_position_CoMs(position_CoMs);

    // Update the mobile base's inertia tensor
    MatrixXd inertia_tensor =
        this->_get_interface_sptr()->get_inertia_matrix(this->base_frame_name_);
    std::vector<Matrix3d> inertia_tensors{inertia_tensor};
    robot_dynamics->set_inertia_tensors(inertia_tensors);
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
void DQ_BranchedCoppeliaSimLegacyRobot::update_branch_dynamic_parameters(
    std::shared_ptr<DQ_Dynamics> robot_dynamics,
    const int&starting_name_index)
{
    int dim_configuration_space = robot_dynamics->get_dim_configuration_space();
    VectorXd masses = VectorXd::Zero(dim_configuration_space, 1);
    VectorXdq position_CoMs = VectorXdq::Zero(dim_configuration_space, 1);
    std::vector<Matrix3d> inertia_tensors;

    int name_index = starting_name_index;
    VectorXd q_read = robot_dynamics->get_configuration_space_positions();
    for (int i=0; i<dim_configuration_space; i++){
        std::string link_name = this->link_names_.at(name_index);
        // Get the link's mass
        masses(i) = this->_get_interface_sptr()->get_mass(link_name);

        // Get the link's center of mass position with respect to the end-link frame
        DQ_robotics::DQ pcm_in_0 = this->_get_interface_sptr()->get_center_of_mass(
            link_name,
            DQ_VrepInterface::REFERENCE_FRAMES::ABSOLUTE_FRAME);

        DQ_robotics::DQ rcm_in_0 = this->_get_interface_sptr()->get_object_rotation(
            link_name);
        DQ_robotics::DQ xcm_in_0 = rcm_in_0 + (1/2)*DQ_robotics::E_*pcm_in_0*rcm_in_0;
        DQ_robotics::DQ xl_in_0 = robot_dynamics->fkm(q_read, i);
        DQ_robotics::DQ xcm_in_xl = (xl_in_0.conj())*xcm_in_0;
        position_CoMs(i) = xcm_in_xl.translation();

        // I need to make sure that set_position_CoMs() calls _update_dh_frames()
        // as it is, updating the CoM position does not update the pose

        // Get the link's inertia tensor with respect to the end-link frame
        MatrixXd inertia_tensor_in_0 = this->_get_interface_sptr()->get_inertia_matrix(
            link_name,
            DQ_VrepInterface::REFERENCE_FRAMES::ABSOLUTE_FRAME);

        Matrix3d Rcm_in_0 = DQ_Conversions::rotation_matrix_from_quaternion(xcm_in_0.P());
        const Matrix3d inertia_tensor_ith_link =
            (Rcm_in_0.transpose())*inertia_tensor_in_0*Rcm_in_0;
        inertia_tensors.push_back(inertia_tensor_ith_link);

        // Move to the next name index
        ++name_index;
    }

    // Update the link's mass, center of mass position, and inertia tensor
    robot_dynamics->set_masses(masses);
    robot_dynamics->set_position_CoMs(position_CoMs);
    robot_dynamics->set_inertia_tensors(inertia_tensors);
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
void DQ_BranchedCoppeliaSimLegacyRobot::update_dynamic_parameters(
    std::shared_ptr<DQ_SerialManipulatorDynamics> robot_dynamics,
    const int &starting_name_index)
{
    this->update_branch_dynamic_parameters(robot_dynamics, starting_name_index);
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
void DQ_BranchedCoppeliaSimLegacyRobot::update_dynamic_parameters(
    std::shared_ptr<DQ_BranchedWholeBody> robot_dynamics,
    const int &starting_name_index)
{
    int chain_size = robot_dynamics->get_chain().size();

    int name_index = starting_name_index;
    std::vector<std::shared_ptr<DQ_Dynamics>> my_chain = robot_dynamics->get_chain();
    for (int i=0; i<chain_size; i++){
        this->update_branch_dynamic_parameters(my_chain.at(i), name_index);

        name_index = name_index +
                     my_chain.at(i)->get_configuration_space_positions().size();
    }
}

/****************************************************************
******DQ_BRANCHEDCOPPELIASIMLEGACYROBOT PUBLIC CLASS METHODS*****
*****************************************************************/

/**
 * @brief Creates a DQ_BranchedCoppeliaSimLegacyRobot object with empty
 *        link names.
 *
 * @param base_robot_name The base name of the robot in the CoppeliaSim scene.
 * @param robot_dof The number of DoF of the robot in the CoppeliaSim scene.
 * @param robot_name The name of the robot in the CoppeliaSim scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 */
DQ_BranchedCoppeliaSimLegacyRobot::DQ_BranchedCoppeliaSimLegacyRobot(
    const std::string &base_robot_name,
    const int &robot_dof,
    const std::string& robot_name,
    const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr):
    DQ_SerialVrepRobot(base_robot_name, robot_dof, robot_name, vrep_interface_sptr)
{
    // From the second copy of the robot and onward, VREP appends a #number in the
    // robot's name. We check here if the robot is called by the correct name and
    // assign an index that will be used to correctly infer the robot's link labels.
    std::vector<std::string> splited_name = _strsplit(this->robot_name_,'#');
    std::string robot_label = splited_name[0];

    if(robot_label.compare(std::string(base_robot_name)) != 0)
    {
        throw std::runtime_error(std::string("Expected ") + base_robot_name);
    }

    std::string robot_index("");
    if (splited_name.size() > 1){
        robot_index = "#" + splited_name[1];
    }

    // Initialize link names
    for(int i=1;i<(robot_dof+1);i++){
        std::string link_name = robot_label + std::string("_link") +
                                    std::to_string(i + 1) + robot_index;
        link_names_.push_back(link_name);
    }
}

}//namespace DQ_dynamics
