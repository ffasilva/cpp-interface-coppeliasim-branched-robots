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

#include <dqdynamics/interfaces/coppeliasim/zmq/robots/FreeFlyingCoppeliaSimZMQRobot.h>

#include <dqdynamics/robots/FreeFlyingRobotDynamics.h>

#include <dqrobotics/DQ.h>

namespace DQ_dynamics
{

/**
 * @brief Constructor of the FreeFlyingCoppeliaSimZMQRobot class
 *
 * @param robot_name The name of robot used on the CoppeliaSim scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 * @return A FreeFlyingCoppeliaSimZMQRobot object representing the
 *         free-flying robot in the CoppeliaSim scene.
 *
 *  Example:
 *          auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *          vi->connect();
 *          vi->start_simulation();
 *          FreeFlyingCoppeliaSimZMQRobot free_flying_coppeliasim_robot(
 *              "/free_flying",
 *              vi);
 *          vi->stop_simulation();
 */
FreeFlyingCoppeliaSimZMQRobot::FreeFlyingCoppeliaSimZMQRobot(
    const std::string& robot_name,
    const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental>&
        vrep_interface_sptr):
    DQ_BranchedCoppeliaSimZMQRobot(robot_name, vrep_interface_sptr)
{
    // Fix the joint names automatically assigned by the
    // DQ_CoppeliaSimRobotZMQ class
    this->jointnames_.at(0) = "/free_flying/free_flying_prism_joint_x";
    this->jointnames_.at(1) = "/free_flying/free_flying_prism_joint_y";
    this->jointnames_.at(2) = "/free_flying/free_flying_prism_joint_z";
    this->jointnames_.at(3) = "/free_flying/free_flying_rev_joint_x";
    this->jointnames_.at(4) = "/free_flying/free_flying_rev_joint_y";
    this->jointnames_.at(5) = "/free_flying/free_flying_rev_joint_z";

    // Fix base frame name automatically assigned by the
    // DQ_CoppeliaSimRobotZMQ class
    base_frame_name_ = "/free_flying/free_flying_base";

    // Set the name of the robot's force sensor
    force_sensor_name_ = "/free_flying/force_sensor";
}

/**
 * @brief Returns the vec8() of the unit dual quaternion representing the pose
 *        of the free-flying robot in the CoppeliaSim scene. This method is
 *        kept for compatibility with other classes that rely on calls to
 *        get_configuration_space() in which the return is a vector of real
 *        numbers.
 *
 * @return A VectorXd with the coefficients of the unit dual quaternion
 *         representing the pose of the free-flying robot in the CoppeliaSim
 *         scene.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      FreeFlyingCoppeliaSimZMQRobot free_flying_coppeliasim_robot(
 *          "/free_flying",
 *          vi);
 *      VectorXd q = free_flying_coppeliasim_robot.get_configuration_space();
 *      vi->stop_simulation();
 */
VectorXd FreeFlyingCoppeliaSimZMQRobot::get_configuration_space()
{
    // Read the robot's pose from CoppeliaSim
    const auto coppeliasim_sptr = this->_get_interface_sptr();
    DQ_robotics::DQ x = DQ_robotics::normalize(
        coppeliasim_sptr->get_object_pose(base_frame_name_));

    // Vectorizes the pose for compatibility with other classes that rely
    // on calls to get_configuration_space() in which the return is a
    // vector of real numbers
    VectorXd x_vec = x.vec8();

    return x_vec;
}

/**
 * @brief Returns the vec8() of the pure dual quaternion representing the
 *        twist of the free-flying robot in the CoppeliaSim scene. This
 *        method is kept for compatibility with other classes that rely
 *        on calls to get_configuration_space_velocities() in which the
 *        return is a vector of real numbers.
 *
 * @return A VectorXd with the coefficients of the pure dual quaternion
 *         representing the twist of the free-flying robot in the CoppeliaSim
 *         scene.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      FreeFlyingCoppeliaSimZMQRobot free_flying_coppeliasim_robot(
 *          "/free_flying",
 *          vi);
 *      VectorXd dq =
 *          free_flying_coppeliasim_robot.get_configuration_space_velocities();
 *      vi->stop_simulation();
 */
VectorXd FreeFlyingCoppeliaSimZMQRobot::get_configuration_space_velocities()
{
    // Read the robot's twist from CoppeliaSim
    const auto coppeliasim_sptr = this->_get_interface_sptr();
    // DQ_robotics::DQ xi = coppeliasim_sptr->get_twist(base_frame_name_,
    //     DQ_CoppeliaSimInterfaceZMQExperimental::REFERENCE::BODY_FRAME);
    DQ_robotics::DQ xi = coppeliasim_sptr->get_twist(base_frame_name_);

    // Vectorizes the twist for compatibility with other classes that rely
    // on calls to get_configuration_space_velocities() in which the return
    // is a vector of real numbers
    VectorXd xi_vec = xi.vec8();

    return xi_vec;
}

/**
 * @brief Returns the vec6() of the pure dual quaternion representing the
 *        wrench read from the robot's force sensor in the CoppeliaSim
 *        scene. This method is kept for compatibility with other classes
 *        that rely on calls to get_configuration_space_torques() in which
 *        the return is a vector of real numbers.
 *
 * @return A VectorXd with the coefficients of the pure dual quaternion
 *         representing the wrench read from the robot's force sensor
 *         in the CoppeliaSim scene.
 *
 *  Example:
 *      auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
 *      vi->connect();
 *      vi->start_simulation();
 *      FreeFlyingCoppeliaSimZMQRobot free_flying_coppeliasim_robot(
 *          "/free_flying",
 *          vi);
 *      VectorXd tau =
 *          free_flying_coppeliasim_robot.get_configuration_space_torques();
 *      vi->stop_simulation();
 */
VectorXd FreeFlyingCoppeliaSimZMQRobot::get_configuration_space_torques()
{
    // Read the wrench from the robot's force sensor in CoppeliaSim
    const auto coppeliasim_sptr = this->_get_interface_sptr();
    DQ_robotics::DQ sensor_wrench_in_sensor =
        coppeliasim_sptr->get_sensor_wrench(force_sensor_name_);

    // Conjugate the wrench because the generalized forces are
    // given by the wrench acting on the robot rather than by the
    // reaction forces read by the sensor
    DQ_robotics::DQ wrench_in_sensor = sensor_wrench_in_sensor.conj();

    // Express the wrench in the inertial frame
    DQ_robotics::DQ sensor_pose = coppeliasim_sptr->get_object_pose(
        force_sensor_name_);
    DQ_robotics::DQ wrench = DQ_robotics::Ad(sensor_pose.P(),
                                             wrench_in_sensor);

    // Vectorizes the wrench for compatibility with other classes that rely
    // on calls to get_configuration_space_torques() in which the return
    // is a vector of real numbers
    return wrench.vec6();
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

    // Set the robot configuration with CoppeliaSim values
    VectorXd q_read = this->get_configuration_space();
    VectorXd dq_read = this->get_configuration_space_velocities();
    VectorXd ddq_read = VectorXd::Zero(8,1);

    dyn.set_configurations(q_read, dq_read, ddq_read);

    // Update base and reference frame with CoppeliaSim values
    const DQ_robotics::DQ& base_frame = DQ_robotics::normalize(
        this->_get_interface_sptr()->get_object_pose(this->base_frame_name_));
    dyn.set_reference_frame(base_frame);
    dyn.set_base_frame(base_frame);

    // Update dynamic parameters with CoppeliaSim information
    this->update_base_dynamic_parameters(dyn);

    return dyn;
}

}//namespace DQ_dynamics
