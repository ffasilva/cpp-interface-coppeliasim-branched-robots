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

#include <dqdynamics/utils/NumericalDifferentiation.h>

#include <cmath>

namespace DQ_dynamics
{

/****************************************************************
**************DQ_DYNAMICS PRIVATE CLASS METHODS******************
*****************************************************************/

/**
 * @brief Returns the numerical differentation using SFFD and Richardson
 *        extrapolation.
 * @param q_joint A VectorXd representing the joint values throughout
 *        the last n time steps.
 * @return The numerical differentiation of the joint value at the nth
 *         time step.
 */
double NumericalDifferentiation::diff_sffd_richardson_(const VectorXd& q_joint)
{
    h2_ = sampling_time_*h2_scale_;

    int n = q_joint.rows();
    double dq_joint;
    if (n > 2*h2_scale_){
        double x31 = q_joint(last-2);
        double x21 = q_joint(last-1);
        double x11 = q_joint(last);

        double x32 = q_joint(last-2*h2_scale_);
        double x22 = q_joint(last-h2_scale_);
        double x12 = q_joint(last);

        double g1 = (3*x11 - 4*x21 + x31)/(2*sampling_time_);
        double g2 = (3*x12 - 4*x22 + x32)/(2*h2_);

        double ratio_st_h2 = sampling_time_/h2_;
        double a =  ((pow(ratio_st_h2, extrapol_))*g2 - g1);
        double b =  ((pow(ratio_st_h2, extrapol_)) - 1);

        dq_joint = a/b;
    } else { // we use the Euler method when there isn't enough data
        dq_joint = (q_joint(last) - q_joint(last-1))/sampling_time_;
    }

    return dq_joint;
}

/****************************************************************
**************DQ_DYNAMICS PUBLIC CLASS METHODS*******************
*****************************************************************/

/**
 * @brief Creates a NumericalDifferentiation object for given a
 *        sampling time.
 */
NumericalDifferentiation::NumericalDifferentiation(const double &sampling_time)
{
    sampling_time_ = sampling_time;
}

/**
 * @brief Sets the time initial value for the numerical time derivative.
 * @param initial_value A double representing the estimated initial value
 *        for the numerical time derivative.
 */
void NumericalDifferentiation::set_initial_value(const double& initial_value)
{
    initial_value_ = initial_value;
}

/**
 * @brief Sets the scale from the sampling time to h2.
 * @param h2_scale An int representing the scale from the sampling time
 *        to h2.
 */
void NumericalDifferentiation::set_h2_scale(const int& h2_scale)
{
    h2_scale_ = h2_scale;
}

/**
 * @brief Returns the numerical differentation of the values of the
 *        robot's joints using SFFD and Richardson extrapolation.
 * @param Q_joints A mxn MatrixXd representing the historic values of
 *        the m joints throughout the last n time steps.
 * @return The numerical differentiation of the joint values at the nth
 *         time step.
 */
VectorXd NumericalDifferentiation::vector_differentiation(const MatrixXd& Q_joints)
{
    int m = Q_joints.rows();
    int n = Q_joints.cols();
    VectorXd dq_joints = VectorXd::Zero(m);
    if (n == 1){ // there's only one data point for each joint
        for (int i=0; i<m; i++){
            dq_joints(i) = (Q_joints(i,0) - initial_value_)/sampling_time_;
        }
    } else{
        for (int i=0; i<m; i++){ // calculate the time derivative of the ith joint
            dq_joints(i) = diff_sffd_richardson_(Q_joints(i,all).transpose());
        }
    }

    return dq_joints;
}

}//namespace DQ_dynamics
