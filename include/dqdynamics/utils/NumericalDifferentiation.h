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
        - Adapted from Juan José Quiroz Omanã's MATLAB implementation
            https://github.com/ffasilva/dynamic-modular-composition-matlab/blob/2ef8bc8b21dd1490d908876cdae144cf9d8f375e/utils/NumericalDifferentiation.m
*/



#pragma once

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;

namespace DQ_dynamics
{

class NumericalDifferentiation
{
private:
    double sampling_time_ = 0.01;
    double h2_ = 0.04;
    int h2_scale_ = 4; // recommended: 2, 4, 6, 8
    double initial_value_ = 0.0;
    int extrapol_ = 2;

    double diff_sffd_richardson_(const VectorXd& q_joint);
public:
    NumericalDifferentiation() = delete;
    NumericalDifferentiation(const double& sampling_time);
    virtual ~NumericalDifferentiation() = default;

    void set_initial_value(const double& initial_value);
    void set_h2_scale(const int& h2_scale);

    VectorXd vector_differentiation(const MatrixXd& Q_joints);
};

}//namespace DQ_dynamics
