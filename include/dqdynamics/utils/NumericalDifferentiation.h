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

#include<dqrobotics/DQ.h>

using namespace Eigen;

namespace DQ_dynamics
{

class NumericalDifferentiation
{
public:
    NumericalDifferentiation() = delete;
    virtual ~NumericalDifferentiation() = default;
};

}//namespace DQ_dynamics
