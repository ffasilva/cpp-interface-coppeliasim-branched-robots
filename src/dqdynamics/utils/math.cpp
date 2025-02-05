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

#include <dqdynamics/utils/math.h>

namespace DQ_dynamics
{

/****************************************************************
**************NAMESPACE ONLY FUNCTIONS***************************
*****************************************************************/

/**
 * @brief Returns the variance of the elements of a vector.
 * @param q A VectorXd with double elements.
 * @return The the variance of the elements of the input vector.
 */
double variance(const VectorXd &q)
{
    double sum = 0.0;
    double temp = 0.0;
    double q_mean = q.mean();

    int n = q.rows();
    for ( int i=0; i<n; i++){
        temp = pow((q(i) - q_mean), 2);
        sum += temp;
    }
    double var = sum/(n - 2);

    return var;
}

}//namespace DQ_dynamics
