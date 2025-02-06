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
 * @brief Returns the CMC between two curves of same length, as
 *        described in:
 *          Ferrari, A., Cutti, A. G., and Cappello, A. (2010).
 *          ‘A new formulation of the coefficient of multiple correlation
 *           to assess the similarity of waveforms measured synchronously
 *           by different motion analysis protocols’.
 *          Gait and Posture, 31(4), 540–542.
 *          https://doi.org/10.1016/j.gaitpost.2010.02.009.
 *
 *        Adapted from the MATLAB implementation by:
 *          Ana Christine de Oliveira (ana.oliveira@utexas.edu)
 * @param waveform1 A VectorXd vector with data from the first waveform.
 * @param waveform2 A VectorXd vector with data from the second waveform.
 * @return The CMC between two curves, given by a value between 0 and 1.
 *         The closer to 1, the more similar the waverforms are. A NaN
 *         value represents dissimilarity.
 */
double cmc(const VectorXd& waveform1, const VectorXd& waveform2)
{
    int frames1 = waveform1.rows();
    int frames2 = waveform2.rows();
    if (frames1 != frames2){
        throw std::runtime_error("The waveforms should be of the same "
                                 "size!");
    }

    VectorXd Yf_bar = 1.0/2.0*(waveform1 + waveform2);
    auto partial_num = ((waveform1 - Yf_bar).array().pow(2) +
                        (waveform2 - Yf_bar).array().pow(2)).sum();
    double numerator = partial_num/(frames1);

    double Y_bar = 1.0/(2.0*frames1)*(waveform1.sum() + waveform2.sum());
    auto partial_den = ((waveform1.array() - Y_bar).array().pow(2) +
                        (waveform2.array() - Y_bar).array().pow(2)).sum();
    double denominator = partial_den/((2*frames2 - 1));

    double ret = sqrt(1 - numerator/denominator);
    if (std::isnan(ret)){ // a NaN value represents dissimilarity
        ret = 0.0;
    }

    return ret;
}

/**
 * @brief Returns the variance of the elements of a vector.
 * @param q A VectorXd with double elements.
 * @return The variance of the elements of the input vector.
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
