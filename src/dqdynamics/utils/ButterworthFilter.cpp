/**
(C) Copyright 2024-2025 DQ Dynamics Developers

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
        - Adapted from Tom's implementation
            https://thecodehound.com/butterworth-filter-design-in-c/
*/

#include <dqdynamics/utils/ButterworthFilter.h>

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <numeric>

namespace DQ_dynamics
{

/****************************************************************
**************DQ_DYNAMICS PRIVATE CLASS METHODS******************
*****************************************************************/

/**
 * @brief Calculates the coefficients of the polynomial with the specified
 *        roots. Emulate Matlab 'poly()' function.
 * @param roots A vector with complex elements representing the roots of
 *        the polynomial.
 * @return A vector with complex elements representing the coefficients
 *         of the polynomial with the specified roots.
 */
std::vector<std::complex<double>> ButterworthFilter::_poly(
    std::vector<std::complex<double>> roots)
{
    Polynomial result{1.0};
    for (auto root : roots)
    {
        Polynomial factor({-root, 1.0});
        result = result*factor;
    }

    // Matlab returns the highest order coefficients first
    std::reverse(result.begin(), result.end());
    return result;
}

/**
 * @brief Sums the coefficients of a given vector.
 * @param vector A vector of complex numbers.
 * @return A complex number representing the sum of the coefficients of
 *         the given vector. Emulate Matlab 'sum()' function.
 */
std::complex<double> ButterworthFilter::_sum(
    const std::vector<std::complex<double>>& vector)
{
    return std::accumulate(vector.begin(), vector.end(), 0.0i);
}

/**
 * @brief Calculates the coefficients of a Butterworth filter for the
 *        specification given in the creation of the ButterworthFilter
 *        object. This method is an adaptation of Tom's conversion to
 *        to C++ of Neil Robertson's MATLAB original implementation:
 *          https://www.dsprelated.com/showarticle/1119.php
 *        Reduce, Reuse, and Recycle!
 * @return The coefficients of a Butterworth filter for the specification
 *         given in the creation of the ButterworthFilter object.
 */
void ButterworthFilter::_calculate_butterworth_coefficients()
{
    const int& N = filter_order_;
    std::vector<std::complex<double>> pa(N);
    std::vector<std::complex<double>> p(N);
    std::vector<std::complex<double>> q(N, -1.0);

    const double& fc = cutoff_frequency_;
    const double& fs = sampling_frequency_;
    assert(fc < fs / 2); // cutoff frequency must be less that fs/2

    // I. Find poles of analog filter
    const double& pi = M_PI;
    for (int i = 0; i < N; i++){
        int k = i + 1;
        double theta = (2*k - 1)*pi/(2*N);
        pa[i] = -sin(theta) + 1.0i * cos(theta);
    }

    // II. Scale poles in frequency
    const double Fc = fs/pi*tan(pi*fc/fs);
    for (size_t i=0; i<pa.size(); i++){
        pa[i] *= 2*pi*Fc;
    }

    // III. Find coeffs of digital filter poles and zeros in the z plane
    for (int i = 0; i < N; i++){
        p[i] = (1.0 + pa[i]/(2*fs))/(1.0 - pa[i]/(2*fs));
    }

    auto a = this->_poly(p);
    for (size_t i = 0; i < a.size(); i++){
        a[i] = a[i].real();
    }

    auto b = this->_poly(q);
    const auto K = this->_sum(a)/this->_sum(b);
    for (size_t i = 0; i < b.size(); i++){
        b[i] *= K;
    }

    for (auto coeff : a){
        coeff_a_.push_back(coeff.real());
    }
    for (auto coeff : b){
        coeff_b_.push_back(coeff.real());
    }
}

/****************************************************************
**************DQ_DYNAMICS PUBLIC CLASS METHODS*******************
*****************************************************************/

/**
 * @brief Creates a ButterworthFilter object for a given specification.
 * @param filter_order An integer representing the order of the IIR filter.
 * @param cutoff_frequency An integer representing the desired cutoff
 *        frequency. The cutoff frequency is the frequency at which the
 *        magnitude response of the filter is 1/sqrt(2)​.
 * @param sampling_time An integer represeting the sampling time of the
 *        input signal.
 */
ButterworthFilter::ButterworthFilter(const int& filter_order,
                                     const double& cutoff_frequency,
                                     const double& sampling_time)
{
    filter_order_ = filter_order;
    cutoff_frequency_ = cutoff_frequency;
    sampling_frequency_ = 1.0/sampling_time;

    this->_calculate_butterworth_coefficients();
}

/**
 * @brief Returns the coefficients of a Butterworth filter for the
 *        specification given in the creation of the ButterworthFilter
 *        object.
 * @return The coefficients of a Butterworth filter for the specification
 *         given in the creation of the ButterworthFilter object.
 */
std::tuple<std::vector<double>, std::vector<double>>
ButterworthFilter::get_butterworth_coefficients()
{
    return std::make_tuple(coeff_a_, coeff_b_);
}

/**
 * @brief Filters a given signal using the Butterworth filter specified
 *        in the ButterworthFilter object creation.
 * @param signal A vector of doubles with elements representing samples
 *        of a signal.
 * @return The signal filtered by a Butterworth filter specified in the
 *         ButterworthFilter object creation.
 */
VectorXd ButterworthFilter::filter(const VectorXd& signal)
{
    const int n = signal.size();
    const int m = coeff_b_.size();
    // For an input signal x with n samples and a Butterworth filter of order m,
    // calculate the i-th element of the output signal y as:
    // y[i] = b0*x[i] + b1*x[i-1] + ... + bm*x[i-m] - a1*y[i-1] - ... - -am*y[i-m]
    filtered_signal = VectorXd::Zero(n);
    for (int i=0; i<n; i++){
        for (int j=0; j<m; j++){
            if ((i-j) >= 0){
                // y[i] = b0*x[i] + b1*x[i-1] + ... + bm*x[i-m]
                filtered_signal(i) = filtered_signal(i) +
                                     coeff_b_.at(j)*signal(i-j);
                if (((i-(j+1)) >= 0) && (j < (m - 1))){
                    // y[i] = -a1*y[i-1] - a2*y[i-2] - ... - -am*y[i-m]
                    filtered_signal(i) = filtered_signal(i) -
                                         coeff_a_.at(j+1)*filtered_signal(i-(j+1));
                }
            } else {
                break;
            }
        }
    }

    return filtered_signal;
}

/****************************************************************
**************NAMESPACE ONLY FUNCTIONS***************************
*****************************************************************/

/**
 * @brief Multiplies two Polynomials.
 * @param p A Polynomial.
 * @param q A Polynomial.
 * @return The result of the multiplication of the two given Polynomials.
 */
Polynomial operator*(const Polynomial& p, const Polynomial& q)
{
    const size_t n = p.size() + q.size() - 1;
    Polynomial result(n);
    for (size_t i = 0; i < p.size(); i++){
        for (size_t j = 0; j < q.size(); j++){
            result[i + j] += p[i]*q[j];
        }
    }

    return result;
}


}//namespace DQ_dynamics
