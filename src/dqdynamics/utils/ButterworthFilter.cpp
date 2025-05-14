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
 *        roots. Emulate Matlab 'poly' function.
 * @param roots A vector with complex elements representing the roots of
 *        the polynomial.
 * @return A vector with complex elements representing the coefficients
 *         of the polynomial with the specified roots.
 */
std::vector<std::complex<double>> ButterworthFilter::poly(
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

std::complex<double> ButterworthFilter::sum(
    const std::vector<std::complex<double>>& vector)
{
    return std::accumulate(vector.begin(), vector.end(), 0.0i);
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
}

/**
 * @brief Calculates the coefficients of a Butterworth filter for the
 *        specification given in the creation of the ButterworthFilter
 *        object. This method is an adaptation of Tom's conversion to
 *        to C++ of Neil Robertson's MATLAB original implementation:
 *          https://www.dsprelated.com/showarticle/1119.php
 * @return The coefficients of a Butterworth filter for the specification
 *         given in the creation of the ButterworthFilter object.
 */
std::tuple<std::vector<double>, std::vector<double>>
ButterworthFilter::get_butterworth_coefficients()
{
    const int& N = filter_order_;
    std::vector<std::complex<double>> pa(N);
    std::vector<std::complex<double>> p(N);
    std::vector<std::complex<double>> q(N, -1.0);

    const double& fc = cutoff_frequency_;
    const double& fs = sampling_frequency_;
    assert(fc < fs / 2); // Cutoff frequency must be less that fs/2

    // I. Find poles of analog filter
    const double& pi = M_PI;
    for (int i = 0; i < N; i++)
    {
        int k = i + 1;
        double theta = (2 * k - 1) * pi / (2 * N);
        pa[i] = -sin(theta) + 1.0i * cos(theta);
    }

    // II. Scale poles in frequency
    double Fc = fs / pi * tan(pi * fc / fs);
    for (size_t i=0; i<pa.size(); i++)
        pa[i] *= 2 * pi * Fc;

    // III. Find coeffs of digital filter poles and zeros in the z plane
    for (int i = 0; i < N; i++)
        p[i] = (1.0 + pa[i] / (2 * fs)) / (1.0 - pa[i] / (2 * fs));

    auto a = poly(p);
    for (size_t i = 0; i < a.size(); i++)
        a[i] = a[i].real();

    auto b = poly(q);
    auto K = sum(a) / sum(b);
    for (size_t i = 0; i < b.size(); i++)
        b[i] *= K;

    for (auto coeff : a)
        coeff_a_.push_back(coeff.real());
    for (auto coeff : b)
        coeff_b_.push_back(coeff.real());

    return std::make_tuple(coeff_a_, coeff_b_);;
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
    size_t n = p.size() + q.size() - 1;
    Polynomial result(n);
    for (size_t i = 0; i < p.size(); i++)
        for (size_t j = 0; j < q.size(); j++)
            result[i + j] += p[i] * q[j];
    return result;
}


}//namespace DQ_dynamics
