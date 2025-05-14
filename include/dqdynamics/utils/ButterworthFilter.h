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



#pragma once

#include <complex>
#include <vector>
#include <tuple>

using namespace std::literals::complex_literals;

using Polynomial = std::vector<std::complex<double>>;

namespace DQ_dynamics
{

class ButterworthFilter
{
private:
    int filter_order_;
    double sampling_frequency_, cutoff_frequency_;
    std::vector<double> coeff_a_, coeff_b_;

    static std::vector<std::complex<double>> poly(
        std::vector<std::complex<double>> roots);

    static std::complex<double>
    sum(const std::vector<std::complex<double>>& vector);
public:
    ButterworthFilter() = delete;
    ButterworthFilter(const int& filter_order,
                      const double& cutoff_frequency,
                      const double& sampling_time);
    virtual ~ButterworthFilter() = default;

    std::tuple<std::vector<double>, std::vector<double>>
    get_butterworth_coefficients();
};

/****************************************************************
**************NAMESPACE ONLY FUNCTIONS***************************
*****************************************************************/

Polynomial operator*(const Polynomial& p, const Polynomial& q);

}//namespace DQ_dynamics
