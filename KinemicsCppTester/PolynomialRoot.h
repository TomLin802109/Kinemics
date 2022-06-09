#pragma once
#include <complex>

namespace Polynomial {
    std::complex<double>* solve_quadratic_eq(double a, double b, double c);
    std::complex<double>* solve_biquadratic_eq(double a, double b, double c, double d, double e);
}