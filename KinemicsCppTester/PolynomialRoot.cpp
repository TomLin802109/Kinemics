#pragma once
#include "PolynomialRoot.h"

namespace Polynomial {
    std::complex<double>* solve_quadratic_eq(double a, double b, double c) {
        std::complex<double>* ans = new std::complex<double>[2];
        std::complex<double> complex_a{ a };
        std::complex<double> complex_b{ b };
        std::complex<double> complex_c{ c };
        ans[0] = (-complex_b + sqrt(pow(complex_b, 2) - 4.0 * complex_a * complex_c)) / (2.0 * complex_a);
        ans[1] = (-complex_b - sqrt(pow(complex_b, 2) - 4.0 * complex_a * complex_c)) / (2.0 * complex_a);
        return ans;
    }
    std::complex<double>* solve_biquadratic_eq(double a, double b, double c, double d, double e)
    {
        std::complex<double>* ans = new std::complex<double>[4];
        std::complex<double> complex_a{ a };
        std::complex<double> complex_b{ b };
        std::complex<double> complex_c{ c };
        std::complex<double> complex_d{ d };
        std::complex<double> complex_e{ e };
        complex_a = 1.0 / complex_a;
        complex_b *= complex_a;
        complex_c *= complex_a;
        complex_d *= complex_a;
        complex_e *= complex_a;
        std::complex<double> P = (complex_c * complex_c + 12.0 * complex_e - 3.0 * complex_b * complex_d) / 9.0;
        std::complex<double> Q = (27.0 * complex_d * complex_d + 2.0 * complex_c * complex_c * complex_c + 27.0 * complex_b * complex_b * complex_e - 72.0 * complex_c * complex_e - 9.0 * complex_b * complex_c * complex_d) / 54.0;
        std::complex<double> D = sqrt(Q * Q - P * P * P);
        std::complex<double> u = Q + D;
        std::complex<double> v = Q - D;
        if (v.real() * v.real() + v.imag() * v.imag() > u.real() * u.real() + u.imag() * u.imag())
        {
            u = pow(v, 1 / 3.0);
        }
        else
        {
            u = pow(u, 1 / 3.0);
        }
        std::complex<double> y;
        if (u.real() * u.real() + u.imag() * u.imag() > 0.0)
        {
            v = P / u;
            std::complex<double> o1(-0.5, +0.86602540378443864676372317075294);
            std::complex<double> o2(-0.5, -0.86602540378443864676372317075294);
            std::complex<double>& yMax = ans[0];
            double m2 = 0.0;
            double m2Max = 0.0;
            int iMax = -1;
            for (int i = 0; i < 3; ++i)
            {
                y = u + v + complex_c / 3.0;
                u *= o1;
                v *= o2;
                complex_a = complex_b * complex_b + 4.0 * (y - complex_c);
                m2 = complex_a.real() * complex_a.real() + complex_a.imag() * complex_a.imag();
                if (0 == i || m2Max < m2)
                {
                    m2Max = m2;
                    yMax = y;
                    iMax = i;
                }
            }
            y = yMax;
        }
        else
        {//一元三次方程，三重根
            y = complex_c / 3.0;
        }
        std::complex<double> m = sqrt(complex_b * complex_b + 4.0 * (y - complex_c));
        if (m.real() * m.real() + m.imag() * m.imag() >= DBL_MIN)
        {
            std::complex<double> n = (complex_b * y - 2.0 * complex_d) / m;
            complex_a = sqrt((complex_b + m) * (complex_b + m) - 8.0 * (y + n));
            ans[0] = (-(complex_b + m) + complex_a) / 4.0;
            ans[1] = (-(complex_b + m) - complex_a) / 4.0;
            complex_a = sqrt((complex_b - m) * (complex_b - m) - 8.0 * (y - n));
            ans[2] = (-(complex_b - m) + complex_a) / 4.0;
            ans[3] = (-(complex_b - m) - complex_a) / 4.0;
        }
        else
        {
            complex_a = sqrt(complex_b * complex_b - 8.0 * y);
            ans[0] =
                ans[1] = (-complex_b + complex_a) / 4.0;
            ans[2] =
                ans[3] = (-complex_b - complex_a) / 4.0;
        }
        return ans;
    }
}