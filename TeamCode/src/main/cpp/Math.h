#pragma once

#include <cmath>

namespace Math
{
    constexpr double EPSILON = 1e-6;

    inline bool epsilonEquals(const double lhs, const double rhs) noexcept
    {
        return abs(lhs - rhs) < EPSILON;
    }
}
