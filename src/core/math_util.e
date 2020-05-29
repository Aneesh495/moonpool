-- Moonpool Subsea Robotics Core
-- High-Precision Mathematical Utilities and Numerical Solvers
-- OpenEuphoria v4.1+ Specification

namespace moonpool_math

include std/math.e
include src/core/constants.e

-- Clamp an atom value within lower and upper bounds
public function clamp(atom val, atom min_val, atom max_val)
    if val < min_val then
        return min_val
    elsif val > max_val then
        return max_val
    else
        return val
    end if
end function

-- Linear interpolation between a and b with ratio t [0, 1]
public function lerp(atom a, atom b, atom t)
    return a + (b - a) * t
end function

-- Wrap angle in radians to [-PI, +PI]
public function wrap_pi(atom angle_rad)
    while angle_rad > PI do
        angle_rad -= TWO_PI
    end while
    while angle_rad < -PI do
        angle_rad += TWO_PI
    end while
    return angle_rad
end function

-- Wrap angle in degrees to [0.0, 360.0)
public function wrap_360(atom angle_deg)
    while angle_deg >= 360.0 do
        angle_deg -= 360.0
    end while
    while angle_deg < 0.0 do
        angle_deg += 360.0
    end while
    return angle_deg
end function

-- Floating point approximate equality check
public function approx_equal(atom a, atom b, atom tol)
    if tol <= 0.0 then
        tol = EPSILON
    end if
    return abs(a - b) <= tol
end function

-- Arithmetic mean of a sequence of atoms
public function mean(sequence vals)
    atom sum_val
    integer n
    n = length(vals)
    if n = 0 then
        return 0.0
    end if
    sum_val = 0.0
    for i = 1 to n do
        sum_val += vals[i]
    end for
    return sum_val / n
end function

-- Sample variance of a sequence of atoms (Bessel-corrected, N-1)
public function variance(sequence vals)
    atom m, diff, sum_sq
    integer n
    n = length(vals)
    if n < 2 then
        return 0.0
    end if
    m = mean(vals)
    sum_sq = 0.0
    for i = 1 to n do
        diff = vals[i] - m
        sum_sq += diff * diff
    end for
    return sum_sq / (n - 1)
end function

-- Standard deviation
public function std_dev(sequence vals)
    return sqrt(variance(vals))
end function

-- Covariance between two sequences of equal length
public function covariance(sequence x, sequence y)
    atom mx, my, sum_prod
    integer n
    n = length(x)
    if n < 2 or n != length(y) then
        return 0.0
    end if
    mx = mean(x)
    my = mean(y)
    sum_prod = 0.0
    for i = 1 to n do
        sum_prod += (x[i] - mx) * (y[i] - my)
    end for
    return sum_prod / (n - 1)
end function

-- Polynomial evaluation using Horner's method:
-- coeffs = {c0, c1, c2, ...} computes c0 + c1*x + c2*x^2 + ...
public function polynomial_eval(sequence coeffs, atom x)
    atom result_val
    integer n
    n = length(coeffs)
    if n = 0 then
        return 0.0
    end if
    result_val = coeffs[n]
    for i = n - 1 to 1 by -1 do
        result_val = result_val * x + coeffs[i]
    end for
    return result_val
end function

-- Simpson's 1/3 rule numerical integration of a discrete sequence of samples
-- with uniform spacing dx
public function simpson_integral(sequence samples, atom dx)
    integer n
    atom total_val
    n = length(samples)
    if n < 3 then
        if n = 2 then
            return 0.5 * (samples[1] + samples[2]) * dx
        elsif n = 1 then
            return samples[1] * dx
        else
            return 0.0
        end if
    end if
    
    total_val = samples[1] + samples[n]
    for i = 2 to n - 1 do
        if remainder(i, 2) = 0 then
            total_val += 4.0 * samples[i]
        else
            total_val += 2.0 * samples[i]
        end if
    end for
    return (dx / 3.0) * total_val
end function

-- Bisection root finding algorithm for single-variable continuous functions
-- func_id: routine id of the function f(x) -> atom
public function bisection_root(integer func_id, atom a, atom b, atom tol, integer max_iter)
    atom fa, fb, c, fc
    integer iter
    fa = call_func(func_id, {a})
    fb = call_func(func_id, {b})
    if fa * fb > 0.0 then
        return (a + b) / 2.0
    end if
    
    iter = 0
    while (b - a) / 2.0 > tol and iter < max_iter do
        c = (a + b) / 2.0
        fc = call_func(func_id, {c})
        if fc = 0.0 or (b - a) / 2.0 < tol then
            return c
        end if
        if fa * fc < 0.0 then
            b = c
            fb = fc
        else
            a = c
            fa = fc
        end if
        iter += 1
    end while
    return (a + b) / 2.0
end function
