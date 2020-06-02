-- Moonpool Subsea Robotics Core
-- Linear Algebra, Matrix Factorization, and Decomposition Library
-- OpenEuphoria v4.1+ Specification

namespace moonpool_matrix

include std/math.e
include src/core/constants.e

-- Create an (r x c) matrix filled with zeros
public function mat_zeros(integer r, integer c)
    sequence M
    M = repeat(0, r)
    for i = 1 to r do
        M[i] = repeat(0.0, c)
    end for
    return M
end function

-- Create an (n x n) identity matrix
public function mat_eye(integer n)
    sequence I
    I = mat_zeros(n, n)
    for i = 1 to n do
        I[i][i] = 1.0
    end for
    return I
end function

-- Create a diagonal matrix from a vector
public function mat_diag(sequence v)
    integer n
    sequence D
    n = length(v)
    D = mat_zeros(n, n)
    for i = 1 to n do
        D[i][i] = v[i]
    end for
    return D
end function

-- Vector addition
public function vec_add(sequence u, sequence v)
    integer n
    sequence w
    n = length(u)
    w = repeat(0.0, n)
    for i = 1 to n do
        w[i] = u[i] + v[i]
    end for
    return w
end function

-- Vector subtraction
public function vec_sub(sequence u, sequence v)
    integer n
    sequence w
    n = length(u)
    w = repeat(0.0, n)
    for i = 1 to n do
        w[i] = u[i] - v[i]
    end for
    return w
end function

-- Vector scalar multiplication
public function vec_scale(sequence u, atom s)
    integer n
    sequence w
    n = length(u)
    w = repeat(0.0, n)
    for i = 1 to n do
        w[i] = u[i] * s
    end for
    return w
end function

-- Vector dot product
public function vec_dot(sequence u, sequence v)
    atom sum_val
    integer n
    n = length(u)
    sum_val = 0.0
    for i = 1 to n do
        sum_val += u[i] * v[i]
    end for
    return sum_val
end function

-- 3D vector cross product: u x v
public function vec_cross(sequence u, sequence v)
    return {
        u[2] * v[3] - u[3] * v[2],
        u[3] * v[1] - u[1] * v[3],
        u[1] * v[2] - u[2] * v[1]
    }
end function

-- Vector Euclidean L2 norm
public function vec_norm(sequence v)
    return sqrt(vec_dot(v, v))
end function

-- Vector normalization (unit vector)
public function vec_normalize(sequence v)
    atom n
    n = vec_norm(v)
    if n < EPSILON then
        return repeat(0.0, length(v))
    end if
    return vec_scale(v, 1.0 / n)
end function

-- Matrix transpose: M^T
public function mat_transpose(sequence M)
    integer r, c
    sequence MT
    r = length(M)
    if r = 0 then
        return {}
    end if
    c = length(M[1])
    MT = mat_zeros(c, r)
    for i = 1 to r do
        for j = 1 to c do
            MT[j][i] = M[i][j]
        end for
    end for
    return MT
end function

-- Matrix addition: A + B
public function mat_add(sequence A, sequence B)
    integer r, c
    sequence C
    r = length(A)
    c = length(A[1])
    C = mat_zeros(r, c)
    for i = 1 to r do
        for j = 1 to c do
            C[i][j] = A[i][j] + B[i][j]
        end for
    end for
    return C
end function

-- Matrix subtraction: A - B
public function mat_sub(sequence A, sequence B)
    integer r, c
    sequence C
    r = length(A)
    c = length(A[1])
    C = mat_zeros(r, c)
    for i = 1 to r do
        for j = 1 to c do
            C[i][j] = A[i][j] - B[i][j]
        end for
    end for
    return C
end function

-- Matrix scalar multiplication: s * A
public function mat_scale(sequence A, atom s)
    integer r, c
    sequence C
    r = length(A)
    c = length(A[1])
    C = mat_zeros(r, c)
    for i = 1 to r do
        for j = 1 to c do
            C[i][j] = A[i][j] * s
        end for
    end for
    return C
end function

-- Matrix multiplication: A * B (dimension: (rA x cA) * (cA x cB) -> (rA x cB))
public function mat_mul(sequence A, sequence B)
    integer rA, cA, cB
    atom sum_val
    sequence C
    rA = length(A)
    if rA = 0 then
        return {}
    end if
    cA = length(A[1])
    cB = length(B[1])
    C = mat_zeros(rA, cB)
    for i = 1 to rA do
        for j = 1 to cB do
            sum_val = 0.0
            for k = 1 to cA do
                sum_val += A[i][k] * B[k][j]
            end for
            C[i][j] = sum_val
        end for
    end for
    return C
end function

-- Matrix-vector multiplication: A * v (dimension: (rA x cA) * (cA) -> (rA))
public function mat_vec_mul(sequence A, sequence v)
    integer rA, cA
    atom sum_val
    sequence result_vec
    rA = length(A)
    if rA = 0 then
        return {}
    end if
    cA = length(A[1])
    result_vec = repeat(0.0, rA)
    for i = 1 to rA do
        sum_val = 0.0
        for j = 1 to cA do
            sum_val += A[i][j] * v[j]
        end for
        result_vec[i] = sum_val
    end for
    return result_vec
end function

-- Matrix trace (sum of diagonal entries)
public function mat_trace(sequence A)
    integer n
    atom tr
    n = length(A)
    tr = 0.0
    for i = 1 to n do
        tr += A[i][i]
    end for
    return tr
end function

-- Determinant of a 2x2 matrix
public function mat_det_2x2(sequence A)
    return A[1][1] * A[2][2] - A[1][2] * A[2][1]
end function

-- Determinant of a 3x3 matrix
public function mat_det_3x3(sequence A)
    return A[1][1] * (A[2][2] * A[3][3] - A[2][3] * A[3][2]) -
           A[1][2] * (A[2][1] * A[3][3] - A[2][3] * A[3][1]) +
           A[1][3] * (A[2][1] * A[3][2] - A[2][2] * A[3][1])
end function

-- LU Decomposition with partial pivoting: P * A = L * U
-- Returns sequence {L, U, P_vec} where P_vec maps permuted row indices
public function mat_lu_decompose(sequence A)
    integer n, max_row
    atom max_val, factor
    sequence L, U, P
    sequence temp_row
    integer temp_idx

    n = length(A)
    L = mat_eye(n)
    U = A
    P = repeat(0, n)
    for i = 1 to n do
        P[i] = i
    end for

    for k = 1 to n - 1 do
        -- Find pivot
        max_row = k
        max_val = abs(U[k][k])
        for i = k + 1 to n do
            if abs(U[i][k]) > max_val then
                max_val = abs(U[i][k])
                max_row = i
            end if
        end for

        -- Row swap in U, L, and P
        if max_row != k then
            temp_row = U[k]
            U[k] = U[max_row]
            U[max_row] = temp_row

            temp_idx = P[k]
            P[k] = P[max_row]
            P[max_row] = temp_idx

            for j = 1 to k - 1 do
                atom tmp_l = L[k][j]
                L[k][j] = L[max_row][j]
                L[max_row][j] = tmp_l
            end for
        end if

        -- Elimination step
        if abs(U[k][k]) > EPSILON then
            for i = k + 1 to n do
                factor = U[i][k] / U[k][k]
                L[i][k] = factor
                for j = k to n do
                    U[i][j] -= factor * U[k][j]
                end for
            end for
        end if
    end for

    return {L, U, P}
end function

-- Solve linear system A * x = b using LU factors and permutation P
public function mat_lu_solve(sequence L, sequence U, sequence P, sequence b)
    integer n
    atom sum_val
    sequence pb, y, x

    n = length(L)
    pb = repeat(0.0, n)
    for i = 1 to n do
        pb[i] = b[P[i]]
    end for

    -- Forward substitution: L * y = pb
    y = repeat(0.0, n)
    for i = 1 to n do
        sum_val = pb[i]
        for j = 1 to i - 1 do
            sum_val -= L[i][j] * y[j]
        end for
        y[i] = sum_val / L[i][i]
    end for

    -- Back substitution: U * x = y
    x = repeat(0.0, n)
    for i = n to 1 by -1 do
        sum_val = y[i]
        for j = i + 1 to n do
            sum_val -= U[i][j] * x[j]
        end for
        if abs(U[i][i]) > EPSILON then
            x[i] = sum_val / U[i][i]
        else
            x[i] = 0.0
        end if
    end for

    return x
end function

-- General square matrix inversion via LU decomposition: A^-1
public function mat_inv(sequence A)
    integer n
    sequence lu_res, L, U, P, A_inv, col_b, col_x

    n = length(A)
    if n = 1 then
        if abs(A[1][1]) > EPSILON then
            return {{1.0 / A[1][1]}}
        else
            return {{0.0}}
        end if
    end if

    lu_res = mat_lu_decompose(A)
    L = lu_res[1]
    U = lu_res[2]
    P = lu_res[3]

    A_inv = mat_zeros(n, n)
    col_b = repeat(0.0, n)

    for j = 1 to n do
        for i = 1 to n do
            if i = j then
                col_b[i] = 1.0
            else
                col_b[i] = 0.0
            end if
        end for
        col_x = mat_lu_solve(L, U, P, col_b)
        for i = 1 to n do
            A_inv[i][j] = col_x[i]
        end for
    end for

    return A_inv
end function

-- Cholesky Factorization for symmetric positive-definite matrix A = L * L^T
-- Returns lower triangular matrix L
public function mat_cholesky(sequence A)
    integer n
    atom sum_val
    sequence L

    n = length(A)
    L = mat_zeros(n, n)

    for i = 1 to n do
        for j = 1 to i do
            sum_val = 0.0
            for k = 1 to j - 1 do
                sum_val += L[i][k] * L[j][k]
            end for

            if i = j then
                atom diag_val = A[i][i] - sum_val
                if diag_val <= 0.0 then
                    -- Regularization for near-semi-definite covariance
                    diag_val = EPSILON
                end if
                L[i][j] = sqrt(diag_val)
            else
                if L[j][j] > EPSILON then
                    L[i][j] = (A[i][j] - sum_val) / L[j][j]
                else
                    L[i][j] = 0.0
                end if
            end if
        end for
    end for

    return L
end function

-- Moore-Penrose Pseudo-Inverse: A^+
-- If rows < cols (underdetermined, e.g. 6x8 thruster allocation matrix):
--   A^+ = A^T * (A * A^T + lambda^2 * I)^-1 (Damped Least Squares)
-- If rows >= cols (overdetermined, e.g. sensor regression):
--   A^+ = (A^T * A + lambda^2 * I)^-1 * A^T
public function mat_pseudo_inverse(sequence A, atom damping)
    integer r, c
    sequence AT, gram, reg_I, inv_gram, result_mat

    r = length(A)
    if r = 0 then
        return {}
    end if
    c = length(A[1])
    AT = mat_transpose(A)

    if r <= c then
        -- Underdetermined or square: A * A^T is (r x r)
        gram = mat_mul(A, AT)
        if damping > 0.0 then
            reg_I = mat_scale(mat_eye(r), damping * damping)
            gram = mat_add(gram, reg_I)
        end if
        inv_gram = mat_inv(gram)
        result_mat = mat_mul(AT, inv_gram)
    else
        -- Overdetermined: A^T * A is (c x c)
        gram = mat_mul(AT, A)
        if damping > 0.0 then
            reg_I = mat_scale(mat_eye(c), damping * damping)
            gram = mat_add(gram, reg_I)
        end if
        inv_gram = mat_inv(gram)
        result_mat = mat_mul(inv_gram, AT)
    end if

    return result_mat
end function
