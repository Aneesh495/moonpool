-- Moonpool Unit Test Suite: Core Mathematics, Matrix, Quaternions, and Coordinates
-- OpenEuphoria v4.1+

include std/unittest.e
include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e
include src/core/quaternion.e
include src/core/coords.e

set_test_verbosity(TEST_SHOW_ALL)

-- 1. Math Util Tests
test_equal("Clamp within bounds", 5.0, clamp(5.0, 0.0, 10.0))
test_equal("Clamp lower bound", 0.0, clamp(-2.0, 0.0, 10.0))
test_equal("Clamp upper bound", 10.0, clamp(15.0, 0.0, 10.0))
test_equal("Lerp midpoint", 5.0, lerp(0.0, 10.0, 0.5))
test_equal("Sign positive", 1.0, sign(42.5))
test_equal("Sign negative", -1.0, sign(-12.3))
test_equal("Sign zero", 0.0, sign(0.0))

test_true("Wrap PI upper", approx_equal(0.0, wrap_pi(TWO_PI), 1e-7))
test_true("Mean calculation", approx_equal(3.0, mean({1.0, 2.0, 3.0, 4.0, 5.0}), 1e-7))
test_true("Variance calculation", approx_equal(2.5, variance({1.0, 2.0, 3.0, 4.0, 5.0}), 1e-7))

-- 2. Matrix Tests
sequence A = {{1.0, 2.0}, {3.0, 4.0}}
sequence B = {{2.0, 0.0}, {1.0, 2.0}}
sequence C_add = mat_add(A, B)
test_equal("Matrix addition (1,1)", 3.0, C_add[1][1])
test_equal("Matrix addition (2,2)", 6.0, C_add[2][2])

sequence C_mul = mat_mul(A, B)
test_equal("Matrix mul (1,1)", 4.0, C_mul[1][1])
test_equal("Matrix mul (1,2)", 4.0, C_mul[1][2])
test_equal("Matrix mul (2,1)", 10.0, C_mul[2][1])
test_equal("Matrix mul (2,2)", 8.0, C_mul[2][2])

test_true("Matrix det 2x2", approx_equal(-2.0, mat_det_2x2(A), 1e-7))

sequence A_inv = mat_inv(A)
sequence prod_inv = mat_mul(A, A_inv)
test_true("Matrix inverse identity check (1,1)", approx_equal(1.0, prod_inv[1][1], 1e-5))
test_true("Matrix inverse identity check (1,2)", approx_equal(0.0, prod_inv[1][2], 1e-5))
test_true("Matrix inverse identity check (2,1)", approx_equal(0.0, prod_inv[2][1], 1e-5))
test_true("Matrix inverse identity check (2,2)", approx_equal(1.0, prod_inv[2][2], 1e-5))

-- Vector dot & cross product
sequence u = {1.0, 0.0, 0.0}
sequence v = {0.0, 1.0, 0.0}
sequence w = vec_cross(u, v)
test_equal("Cross product X", 0.0, w[1])
test_equal("Cross product Y", 0.0, w[2])
test_equal("Cross product Z", 1.0, w[3])

-- 3. Quaternion Tests
sequence q_id = quat_identity()
test_equal("Quaternion identity norm", 1.0, quat_norm(q_id))

-- 90 deg Yaw rotation around Z axis
sequence q_yaw90 = quat_from_euler(0.0, 0.0, HALF_PI)
sequence euler = quat_to_euler(q_yaw90)
test_true("Euler Yaw extraction", approx_equal(HALF_PI, euler[3], 1e-5))

sequence v_body = {1.0, 0.0, 0.0}
sequence v_nav = quat_rotate_vec(q_yaw90, v_body)
-- Rotating {1, 0, 0} by +90 deg yaw yields {0, 1, 0}
test_true("Quaternion rotate vector X", approx_equal(0.0, v_nav[1], 1e-5))
test_true("Quaternion rotate vector Y", approx_equal(1.0, v_nav[2], 1e-5))
test_true("Quaternion rotate vector Z", approx_equal(0.0, v_nav[3], 1e-5))

-- 4. Coordinates Tests
sequence ecef = geodetic_to_ecef(56.5, 3.2, 0.0)
sequence geo_back = ecef_to_geodetic(ecef[1], ecef[2], ecef[3])
test_true("Geodetic roundtrip Lat", approx_equal(56.5, geo_back[1], 1e-4))
test_true("Geodetic roundtrip Lon", approx_equal(3.2, geo_back[2], 1e-4))
test_true("Geodetic roundtrip Alt", approx_equal(0.0, geo_back[3], 1e-2))

test_report()
