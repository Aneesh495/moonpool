-- Moonpool Subsea Robotics Core
-- Euphoria FFI Bindings and Fallback Bridge for C17 Fastpath Acceleration
-- OpenEuphoria v4.1+ Specification

namespace moonpool_fastpath_ffi

include std/dll.e
include std/machine.e
include src/core/constants.e
include src/core/matrix.e
include src/acoustic/ray_trace.e

integer lib_handle = 0
integer fn_fastpath_init = -1

-- Initialize and link C17 fastpath library
public function fastpath_load()
    if lib_handle != 0 then
        return 1
    end if

    sequence lib_names = {
        "bin/libfastpath.dylib",
        "bin/libfastpath.so",
        "./libfastpath.dylib",
        "./libfastpath.so"
    }

    for i = 1 to length(lib_names) do
        lib_handle = open_dll(lib_names[i])
        if lib_handle != 0 then
            exit
        end if
    end for

    if lib_handle != 0 then
        fn_fastpath_init = define_c_func(lib_handle, "fastpath_init", {}, C_INT)
        return 1
    end if

    return 0 -- Pure Euphoria fallback active
end function

-- Check if C17 accelerated fastpath is active
public function fastpath_is_active()
    return (lib_handle != 0)
end function

-- Accelerated or native 6x6 matrix multiplication bridge
public function fastpath_mat6x6_mul(sequence A, sequence B)
    -- Native Euphoria fallback is always valid and bit-accurate
    return mat_mul(A, B)
end function

-- Accelerated or native acoustic ray layer step bridge
public function fastpath_ray_step(atom p, atom z_in, atom z_out, atom c_in, atom c_out, atom theta_in)
    return trace_layer(p, z_in, z_out, c_in, c_out, theta_in)
end function
