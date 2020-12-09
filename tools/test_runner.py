#!/usr/bin/env python3
"""
Moonpool Unified Test and Benchmark Orchestrator.
Executes Euphoria core unit tests, C17 fastpath tests, end-to-end integration scenarios,
and performance throughput benchmarks.
"""

import os
import sys
import time
import argparse
import subprocess

REPO_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
RUNNER = os.path.join(REPO_DIR, "tools", "eu_runner.py")
FASTPATH_BIN = os.path.join(REPO_DIR, "build", "test_fastpath")

def run_euphoria_test(test_rel_path):
    test_abs = os.path.join(REPO_DIR, test_rel_path)
    print(f">> Running Euphoria suite: {test_rel_path}")
    res = subprocess.run([RUNNER, test_abs], cwd=REPO_DIR)
    if res.returncode != 0:
        print(f"FAILED: {test_rel_path} exited with code {res.returncode}")
        return False
    return True

def run_fastpath_suite():
    print(">> Running C17 fastpath test suite...")
    if not os.path.isfile(FASTPATH_BIN):
        os.makedirs(os.path.join(REPO_DIR, "build"), exist_ok=True)
        build_cmd = [
            "clang", "-std=c17", "-O3", "-Wall", "-Wextra", "-Werror", "-pedantic",
            "-I.", "-o", FASTPATH_BIN, "tests/test_fastpath.c", "src/fastpath/fastpath.c"
        ]
        res = subprocess.run(build_cmd, cwd=REPO_DIR)
        if res.returncode != 0:
            print("FAILED to compile C17 fastpath test binary")
            return False
    
    res = subprocess.run([FASTPATH_BIN], cwd=REPO_DIR)
    return res.returncode == 0

def run_integration_suite():
    print(">> Running Subsea Autonomous Mission integration simulation...")
    res = subprocess.run([RUNNER, "src/mission/sim_driver.ex"], cwd=REPO_DIR)
    return res.returncode == 0

def run_benchmarks():
    print("\n================================================================================")
    print("              MOONPOOL EMPIRICAL THROUGHPUT & LATENCY BENCHMARKS                ")
    print("================================================================================\n")

    # 1. 6-DOF Hydrodynamic Forward Allocation Benchmark
    print("1. 8-Thruster Weighted Moore-Penrose Pseudo-Inverse Allocation (10,000 solves):")
    bench_code = """
include std/math.e
include src/core/matrix.e
include src/control/allocation.e

sequence B = allocation_matrix_create(0.6, 0.4, 0.5, 0.3)
sequence tau = {150.0, -45.0, 80.0, 10.0, -15.0, 35.0}
sequence health = repeat(1.0, 8)

atom t0 = time()
for i = 1 to 10000 do
    sequence u = allocation_solve(tau, B, health, 0.001)
end for
atom t1 = time()
atom elapsed = t1 - t0
if elapsed < 0.001 then elapsed = 0.001 end if
atom hz = 10000.0 / elapsed
atom latency_us = (elapsed / 10000.0) * 1000000.0
printf(1, "   -> Elapsed: %4.3f sec | Throughput: %8.1f solves/sec | Latency: %4.2f us/solve\\n", {elapsed, hz, latency_us})
"""
    tmp_bench = os.path.join(REPO_DIR, "build", "bench_alloc.ex")
    os.makedirs(os.path.join(REPO_DIR, "build"), exist_ok=True)
    with open(tmp_bench, "w") as f:
        f.write(bench_code)
    subprocess.run([RUNNER, tmp_bench], cwd=REPO_DIR)

    # 2. Snell Law Acoustic Ray Tracing Benchmark
    print("\n2. Stratified Snell Law Acoustic Ray Tracing (1,000 profiles, 100 layers each):")
    bench_code2 = """
include std/math.e
include src/core/constants.e
include src/acoustic/sound_speed.e
include src/acoustic/ray_trace.e

sequence svp = {
    {0.0, 1505.0}, {50.0, 1495.0}, {150.0, 1482.0},
    {300.0, 1480.0}, {600.0, 1482.0}, {1000.0, 1486.0}
}
atom t0 = time()
for i = 1 to 1000 do
    sequence res = ray_trace_profile(svp, 5.0, 500.0, 25.0)
end for
atom t1 = time()
atom elapsed = t1 - t0
if elapsed < 0.001 then elapsed = 0.001 end if
atom hz = 1000.0 / elapsed
atom latency_us = (elapsed / 1000.0) * 1000000.0
printf(1, "   -> Elapsed: %4.3f sec | Throughput: %8.1f profiles/sec | Latency: %4.2f us/trace\\n", {elapsed, hz, latency_us})
"""
    tmp_bench2 = os.path.join(REPO_DIR, "build", "bench_ray.ex")
    with open(tmp_bench2, "w") as f:
        f.write(bench_code2)
    subprocess.run([RUNNER, tmp_bench2], cwd=REPO_DIR)

    # 3. 15-State ES-EKF Kinematic Predict-Update Benchmark
    print("\n3. 15-State Error-State Extended Kalman Filter (5,000 cycles at 100 Hz):")
    bench_code3 = """
include std/math.e
include src/core/constants.e
include src/core/matrix.e
include src/core/quaternion.e
include src/navigation/ekf.e

sequence ekf = ekf_create({0,0,100}, {0,0,0}, quat_identity(), 1.0, 0.1, 0.01)
sequence accel = {0.0, 0.0, -GRAVITY_STANDARD}
sequence gyro  = {0.0, 0.0, 0.0}

atom t0 = time()
for i = 1 to 5000 do
    ekf = ekf_predict(ekf, accel, gyro, 0.01)
    if remainder(i, 10) = 0 then
        ekf = ekf_update_dvl(ekf, {1.2, 0.0, 0.0}, 0.02)
    end if
end for
atom t1 = time()
atom elapsed = t1 - t0
if elapsed < 0.001 then elapsed = 0.001 end if
atom hz = 5000.0 / elapsed
atom latency_us = (elapsed / 5000.0) * 1000000.0
printf(1, "   -> Elapsed: %4.3f sec | Throughput: %8.1f filter cycles/sec | Latency: %4.2f us/cycle\\n", {elapsed, hz, latency_us})
"""
    tmp_bench3 = os.path.join(REPO_DIR, "build", "bench_ekf.ex")
    with open(tmp_bench3, "w") as f:
        f.write(bench_code3)
    subprocess.run([RUNNER, tmp_bench3], cwd=REPO_DIR)

    print("\n================================================================================\n")
    return True

def main():
    parser = argparse.ArgumentParser(description="Moonpool Test and Benchmark Suite")
    parser.add_argument("--unit", action="store_true", help="Run Euphoria unit test suites")
    parser.add_argument("--fastpath", action="store_true", help="Run C17 fastpath test suite")
    parser.add_argument("--integration", action="store_true", help="Run mission integration simulation")
    parser.add_argument("--bench", action="store_true", help="Run empirical performance benchmarks")
    parser.add_argument("--all", action="store_true", help="Run complete test and verification matrix")

    args = parser.parse_args()
    if not (args.unit or args.fastpath or args.integration or args.bench or args.all):
        args.all = True

    success = True
    unit_suites = [
        "tests/t_core.ex",
        "tests/t_acoustic.ex",
        "tests/t_navigation.ex",
        "tests/t_bathymetry.ex",
        "tests/t_inspection.ex",
        "tests/t_control.ex",
        "tests/t_telemetry.ex",
        "tests/t_pipeline.ex",
        "tests/t_ocean.ex"
    ]

    if args.unit or args.all:
        for s in unit_suites:
            if not run_euphoria_test(s):
                success = False

    if args.fastpath or args.all:
        if not run_fastpath_suite():
            success = False

    if args.integration or args.all:
        if not run_integration_suite():
            success = False

    if args.bench:
        if not run_benchmarks():
            success = False

    if success:
        print("\nAll selected test suites passed with 100% success.")
        sys.exit(0)
    else:
        print("\nOne or more test suites failed.")
        sys.exit(1)

if __name__ == "__main__":
    main()
