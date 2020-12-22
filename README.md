# Moonpool

```
 __  __  ___   ___  _   _ ____   ___   ___  _     
|  \/  |/ _ \ / _ \| \ | |  _ \ / _ \ / _ \| |    
| |\/| | | | | | | |  \| | |_) | | | | | | | |    
| |  | | |_| | |_| | |\  |  __/| |_| | |_| | |___ 
|_|  |_|\___/ \___/|_| \_|_|    \___/ \___/|_____|
```

> **Autonomous Subsea Pipeline & Intervention Executive**  
> *Engineered in OpenEuphoria with ISO C17 Fastpath Acceleration*

---

[![Language: OpenEuphoria](https://img.shields.io/badge/Language-OpenEuphoria%204.1.0-blue.svg)](https://openeuphoria.org/)
[![Acceleration: C17 Fastpath](https://img.shields.io/badge/Acceleration-ISO%20C17%20SIMD-orange.svg)](src/fastpath/)
[![Tests: 173 Passing](https://img.shields.io/badge/Tests-173%2F173%20Passing-brightgreen.svg)](docs/VERIFICATION_REPORT.md)
[![Acoustics: Snell Eigenray](https://img.shields.io/badge/Acoustics-Ray%20Tracing%20%26%20USBL-purple.svg)](docs/ACOUSTICS_MANUAL.md)
[![Standards: NACE & ASME B31G](https://img.shields.io/badge/Standards-NACE%20%7C%20B31G%20%7C%20DNV--F105-red.svg)](docs/INSPECTION_STANDARDS.md)
[![Typography: Zero Em-Dash](https://img.shields.io/badge/Typography-Zero%20Em--Dash-success.svg)](tools/lint_no_em_dash.py)
[![License: ISC](https://img.shields.io/badge/License-ISC-lightgrey.svg)](LICENSE)

---

## Overview

**Moonpool** is an autonomous mission executive, acoustic positioning solver, and real-time control system designed for subsea intervention autonomous underwater vehicles (AUVs) and work-class remotely operated vehicles (ROVs).

Operating in deepwater offshore environments where satellite GNSS signals attenuate within millimeters of the ocean surface, Moonpool provides full-stack autonomy for tracking, mapping, and inspecting seafloor trunklines, flowlines, and umbilicals.

Moonpool is built with **OpenEuphoria** as its core programming language, taking full advantage of Euphoria's dynamic sequence architecture, clean procedural syntax, and memory isolation. Performance-critical inner loops (6x6 matrix SIMD multiplication, Snell's law layer stepping, and multi-threaded lockless telemetry ring buffers) are accelerated via a native **ISO C17 fastpath library**.

---

## Key Capabilities

```
+-----------------------------------------------------------------------------+
|                            MOONPOOL ARCHITECTURE                            |
+-----------------------------------------------------------------------------+
|                                                                             |
|  [Sensory Telemetry]                                                        |
|     * Teledyne RD Instruments PD0 DVL (4-Beam Janus Doppler)               |
|     * NMEA-0183 ($GPGGA surface GPS, $PASHR inertial heading/attitude)      |
|     * UNESCO PSS-78 CTD (Conductivity, Temperature, Salinity)               |
|     * Paroscientific Digiquartz Depth Transducer (Saunders-Fofonoff UNESCO) |
|                                                                             |
|  [Underwater Acoustics & Positioning]                                       |
|     * Numerical Snell's Law Ray Tracer (Mackenzie & Del Grosso Sound Speed) |
|     * Multi-Arrival Eigenray Solver (Surface & Seabed Multipath)             |
|     * USBL Phase Interferometry (Direction-of-Arrival & Attitude Lever Arm) |
|     * Long Baseline (LBL) Gauss-Newton Multilateration & DOP Dilution       |
|     * Phased-Array Delay-and-Sum Acoustic Beamforming                       |
|                                                                             |
|  [State Estimation & Mapping]                                               |
|     * 15-State Error-State Kalman Filter (ES-EKF with Joseph Updates)       |
|     * Multibeam Sonar 2.5D DEM Bathymetry Rasterization                     |
|     * Horn's Topographic Slope & Rugosity Estimation                        |
|     * Benthic Traversability & Obstacle Exclusion Costmaps                  |
|                                                                             |
|  [Pipeline Integrity & NDT Payloads]                                        |
|     * Dual Ag/AgCl Cathodic Protection Potential Survey (NACE SP0169)       |
|     * Electric Field Gradient Coating Holiday Detection                     |
|     * Sacrificial Anode Depletion & Resistance (Dwight & McCoy Models)      |
|     * Ultrasonic Pulse-Echo Wall Thickness & Pitting Profiling              |
|     * ASME B31G & Modified RSTRENG Remaining Burst Pressure Rating         |
|     * DNV-RP-F105 Free Span Vortex-Induced Vibration (VIV) Lock-In          |
|                                                                             |
|  [Guidance, Navigation, & Control (GNC)]                                    |
|     * SNAME 6-DOF Hydrodynamic Equations of Motion (Added Mass, Drag)       |
|     * 8-Thruster Vectored Geometry & Moore-Penrose Pseudo-Inverse           |
|     * Thruster Deadband Compensation & Power Saturation Limiting            |
|     * 6-DOF Decoupled Proportional-Integral-Derivative (PID) Regulators     |
|     * Line-of-Sight (LOS) 3D Trajectory Steering Guidance                   |
|                                                                             |
+-----------------------------------------------------------------------------+
```

---

## Directory Structure

```
moonpool/
├── bin/                          # Compiled shared libraries (libfastpath.dylib/.so)
├── build/                        # Build artifacts and intermediate object files
├── docs/                         # Detailed engineering specifications
│   ├── ACOUSTICS_MANUAL.md       # Underwater acoustic physics & ray propagation
│   ├── ARCHITECTURE.md           # System subsystem topology and mathematical formulation
│   ├── CONTROL_SYSTEM.md         # 6-DOF hydrodynamics and thruster allocation theory
│   ├── INSPECTION_STANDARDS.md   # NACE SP0169, ASME B31G, and DNV-RP-F105 specifications
│   └── VERIFICATION_REPORT.md    # Empirical test matrix and microbenchmark throughput
├── src/
│   ├── acoustic/                 # Snell ray tracer, USBL, LBL, beamforming
│   │   ├── beamforming.e         # Delay-and-sum phased array acoustic beamformer
│   │   ├── lbl.e                 # Long baseline multilateration & DOP computation
│   │   ├── multipath.e           # Surface and seabed reflection ray bounce modeling
│   │   ├── ray_trace.e           # Numerical circular arc sound speed integration
│   │   ├── sound_speed.e         # Mackenzie, Del Grosso, Medwin sound velocity equations
│   │   └── usbl.e                # Ultra-short baseline phase interferometry solver
│   ├── bathymetry/               # Seafloor mapping and terrain reasoning
│   │   ├── costmap.e             # Benthic traversability costmap synthesis
│   │   ├── dem_grid.e            # 2.5D Digital Elevation Model raster grid
│   │   ├── multibeam.e           # Multibeam echosounder swath fan footprint projection
│   │   └── slope.e               # Horn's 3x3 topographic slope and rugosity index
│   ├── control/                  # Hydrodynamics and propulsion allocation
│   │   ├── allocation.e          # 8-thruster geometry matrix & pseudo-inverse solver
│   │   ├── guidance.e            # 3D Line-of-Sight (LOS) waypoint steering controller
│   │   ├── hydrodynamics.e       # SNAME 6-DOF marine vehicle equations of motion
│   │   ├── pid_controller.e      # Decoupled 6-axis PID regulator with anti-windup
│   │   └── thruster_model.e      # Ducted propeller bollard pull and deadband dynamics
│   ├── core/                     # Foundational mathematics, geodesy, and algebra
│   │   ├── constants.e           # Physical constants, fluid properties, and unit scales
│   │   ├── coords.e              # WGS-84, ECEF, NED coordinate frames & Haversine
│   │   ├── math_util.e           # Clamping, interpolation, angle wrapping, statistics
│   │   ├── matrix.e              # LU, Cholesky, SVD, and Moore-Penrose pseudo-inverse
│   │   └── quaternion.e          # Unit quaternions, DCM direction cosines, SLERP
│   ├── fastpath/                 # ISO C17 SIMD and hardware acceleration
│   │   ├── fastpath.c            # SIMD matrix multiplication, ray stepping, ring buffer
│   │   ├── fastpath.h            # C17 public API interface definitions
│   │   └── fastpath_ffi.e        # OpenEuphoria Foreign Function Interface bindings
│   ├── inspection/               # Subsea NDT and structural pipeline integrity
│   │   ├── anode.e               # Dwight/McCoy anode resistance and Faraday consumption
│   │   ├── b31g.e                # ASME B31G and Modified RSTRENG burst pressure
│   │   ├── cathodic.e            # Dual Ag/AgCl CP potential & holiday electric fields
│   │   └── ultrasonic.e          # Pulse-echo A-scan acoustic wall thickness gauging
│   ├── mission/                  # Autonomous state machine and simulation scenario
│   │   ├── executive.e           # Finite state supervisor and contingency manager
│   │   ├── scenario.e            # North Sea 1.5 km pipeline inspection profile
│   │   └── sim_driver.ex         # Interactive ANSI terminal mission control dashboard
│   ├── navigation/               # Marine navigation and state estimation
│   │   ├── dead_reckon.e         # Strapdown kinematics and velocity integration
│   │   ├── depth_sensor.e        # Saunders-Fofonoff UNESCO pressure-to-depth formulation
│   │   ├── dvl.e                 # 4-beam Janus Doppler Velocity Log solving
│   │   └── ekf.e                 # 15-state Error-State Kalman Filter (ES-EKF)
│   ├── ocean/                    # Oceanographic dynamics and boundary layer models
│   │   ├── currents.e            # Ekman spiral current profiles and shear layers
│   │   ├── tides.e               # Astronomical harmonic tidal constituents (M2, S2, etc.)
│   │   └── waves.e               # Airy linear wave dispersion and subsea orbital motion
│   ├── pipeline/                 # Pipeline mechanics, fatigue, and free spans
│   │   ├── coating.e             # Concrete weight coating buoyancy and shear friction
│   │   ├── fatigue.e             # ASTM rainflow cycle counting and Palmgren-Miner damage
│   │   ├── pipe_geometry.e       # Line pipe dimensions, moment of inertia, submerged mass
│   │   └── spans.e               # DNV-RP-F105 unsupported span eigenfrequency and VIV
│   └── telemetry/                # Sensor protocol parsing and serialization
│       ├── ctd.e                 # UNESCO PSS-78 salinity and conductivity equations
│       ├── nmea.e                # NMEA-0183 checksum verification, $GPGGA, $PASHR
│       └── pd0_dvl.e             # Teledyne RD Instruments binary PD0 packet decoding
├── tests/                        # Comprehensive automated verification suites
│   ├── t_acoustic.ex             # Acoustic ray tracing, USBL, and LBL verification
│   ├── t_bathymetry.ex           # Multibeam swath projection and DEM raster tests
│   ├── t_control.ex              # 6-DOF hydrodynamics and thruster allocation tests
│   ├── t_core.ex                 # Matrix, quaternion, and geodesy tests
│   ├── t_inspection.ex           # CP potential, anode life, and B31G burst tests
│   ├── t_navigation.ex           # DVL Doppler solving, depth, and ES-EKF tests
│   ├── t_ocean.ex                # Tidal constituents, wave dispersion, beamforming
│   ├── t_pipeline.ex             # Pipe properties, fatigue rainflow, and free spans
│   ├── t_telemetry.ex            # NMEA, PD0, and CTD sensor decoding tests
│   └── test_fastpath.c           # C17 fastpath SIMD and ring buffer unit tests
├── tools/                        # Developer utilities and build scripts
│   ├── bootstrap.sh              # OpenEuphoria automated toolchain installer
│   ├── eu_runner.py              # Portable OpenEuphoria execution harness
│   ├── lint_no_em_dash.py        # Zero em-dash typography enforcement linter
│   ├── moonpoolctl               # Unified developer CLI for build, test, and sim
│   └── test_runner.py            # Automated multi-suite test harness and reporter
├── Makefile                      # Standardized build and lifecycle orchestrator
└── README.md                     # Master project documentation
```

---

## Quickstart

### Prerequisites

- A Unix-like operating system (macOS or Linux).
- An ISO C17 compliant compiler (`clang` or `gcc`).
- Python 3.8+ (for developer CLI tooling).

### 1. Bootstrap OpenEuphoria

Moonpool includes an automated bootstrapping script that downloads and configures the OpenEuphoria 4.1.0 toolchain locally into `.euphoria/` without requiring root permissions:

```bash
make bootstrap
```

### 2. Build Native Acceleration Library

Compile the ISO C17 fastpath shared library (`bin/libfastpath.dylib` on macOS or `bin/libfastpath.so` on Linux):

```bash
make build
```

### 3. Run Complete Verification Matrix

Execute all 10 verification test suites (173 total assertions across Euphoria and C17):

```bash
make test
```

Expected output:
```
[TEST SUMMARY] All 10 test suites passed successfully! (173/173 assertions)
```

### 4. Execute Performance Benchmarks

Run microbenchmarks measuring thruster allocation solve rates, Snell acoustic ray tracing throughput, and 15-state ES-EKF filter step latency:

```bash
make bench
```

### 5. Launch North Sea Autonomous Mission Simulation

Run the end-to-end autonomous pipeline inspection scenario with an interactive ANSI terminal operator console:

```bash
make sim
```

### 6. Verify Typography and Lint Standards

Verify that no em-dashes exist across any source, documentation, or script file:

```bash
make lint
```

---

## Core Technical Deep Dives

### 1. Acoustic Ray Tracing and Eigenrays

Underwater sound speed varies continuously with depth due to changes in water temperature, salinity, and hydrostatic pressure:

$$c(T, S, z) = 1448.96 + 4.591 T - 5.304 \times 10^{-2} T^2 + 2.374 \times 10^{-4} T^3 + 1.340 (S - 35) + 1.630 \times 10^{-2} z$$

Because sound speed is non-uniform, acoustic waves bend according to Snell's law:

$$\frac{\cos \theta(z)}{c(z)} = p = \text{constant}$$

Moonpool discretizes the ocean water column into horizontal iso-gradient sound speed layers. Within each layer with gradient $g_i = \frac{c_{i+1} - c_i}{z_{i+1} - z_i}$, the acoustic ray path is an exact circular arc with radius of curvature:

$$R_i = -\frac{1}{p \cdot g_i}$$

The horizontal range step $\Delta x_i$ and travel time step $\Delta \tau_i$ across layer $i$ are computed analytically:

$$\Delta x_i = \frac{1}{p \cdot g_i} \left( \sin \theta_i - \sin \theta_{i+1} \right)$$

$$\Delta \tau_i = \frac{1}{|g_i|} \ln\left( \frac{c_{i+1}}{c_i} \cdot \frac{1 + \sin \theta_i}{1 + \sin \theta_{i+1}} \right)$$

Moonpool employs a shooting-method eigenray solver with bisection refinement to identify direct, surface-reflected, and bottom-reflected ray paths connecting subsea transponders to vehicle-mounted acoustic arrays.

### 2. 15-State Error-State Kalman Filter (ES-EKF)

Moonpool tracks vehicle pose and velocity through a 15-state continuous-discrete Error-State Kalman Filter. The error state vector $\delta \mathbf{x} \in \mathbb{R}^{15}$ comprises:

$$\delta \mathbf{x} = \begin{bmatrix} \delta \boldsymbol{\theta}^T & \delta \mathbf{v}^T & \delta \mathbf{p}^T & \delta \mathbf{b}_a^T & \delta \mathbf{b}_g^T \end{bmatrix}^T$$

- $\delta \boldsymbol{\theta}$: Attitude error in tangent space ($\text{rad}$)
- $\delta \mathbf{v}$: Linear velocity error in NED frame ($\text{m/s}$)
- $\delta \mathbf{p}$: Position error in NED frame ($\text{m}$)
- $\delta \mathbf{b}_a$: Accelerometer bias error ($\text{m/s}^2$)
- $\delta \mathbf{b}_g$: Gyroscope bias error ($\text{rad/s}$)

Sensor fusion updates occur asynchronously:
- **DVL 4-Beam Doppler**: Updates body-frame linear velocity $\mathbf{v}_b$ at 5 Hz to 10 Hz.
- **USBL / LBL Acoustics**: Updates NED vehicle position $\mathbf{p}$ at 0.5 Hz to 2 Hz.
- **UNESCO Depth Transducer**: Updates down-position component $p_D$ at 10 Hz with millimeter accuracy.

All covariance updates utilize the numerically stabilized **Joseph form**:

$$\mathbf{P}^+ = (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P}^- (\mathbf{I} - \mathbf{K} \mathbf{H})^T + \mathbf{K} \mathbf{R} \mathbf{K}^T$$

This ensures positive semi-definiteness even under ill-conditioned measurement geometries.

### 3. Vectored 8-Thruster Allocation

The vehicle propulsion system consists of 8 ducted brush-less electric thrusters configured in a vectored arrangement (4 horizontal vectored thrusters in an X-configuration and 4 vertical thrusters angled for pitch and roll control).

The mapping from thruster force vector $\mathbf{u} \in \mathbb{R}^8$ to generalized vehicle forces and torques $\boldsymbol{\tau} \in \mathbb{R}^6$ is governed by the thruster configuration matrix $\mathbf{B} \in \mathbb{R}^{6 \times 8}$:

$$\boldsymbol{\tau} = \mathbf{B} \mathbf{u}$$

Moonpool computes the minimum-energy thrust allocation using the weighted Moore-Penrose pseudo-inverse:

$$\mathbf{u} = \mathbf{W}^{-1} \mathbf{B}^T \left( \mathbf{B} \mathbf{W}^{-1} \mathbf{B}^T \right)^{-1} \boldsymbol{\tau}_{\text{cmd}}$$

where $\mathbf{W}$ is a positive-definite diagonal weighting matrix penalizing thruster wear. The solver dynamically incorporates:
- Motor deadband compensation ($\pm 4.0\text{ N}$)
- Slew rate limiting to protect motor ESC drives ($40.0\text{ N/s}$)
- Quadratic thrust-to-RPM propeller mapping ($T = C_T \rho D^4 n |n|$)

---

## Technical Documentation

Detailed mathematical derivations, engineering manuals, and architectural diagrams are located in the `docs/` directory:

- [System Architecture Specification](docs/ARCHITECTURE.md): Complete subsystem topology, dataflow graphs, and thread execution models.
- [Acoustics Engineering Manual](docs/ACOUSTICS_MANUAL.md): Sound speed profile equations, eigenray solvers, USBL phase interferometry, and array beamforming.
- [6-DOF Control System Manual](docs/CONTROL_SYSTEM.md): SNAME hydrodynamics, added mass tensors, 8-thruster allocation, and anti-windup PID tuning.
- [Inspection & Pipeline Standards](docs/INSPECTION_STANDARDS.md): NACE SP0169 CP survey, ultrasonic NDT, ASME B31G burst pressure, and DNV-RP-F105 free spans.
- [Verification & Benchmarking Report](docs/VERIFICATION_REPORT.md): Empirical test matrix, throughput latency tables, and simulation audit trail.

---

## License

Moonpool is released under the open source [ISC License](LICENSE).
