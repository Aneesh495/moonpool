# Moonpool Acoustics Engineering Manual

## 1. Fundamentals of Underwater Acoustic Propagation

Electromagnetic signals attenuate rapidly in conductive seawater, restricting high-bandwidth radio communications to millimeters. Consequently, underwater positioning, telemetry, and bathymetric mapping rely almost exclusively on acoustic pressure waves.

Acoustic propagation speed $c$ in seawater varies between $1440\text{ m/s}$ and $1550\text{ m/s}$, governed by:
- **Temperature ($T$)**: $+4.5\text{ m/s}$ per degree Celsius
- **Salinity ($S$)**: $+1.3\text{ m/s}$ per PSU
- **Static Depth ($D$)**: $+0.016\text{ m/s}$ per meter of hydrostatic depth

---

## 2. Water Column Stratification and Ray Bending

The water column is divided into three characteristic physical regimes:
1. **Surface Mixed Layer**: Wind-stirred isothermal zone with positive sound speed gradient due to pressure increase.
2. **Main Thermocline**: Sharp negative temperature gradient where cooling water dominates over hydrostatic pressure, resulting in a steep negative sound speed gradient ($\frac{dc}{dz} < 0$).
3. **Deep Isothermal Layer**: Near-constant water temperature ($2^\circ\text{C}$ to $4^\circ\text{C}$), where increasing hydrostatic pressure creates a monotonic positive sound speed gradient ($\frac{dc}{dz} > 0$).

The transition boundary between the negative gradient thermocline and the positive gradient deep water forms the **SOFAR channel (Sound Fixing and Ranging)** axis, where acoustic rays bend continuously toward the minimum sound speed axis, creating an acoustic waveguide capable of trans-oceanic propagation.

```
Depth (m) | Sound Speed (m/s)  Profile Characteristics
----------+-------------------------------------------------------------
0         | 1505 m/s           Warm surface mixed layer
50        | 1495 m/s           Upper thermocline boundary
200       | 1482 m/s           Steep negative thermal gradient
500       | 1480 m/s  <-----   SOFAR Channel Minimum Axis (Waveguide Focus)
1000      | 1484 m/s           Hydrostatic pressure gradient dominates
2000      | 1500 m/s           Deep abyssal ocean
```

---

## 3. Ultra-Short Baseline (USBL) Phase Interferometry

USBL systems measure the relative position of a subsea transponder beacon from a surface vessel using an acoustic transducer array with baseline spacing $d$ on the order of centimeters ($d \approx 5\text{ cm}$ to $15\text{ cm}$).

### Direction of Arrival (DoA) Principle
An incoming plane wave arriving at incidence angle $\theta$ relative to the array normal reaches adjacent hydrophone elements with a path length difference $\Delta L = d \cdot \sin\theta$.

The resulting acoustic phase difference $\Delta\phi$ is:

$$\Delta\phi = \frac{2\pi}{\lambda} d \cdot \sin\theta = \frac{2\pi f}{c} d \cdot \sin\theta$$

Solving for the spatial arrival angle:

$$\sin\theta = \frac{\Delta\phi \cdot c}{2\pi f \cdot d}$$

### Spatial Attitude Transformation
The calculated unit Direction of Arrival vector in the transceiver body frame $u_b = [u_x, u_y, u_z]^T$ is rotated into the local North-East-Down (NED) navigation frame using the vessel attitude quaternion $q$ and Direction Cosine Matrix $R_b^n(q)$:

$$u_n = R_b^n(q) \cdot u_b$$

Accounting for the calibrated lever-arm offset $l_b$ from vessel reference point to transducer head, the 3D position of the subsea beacon in local NED coordinates is:

$$p_{beacon, n} = p_{vessel, n} + R_b^n(q) \cdot l_b + R_{slant} \cdot u_n$$

---

## 4. Long Baseline (LBL) Multilateration

In deepwater installations where subsea vehicles operate kilometers below surface vessels, USBL angular errors scale linearly with slant range ($\sigma_p \approx R_{slant} \cdot \sigma_\theta$). Long Baseline (LBL) systems overcome this by establishing a seafloor array of calibrated acoustic transponders.

### Gauss-Newton Non-Linear Least Squares
Given $m \ge 3$ transponders at known locations $p_{transponder, i}$ and measured one-way or two-way acoustic ranges $r_i$:

$$f_i(p) = \|p - p_{transponder, i}\| - r_i = 0$$

The linearized system is solved iteratively via Gauss-Newton:

$$\Delta p = -\left( J^T J + \lambda I \right)^{-1} J^T f(p)$$

where the Jacobian row $J_{i, :}$ is the unit pointing vector from transponder $i$ to vehicle estimate $p$:

$$J_{i, :} = \frac{(p - p_{transponder, i})^T}{\|p - p_{transponder, i}\|}$$

### Dilution of Precision (DOP)
Position covariance is given by $Q = (J^T J)^{-1}$. Geometric Dilution of Precision metrics quantify spatial geometry quality:

$$HDOP = \sqrt{Q_{11} + Q_{22}}$$
$$VDOP = \sqrt{Q_{33}}$$
$$PDOP = \sqrt{Q_{11} + Q_{22} + Q_{33}}$$
