# Moonpool Control System and Hydrodynamics Manual

## 1. Subsea Vehicle Equations of Motion (SNAME 1950)

Subsea vehicle 6-DOF dynamics are governed by Newton-Euler mechanics and SNAME (1950) marine notation:

$$M \dot{\nu} + C(\nu)\nu + D(\nu)\nu + g(\eta) = \tau_{thruster} + \tau_{environment}$$

where:
- $\nu = [u, v, w, p, q, r]^T$: Body velocities (Surge, Sway, Heave, Roll, Pitch, Yaw)
- $\eta = [x, y, z, \phi, \theta, \psi]^T$: NED position and Euler orientation
- $M = M_{RB} + M_A$: Total inertia matrix combining rigid body mass and hydrodynamic added mass
- $C(\nu) = C_{RB}(\nu) + C_A(\nu)$: Coriolis and centripetal matrix
- $D(\nu)$: Linear and quadratic hydrodynamic damping matrix
- $g(\eta)$: Restoring forces and moments from gravity and buoyancy
- $\tau_{thruster}$: Control wrench produced by thruster propulsion array

---

## 2. Hydrostatic Metacentric Stability

Subsea vehicles achieve passive static stability through vertical separation between the Center of Buoyancy (CB, $[x_B, y_B, z_B]^T$) and the Center of Gravity (CG, $[x_G, y_G, z_G]^T$).

By engineering the vehicle such that the Center of Buoyancy is located strictly above the Center of Gravity ($z_B < z_G$ in downward-positive NED coordinates), any angular perturbation in pitch ($\theta$) or roll ($\phi$) generates a strong righting moment:

$$K_{restoring} = (y_G W - y_B B) \cos\theta \cos\phi + (z_G W - z_B B) \cos\theta \sin\phi$$
$$M_{restoring} = -(z_G W - z_B B) \sin\theta - (x_G W - x_B B) \cos\theta \cos\phi$$

When $W \approx B$ (neutral buoyancy) and $x_G = x_B = 0, y_G = y_B = 0$:

$$K_{restoring} = (z_G - z_B) W \cos\theta \sin\phi \approx \overline{BG} \cdot W \sin\phi$$
$$M_{restoring} = -(z_G - z_B) W \sin\theta \approx -\overline{BG} \cdot W \sin\theta$$

This righting moment acts as a natural mechanical pendulum, ensuring that pitch and roll remain bounded even during severe ocean current disturbances.

---

## 3. Vectored 8-Thruster Geometry

The Moonpool vehicle incorporates eight brushless DC electric ducted thrusters arranged to decouple horizontal planar motion from vertical pitch and roll control:

- **Thrusters 1 to 4**: Horizontal vectored thrusters mounted at 45 degree angles at the four corners of the chassis.
- **Thrusters 5 to 8**: Vertical thrusters positioned at the four corners to provide high-thrust heave, roll, and pitch control.

```
       [Forward +X]
        T2 \      / T1
            \----/
            |    |
   T6 [V]   |    |   T5 [V]
            |    |
   T8 [V]   |    |   T7 [V]
            /----\
        T4 /      \ T3
         [Aft -X]
```

### Configuration Matrix B
The geometry matrix $B \in \mathbb{R}^{6 \times 8}$ maps individual thruster forces $u \in \mathbb{R}^8$ to the generalized vehicle wrench $\tau \in \mathbb{R}^6$:

$$B = \begin{bmatrix}
c_{45} & c_{45} & -c_{45} & -c_{45} & 0 & 0 & 0 & 0 \\
s_{45} & -s_{45} & s_{45} & -s_{45} & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 1 & 1 & 1 \\
0 & 0 & 0 & 0 & l_{yv} & -l_{yv} & l_{yv} & -l_{yv} \\
0 & 0 & 0 & 0 & -l_{xv} & -l_{xv} & l_{xv} & l_{xv} \\
n_1 & n_2 & n_3 & n_4 & 0 & 0 & 0 & 0
\end{bmatrix}$$

---

## 4. Weighted Pseudo-Inverse Allocation and Fault Tolerance

When all thrusters are healthy, the allocation minimizes the Euclidean 2-norm of control forces $\sum_{i=1}^8 u_i^2$, minimizing total electrical power consumption:

$$u = B^T (B B^T)^{-1} \tau_{cmd}$$

### Active Thruster Health Weighting
In subsea operations, thrusters can suffer propeller fouling, winding insulation breakdown, or ESC failure. Moonpool implements a diagonal weighting matrix $W \in \mathbb{R}^{8 \times 8}$ where $W_{ii} \in [0.0, 1.0]$ represents the operational health factor of thruster $i$.

The weighted pseudo-inverse formulation:

$$u = W^{-1} B^T \left( B W^{-1} B^T + \lambda^2 I_6 \right)^{-1} \tau_{cmd}$$

When thruster $i$ is faulted ($W_{ii} \rightarrow 0$), the inverse weight $W_{ii}^{-1} \rightarrow 0$ automatically zeroes the command to that thruster, redistributing the required forces and moments to the remaining seven functioning thrusters.

---

## 5. 3D Line-of-Sight (LOS) Waypoint Guidance

Autonomous trajectory tracking along pipelines and inspection corridors is governed by a 3D Line-of-Sight (LOS) guidance law. Given waypoints $W_k$ and $W_{k+1}$, the vehicle computes horizontal cross-track error $e_y$ and vertical depth error $e_z$.

Commanded steering heading:

$$\psi_{cmd} = \psi_{path} + \arctan\left( -\frac{e_y}{\Delta} \right)$$

Commanded pitch angle:

$$\theta_{cmd} = \arctan\left( \frac{e_z}{\Delta} \right)$$

where $\Delta$ is the lookahead distance. This steering law guarantees asymptotic convergence to the pipeline centerline without overshoot.
