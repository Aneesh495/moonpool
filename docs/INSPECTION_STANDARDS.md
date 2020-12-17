# Subsea Pipeline Inspection and Integrity Standards

Moonpool implements autonomous non-destructive testing (NDT), cathodic protection survey, and structural defect assessment according to recognized subsea pipeline integrity standards:

1. **NACE SP0169 / ISO 15589-1**: Control of External Corrosion on Underground or Submerged Metallic Piping Systems
2. **DNV-RP-F105**: Free Spanning Pipelines (Vortex-Induced Vibration Screening)
3. **ASME B31G**: Manual for Determining the Remaining Strength of Corroded Pipelines
4. **Modified B31G (0.85 dL / RSTRENG)**: Refined Folias Bulging Factor Criterion for Long Corrosion Grooves
5. **DNV-RP-F101**: Corroded Pipelines (Part A: Single Defect Assessment)

```
+-----------------------------------------------------------------------------+
|               SUBSEA PIPELINE INTEGRITY ASSESSMENT PIPELINE                 |
+-----------------------------------------------------------------------------+
|                                                                             |
|  [Contact / Proximity NDT Sensors]                                          |
|         |                                                                   |
|         +---> Dual Ag/AgCl Half-Cells ----> NACE SP0169 CP Evaluation       |
|         |                                      (-850 mV to -1050 mV Window) |
|         |                                                                   |
|         +---> Multi-Channel Ultrasonic ---> A-Scan Time-of-Flight           |
|         |     Thickness Gauge                  (Thickness, Pitting Profile) |
|         |                                                                   |
|  [Bathymetry & Laser Profilometry]                                          |
|         |                                                                   |
|         +---> 3D Free Span Geometry ------> DNV-RP-F105 VIV Screening       |
|                                                (Eigenfrequency, Strouhal)   |
|                                                                             |
|  [Burst Pressure & Safe Operating Pressure Calculator]                     |
|         |                                                                   |
|         +---> ASME B31G Original & Modified RSTRENG Folias Bulging Factor   |
|         |                                                                   |
|         +---> Executive Decision: PASS / MONITOR / RESTRICT_MAOP / ABORT    |
|                                                                             |
+-----------------------------------------------------------------------------+
```

---

## 1. Cathodic Protection (CP) Survey Principles

Subsea steel pipelines are protected against marine galvanic corrosion via sacrificial aluminum-zinc-indium (Al-Zn-In) anodes mounted periodically along the line. Moonpool measures the pipeline electric potential using dual silver/silver chloride (Ag/AgCl) reference electrodes.

### 1.1 Potential Criteria (NACE SP0169 / ISO 15589-1)

Under standard seawater conditions (salinity ~35 PSU, temperature 4 deg C to 15 deg C), the pipe-to-electrolyte potential referenced to Ag/AgCl must satisfy:

| Potential Range ($V$ vs Ag/AgCl) | Assessment Category | Executive Action |
|:---------------------------------|:--------------------|:-----------------|
| $V > -0.800\text{ V}$ | **Severe Underprotection** | Critical alarm; active corrosion occurring |
| $-0.850\text{ V} < V \le -0.800\text{ V}$ | **Marginal / Underprotection** | Warning; inspect nearest anode bracelet |
| $-1.050\text{ V} \le V \le -0.850\text{ V}$ | **Optimal Cathodic Protection** | Nominal operating window |
| $-1.150\text{ V} \le V < -1.050\text{ V}$ | **High Protection** | Safe; monitored |
| $V < -1.150\text{ V}$ | **Hydrogen Overprotection** | Danger of hydrogen embrittlement in high-strength steels ($>\text{SMYS } 450\text{ MPa}$) |

### 1.2 Electric Field Gradient and Holiday Detection

When protective epoxy or polypropylene coating sustains mechanical damage (a coating "holiday"), protective ionic current rushes from the seawater into the exposed steel. This current creates an electric field gradient:

$$\Delta V = V_{\text{remote}} - V_{\text{contact}}$$

Moonpool computes the current density vector entering the defect to localize holidays with centimeter precision without requiring physical contact.

---

## 2. Sacrificial Anode Consumption and Life Prediction

Sacrificial bracelet anodes experience electrochemical dissolution governed by Faraday's Law:

$$m = \frac{I \cdot t}{Q}$$

where:
- $m$: Mass loss ($\text{kg}$)
- $I$: Mean galvanic current draw ($\text{A}$)
- $t$: Operational elapsed time ($\text{hours}$)
- $Q$: Electrochemical capacity of Al-Zn-In alloy ($\approx 2000 \text{ to } 2600\text{ A}\cdot\text{h/kg}$)

### Anode Resistance Formulation

Moonpool predicts remaining anode life using Dwight and McCoy resistance formulations for slender standoff and flush-mounted bracelet anodes:

$$\text{Dwight: } R = \frac{\rho}{2 \pi L} \left( \ln\left(\frac{4 L}{r}\right) - 1 \right)$$

$$\text{McCoy: } R = 0.315 \frac{\rho}{\sqrt{A}}$$

where:
- $\rho$: Seawater resistivity ($\Omega \cdot \text{m}$, derived from CTD salinity and temperature)
- $L$: Anode length ($\text{m}$)
- $r$: Equivalent anode radius ($\text{m}$)
- $A$: Exposed anode surface area ($\text{m}^2$)

---

## 3. Ultrasonic Thickness and Pitting Assessment

Moonpool's contact NDT payload integrates high-frequency (5 MHz to 10 MHz) piezoelectric transducers for pulse-echo longitudinal wave thickness gauging.

### 3.1 Time-of-Flight Acoustic Ranging

The steel wall thickness $t_w$ is calculated from the two-way transit time $\Delta t$ between front-surface and back-wall reflections:

$$t_w = \frac{c_{\text{steel}} \cdot \Delta t}{2}$$

where $c_{\text{steel}} \approx 5920\text{ m/s}$ (longitudinal sound velocity in API 5L X-grade line pipe steel).

### 3.2 Pitting Density and Wall Loss Metric

Wall loss percentage is defined as:

$$\% \text{ Loss} = \frac{t_{\text{nominal}} - t_{\text{measured}}}{t_{\text{nominal}}} \times 100$$

Defects exceeding 80% wall thickness loss are classified as through-wall integrity threats requiring immediate autonomous mission intervention.

---

## 4. Remaining Burst Strength Analysis (ASME B31G & Modified RSTRENG)

Corrosion metal loss lowers the internal pressure containment capacity of the pipeline. Moonpool computes the remaining burst pressure $P_f$ and safe maximum allowable operating pressure (MAOP).

### 4.1 ASME B31G Original Formulation

For parabolic corrosion defect profiles ($d/t \le 0.8$):

$$P_f = 1.1 \cdot P_{\text{MAOP}} \cdot \left[ \frac{1 - \frac{2}{3} \frac{d}{t}}{1 - \frac{2}{3} \frac{d}{t} M^{-1}} \right]$$

where Folias bulging stress concentration factor $M$ is given by:

$$M = \sqrt{1 + 0.8 \left(\frac{L}{\sqrt{D t}}\right)^2} \quad \text{for } \frac{L^2}{D t} \le 20$$

$$M = \infty \quad \text{for } \frac{L^2}{D t} > 20$$

### 4.2 Modified B31G (0.85 dL / RSTRENG)

The original B31G formulation can be excessively conservative for long, shallow corrosion grooves. The Modified B31G standard uses an effective area factor of 0.85 and a modified Folias factor:

$$M = \sqrt{1 + 0.6275 \left(\frac{L^2}{D t}\right) - 0.003375 \left(\frac{L^2}{D t}\right)^2} \quad \text{for } \frac{L^2}{D t} \le 50$$

$$M = 0.032 \left(\frac{L^2}{D t}\right) + 3.3 \quad \text{for } \frac{L^2}{D t} > 50$$

Moonpool evaluates both criteria in real time, alerting human pipeline operators when the safe design pressure drops below current operating pressure.

---

## 5. Free Span Vortex-Induced Vibration (DNV-RP-F105)

When subsea seabed scour removes benthic soil support beneath the pipe, an unsupported span is formed. Crossflow bottom currents shed alternating von Karman vortices that can trigger catastrophic resonant oscillations.

### 5.1 Natural Frequency of Pipe Span

The pinned-pinned fundamental eigenfrequency $f_0$ of an unsupported span of length $L_{\text{span}}$ is:

$$f_0 = \frac{\pi}{2 L_{\text{span}}^2} \sqrt{\frac{E I}{m_e}}$$

where:
- $E$: Elastic Young's modulus of carbon steel ($207 \times 10^9\text{ Pa}$)
- $I$: Second moment of area ($I = \frac{\pi}{64} (D_o^4 - D_i^4)$)
- $m_e$: Effective total unit mass including steel, concrete weight coating, internal fluid contents, and hydrodynamic added mass ($m_{\text{added}} = C_a \frac{\pi \rho_w D_o^2}{4}$)

### 5.2 Vortex Shedding and Strouhal Lock-In

Vortex shedding frequency $f_s$ at crossflow current velocity $U_c$:

$$f_s = \frac{St \cdot U_c}{D_o}$$

where Strouhal number $St \approx 0.20$ for high Reynolds number subsea flows. 

When the vortex shedding frequency approaches the structural eigenfrequency ($0.8 \le f_s / f_0 \le 1.3$), **lock-in** occurs. Moonpool triggers autonomous seafloor sandbagging/grouting intervention markers whenever a span exceeds critical screening limits.
