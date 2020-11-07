-- Moonpool Subsea Robotics Pipeline Engineering
-- 3D Subsea Pipeline Route Geometry, KP Stations, Burial Depth, and Riser Modeling
-- OpenEuphoria v4.1+ Specification

namespace moonpool_pipe_geometry

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e

-- Create a pipeline route specification:
-- D_outer_m: outer diameter of steel pipe (meters, e.g. 0.6096m = 24 inches)
-- t_wall_m: nominal wall thickness (meters, e.g. 0.0191m)
-- t_coating_m: anticorrosion coating thickness (meters, e.g. 0.0035m)
-- t_cwc_m: concrete weight coating thickness (meters, e.g. 0.050m)
-- rho_steel: density of steel (kg/m^3, standard 7850 kg/m^3)
-- rho_cwc: density of concrete weight coating (kg/m^3, standard 2400 to 3040 kg/m^3)
public function pipe_spec_create(
    atom D_outer_m,
    atom t_wall_m,
    atom t_coating_m,
    atom t_cwc_m,
    atom rho_steel,
    atom rho_cwc
)
    return {D_outer_m, t_wall_m, t_coating_m, t_cwc_m, rho_steel, rho_cwc}
end function

-- Calculate total submerged weight per unit length (N/m) in seawater:
-- Positive value means pipe has negative buoyancy (sits stably on seabed)
public function pipe_submerged_weight(sequence spec, atom seawater_density_kg_m3)
    atom D_steel = spec[1]
    atom t_wall = spec[2]
    atom t_coat = spec[3]
    atom t_cwc = spec[4]
    atom rho_steel = spec[5]
    atom rho_cwc = spec[6]

    atom D_inner = D_steel - 2.0 * t_wall
    atom D_coated = D_steel + 2.0 * t_coat
    atom D_total = D_coated + 2.0 * t_cwc

    -- Cross-sectional areas
    atom A_steel = 0.25 * PI * (D_steel * D_steel - D_inner * D_inner)
    atom A_cwc = 0.25 * PI * (D_total * D_total - D_coated * D_coated)
    atom A_displaced = 0.25 * PI * (D_total * D_total)

    -- Weight in air (kg/m)
    atom mass_air = A_steel * rho_steel + A_cwc * rho_cwc
    -- Buoyancy force (kg/m)
    atom mass_buoyancy = A_displaced * seawater_density_kg_m3

    atom net_mass_kg_m = mass_air - mass_buoyancy
    return net_mass_kg_m * GRAVITY_STANDARD
end function

-- Compute 3D position along pipeline route given Kilometric Post (KP in meters):
-- route_waypoints = {{kp0, n0, e0, d0}, {kp1, n1, e1, d1}, ...}
public function pipe_route_kp_to_pos(sequence route_waypoints, atom kp_m)
    integer n = length(route_waypoints)
    if n = 0 then
        return {0.0, 0.0, 0.0}
    elsif n = 1 then
        return {route_waypoints[1][2], route_waypoints[1][3], route_waypoints[1][4]}
    end if

    if kp_m <= route_waypoints[1][1] then
        return {route_waypoints[1][2], route_waypoints[1][3], route_waypoints[1][4]}
    end if

    if kp_m >= route_waypoints[n][1] then
        return {route_waypoints[n][2], route_waypoints[n][3], route_waypoints[n][4]}
    end if

    for i = 1 to n - 1 do
        atom kp0 = route_waypoints[i][1]
        atom kp1 = route_waypoints[i + 1][1]
        if kp_m >= kp0 and kp_m <= kp1 then
            atom ratio = (kp_m - kp0) / (kp1 - kp0)
            atom n_pos = lerp(route_waypoints[i][2], route_waypoints[i + 1][2], ratio)
            atom e_pos = lerp(route_waypoints[i][3], route_waypoints[i + 1][3], ratio)
            atom d_pos = lerp(route_waypoints[i][4], route_waypoints[i + 1][4], ratio)
            return {n_pos, e_pos, d_pos}
        end if
    end for

    return {route_waypoints[n][2], route_waypoints[n][3], route_waypoints[n][4]}
end function

-- Calculate pipeline burial depth (depth of cover) from subsea terrain profile:
-- pipe_top_depth_m: depth of top of pipe below sea surface (meters)
-- seabed_depth_m: bathymetric seabed depth below sea surface (meters)
-- Returns depth of cover in meters:
-- positive = buried under sediment; negative = exposed or in free span!
public function pipe_depth_of_cover(atom pipe_top_depth_m, atom seabed_depth_m)
    return pipe_top_depth_m - seabed_depth_m
end function
