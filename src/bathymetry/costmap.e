-- Moonpool Subsea Robotics Bathymetry and Terrain Mapping
-- Autonomous Navigation Costmap, Slope Hazard Penalties, and Exclusion Zones
-- OpenEuphoria v4.1+ Specification

namespace moonpool_costmap

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e
include src/bathymetry/dem_grid.e
include src/bathymetry/slope.e

-- Create subsea autonomous navigation costmap structure:
-- {dem, slope_grid, max_slope_deg, min_depth_m, max_depth_m, exclusion_zones}
-- exclusion_zones = {{center_n, center_e, radius_m, penalty_cost}, ...}
public function costmap_create(
    sequence dem,
    atom max_slope_deg,
    atom min_depth_m,
    atom max_depth_m
)
    sequence slope_grid = dem_compute_slope_grid(dem)
    sequence exclusion_zones = {}

    return {dem, slope_grid, max_slope_deg, min_depth_m, max_depth_m, exclusion_zones}
end function

-- Add a circular exclusion / obstacle zone around subsea infrastructure (e.g. wellhead, anchor line)
public function costmap_add_exclusion_zone(
    sequence costmap,
    atom center_n,
    atom center_e,
    atom radius_m,
    atom penalty_cost
)
    sequence zones = costmap[6]
    zones = append(zones, {center_n, center_e, radius_m, penalty_cost})
    costmap[6] = zones
    return costmap
end function

-- Evaluate continuous traversability cost at {north_m, east_m}
-- Returns normalized cost in range [0.0, 1.0] where 1.0 indicates impassable hazard
public function costmap_query_cost(sequence costmap, atom north_m, atom east_m)
    sequence dem = costmap[1]
    sequence slope_grid = costmap[2]
    atom max_slope = costmap[3]
    atom min_depth = costmap[4]
    atom max_depth = costmap[5]
    sequence zones = costmap[6]

    atom origin_n = dem[1]
    atom origin_e = dem[2]
    integer rows = dem[3]
    integer cols = dem[4]
    atom cell_size = dem[5]
    sequence depth_grid = dem[6]

    integer r = floor((north_m - origin_n) / cell_size) + 1
    integer c = floor((east_m - origin_e) / cell_size) + 1

    -- Out of bounds check
    if r < 1 or r > rows or c < 1 or c > cols then
        return 1.0
    end if

    atom depth = depth_grid[r][c]
    atom slope = slope_grid[r][c]
    atom total_cost = 0.0

    -- 1. Slope hazard penalty
    if slope >= max_slope then
        return 1.0 -- Impassable cliff
    elsif slope > 10.0 then
        total_cost += (slope - 10.0) / (max_slope - 10.0) * 0.5
    end if

    -- 2. Operating depth window penalty
    if depth < min_depth or depth > max_depth then
        total_cost += 0.4
    end if

    -- 3. Exclusion zones proximity check
    for i = 1 to length(zones) do
        sequence z = zones[i]
        atom dn = north_m - z[1]
        atom de = east_m - z[2]
        atom dist = sqrt(dn * dn + de * de)
        atom radius = z[3]
        atom penalty = z[4]

        if dist < radius then
            return 1.0 -- Inside exclusion perimeter
        elsif dist < 2.0 * radius then
            -- Smooth buffer decay
            atom ratio = (2.0 * radius - dist) / radius
            total_cost += ratio * penalty
        end if
    end for

    return clamp(total_cost, 0.0, 1.0)
end function
