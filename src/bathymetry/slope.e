-- Moonpool Subsea Robotics Bathymetry and Terrain Mapping
-- Benthic Terrain Slope Gradient, Aspect Angle, Surface Normal, and Rugosity
-- OpenEuphoria v4.1+ Specification

namespace moonpool_slope

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e

-- Calculate seabed slope gradient and aspect at specific cell (r, c) using Horn's algorithm
-- dem: DEM structure from dem_grid.e
-- Returns sequence {slope_deg, aspect_deg, normal_vec, rugosity}
public function dem_cell_slope(sequence dem, integer r, integer c)
    integer rows = dem[3]
    integer cols = dem[4]
    atom cell_size = dem[5]
    sequence depth_grid = dem[6]

    -- Clamp neighborhood indices to boundary
    integer rm1 = clamp(r - 1, 1, rows)
    integer rp1 = clamp(r + 1, 1, rows)
    integer cm1 = clamp(c - 1, 1, cols)
    integer cp1 = clamp(c + 1, 1, cols)

    -- 3x3 depth neighborhood:
    -- z_nw, z_n, z_ne
    -- z_w,  z_c, z_e
    -- z_sw, z_s, z_se
    atom z_nw = depth_grid[rm1][cm1]
    atom z_n  = depth_grid[rm1][c]
    atom z_ne = depth_grid[rm1][cp1]

    atom z_w  = depth_grid[r][cm1]
    atom z_e  = depth_grid[r][cp1]

    atom z_sw = depth_grid[rp1][cm1]
    atom z_s  = depth_grid[rp1][c]
    atom z_se = depth_grid[rp1][cp1]

    -- Horn's partial derivatives
    atom dz_dx = ((z_ne + 2.0 * z_e + z_se) - (z_nw + 2.0 * z_w + z_sw)) / (8.0 * cell_size)
    atom dz_dy = ((z_sw + 2.0 * z_s + z_se) - (z_nw + 2.0 * z_n + z_ne)) / (8.0 * cell_size)

    atom grad_mag = sqrt(dz_dx * dz_dx + dz_dy * dz_dy)
    atom slope_rad = arctan(grad_mag)
    atom slope_deg = slope_rad * RAD2DEG

    -- Aspect: compass direction of steepest downward descent
    atom aspect_rad = atan2(-dz_dy, dz_dx)
    atom aspect_deg = wrap_360(aspect_rad * RAD2DEG)

    -- Surface normal vector pointing upwards: n = {-dz_dx, -dz_dy, 1} normalized
    sequence norm_vec = vec_normalize({-dz_dx, -dz_dy, 1.0})

    -- Rugosity: 3D actual surface area divided by planar 2D footprint area
    atom rugosity = sqrt(1.0 + grad_mag * grad_mag)

    return {slope_deg, aspect_deg, norm_vec, rugosity}
end function

-- Compute complete slope raster grid (in degrees) for the entire DEM
public function dem_compute_slope_grid(sequence dem)
    integer rows = dem[3]
    integer cols = dem[4]
    sequence slope_grid = mat_zeros(rows, cols)

    for r = 1 to rows do
        for c = 1 to cols do
            sequence res = dem_cell_slope(dem, r, c)
            slope_grid[r][c] = res[1]
        end for
    end for

    return slope_grid
end function
