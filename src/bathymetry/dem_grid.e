-- Moonpool Subsea Robotics Bathymetry and Terrain Mapping
-- 2.5D Digital Elevation Model (DEM) Bathymetric Raster Grid and Spatial Aggregation
-- OpenEuphoria v4.1+ Specification

namespace moonpool_dem_grid

include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e

-- Create a 2.5D Digital Elevation Model grid structure:
-- origin_n, origin_e: local NED coordinates of the grid's South-West corner (meters)
-- rows, cols: raster dimensions
-- cell_size_m: grid resolution in meters (e.g. 1.0m or 0.5m)
-- default_depth_m: initial depth fill value (meters)
public function dem_create(
    atom origin_n,
    atom origin_e,
    integer rows,
    integer cols,
    atom cell_size_m,
    atom default_depth_m
)
    sequence depth_grid = mat_zeros(rows, cols)
    sequence count_grid = mat_zeros(rows, cols)
    sequence var_grid = mat_zeros(rows, cols)

    for r = 1 to rows do
        for c = 1 to cols do
            depth_grid[r][c] = default_depth_m
        end for
    end for

    return {origin_n, origin_e, rows, cols, cell_size_m, depth_grid, count_grid, var_grid}
end function

-- Insert a single sounding point {north, east, depth} into DEM raster
-- Uses online running average and sample variance (Welford's algorithm)
public function dem_insert_sounding(sequence dem, atom n_m, atom e_m, atom depth_m)
    atom origin_n = dem[1]
    atom origin_e = dem[2]
    integer rows = dem[3]
    integer cols = dem[4]
    atom cell_size = dem[5]
    sequence depth_grid = dem[6]
    sequence count_grid = dem[7]
    sequence var_grid = dem[8]

    integer r_idx = floor((n_m - origin_n) / cell_size) + 1
    integer c_idx = floor((e_m - origin_e) / cell_size) + 1

    if r_idx >= 1 and r_idx <= rows and c_idx >= 1 and c_idx <= cols then
        atom count = count_grid[r_idx][c_idx]
        atom old_mean = depth_grid[r_idx][c_idx]
        atom new_count = count + 1.0

        if count = 0.0 then
            depth_grid[r_idx][c_idx] = depth_m
            var_grid[r_idx][c_idx] = 0.0
        else
            atom delta = depth_m - old_mean
            atom new_mean = old_mean + delta / new_count
            atom delta2 = depth_m - new_mean
            atom m2 = var_grid[r_idx][c_idx] * (count - 1.0) + delta * delta2
            depth_grid[r_idx][c_idx] = new_mean
            if new_count > 1.0 then
                var_grid[r_idx][c_idx] = m2 / (new_count - 1.0)
            end if
        end if
        count_grid[r_idx][c_idx] = new_count
    end if

    dem[6] = depth_grid
    dem[7] = count_grid
    dem[8] = var_grid
    return dem
end function

-- Insert a sequence of sounding points into DEM in batch
public function dem_insert_batch(sequence dem, sequence soundings)
    for i = 1 to length(soundings) do
        sequence pt = soundings[i]
        dem = dem_insert_sounding(dem, pt[1], pt[2], pt[3])
    end for
    return dem
end function

-- Retrieve interpolated seabed depth at arbitrary continuous coordinates {n_m, e_m}
-- using bilinear interpolation across four adjacent cells
public function dem_get_depth_bilinear(sequence dem, atom n_m, atom e_m)
    atom origin_n = dem[1]
    atom origin_e = dem[2]
    integer rows = dem[3]
    integer cols = dem[4]
    atom cell_size = dem[5]
    sequence depth_grid = dem[6]

    atom r_float = (n_m - origin_n) / cell_size + 0.5
    atom c_float = (e_m - origin_e) / cell_size + 0.5

    integer r0 = floor(r_float)
    integer c0 = floor(c_float)
    integer r1 = r0 + 1
    integer c1 = c0 + 1

    r0 = clamp(r0, 1, rows)
    r1 = clamp(r1, 1, rows)
    c0 = clamp(c0, 1, cols)
    c1 = clamp(c1, 1, cols)

    atom dr = clamp(r_float - r0, 0.0, 1.0)
    atom dc = clamp(c_float - c0, 0.0, 1.0)

    atom d00 = depth_grid[r0][c0]
    atom d01 = depth_grid[r0][c1]
    atom d10 = depth_grid[r1][c0]
    atom d11 = depth_grid[r1][c1]

    atom top_interp = lerp(d00, d01, dc)
    atom bot_interp = lerp(d10, d11, dc)

    return lerp(top_interp, bot_interp, dr)
end function
