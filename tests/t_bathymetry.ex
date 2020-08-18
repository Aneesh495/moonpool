-- Moonpool Unit Test Suite: Multibeam Swath Projection, DEM Raster Grids, Slope, and Costmaps
-- OpenEuphoria v4.1+

include std/unittest.e
include std/math.e
include src/core/constants.e
include src/core/math_util.e
include src/core/matrix.e
include src/core/quaternion.e
include src/bathymetry/multibeam.e
include src/bathymetry/dem_grid.e
include src/bathymetry/slope.e
include src/bathymetry/costmap.e

set_test_verbosity(TEST_SHOW_ALL)

-- 1. Multibeam Sonar Tests
sequence mbes_cfg = mbes_config_create(5, 120.0, {0.0, 0.0, 1.0}, 400000.0)
test_equal("MBES beam count", 5, mbes_cfg[1])

sequence angles = mbes_beam_angles(mbes_cfg)
test_equal("MBES angles count", 5, length(angles))
-- Middle beam is nadir (0 deg)
test_true("MBES center beam angle 0", approx_equal(0.0, angles[3], 1e-5))

-- Project swath with vehicle at {0, 0, 50m} and 20m altitude above seabed
-- Nadir depth should be 70m
sequence travel_times = {0.04, 0.03, 0.0266667, 0.03, 0.04} -- TWTTs for beams
sequence soundings = mbes_project_swath(
    mbes_cfg,
    {0.0, 0.0, 50.0},
    quat_identity(),
    travel_times,
    1500.0
)
test_equal("Swath soundings produced", 5, length(soundings))
sequence nadir_pt = soundings[3]
test_true("Nadir sounding depth", approx_equal(71.0, nadir_pt[3], 1e-1)) -- 50m veh + 1m lever + 20m range = 71m

atom swath_w = mbes_swath_width(20.0, 120.0)
-- 2 * 20 * tan(60 deg) = 40 * sqrt(3) ~ 69.28m
test_true("Swath width at 20m alt", approx_equal(69.28, swath_w, 0.1))

-- 2. DEM Raster Grid Tests
-- Create a 20x20 DEM with 1m cell size at origin {0, 0}
sequence dem = dem_create(0.0, 0.0, 20, 20, 1.0, 100.0)
test_equal("DEM rows", 20, dem[3])
test_equal("DEM cols", 20, dem[4])

-- Insert soundings into cell (5, 5) -> N=4.5, E=4.5
dem = dem_insert_sounding(dem, 4.5, 4.5, 95.0)
dem = dem_insert_sounding(dem, 4.5, 4.5, 93.0)
atom d_interp = dem_get_depth_bilinear(dem, 4.5, 4.5)
test_true("DEM sounding averaged depth", approx_equal(94.0, d_interp, 1e-1))

-- 3. Slope and Horn's Algorithm Tests
-- Create an inclined planar seabed: depth = 100.0 + 0.5 * east (slope = arctan(0.5) ~ 26.56 deg)
sequence inclined_dem = dem_create(0.0, 0.0, 10, 10, 1.0, 100.0)
for r = 1 to 10 do
    for c = 1 to 10 do
        atom e_coord = (c - 1) * 1.0
        inclined_dem[6][r][c] = 100.0 + 0.5 * e_coord
    end for
end for

sequence slope_info = dem_cell_slope(inclined_dem, 5, 5)
atom slope_deg = slope_info[1]
-- arctan(0.5) * 180 / PI = 26.565 deg
test_true("Horn slope gradient calculation", approx_equal(26.565, slope_deg, 0.1))
test_true("Rugosity > 1.0 for inclined slope", slope_info[4] > 1.0)

-- 4. Costmap Tests
sequence cmap = costmap_create(inclined_dem, 30.0, 50.0, 150.0)
-- Cell with 26.5 deg slope is passable but has positive penalty
atom cost_slope = costmap_query_cost(cmap, 4.5, 4.5)
test_true("Moderate slope cost positive", cost_slope > 0.1 and cost_slope < 1.0)

-- Add an exclusion zone around subsea manifold at {5.0, 5.0} radius 2m
cmap = costmap_add_exclusion_zone(cmap, 5.0, 5.0, 2.0, 0.8)
atom cost_inside = costmap_query_cost(cmap, 5.0, 5.0)
test_equal("Inside exclusion zone is lethal obstacle", 1.0, cost_inside)

test_report()
