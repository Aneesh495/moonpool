-- Moonpool Subsea Robotics Core
-- Physical Constants, WGS-84 Geodetic Reference Ellipsoid, and Oceanographic Defaults
-- OpenEuphoria v4.1+ Specification

namespace moonpool_constants

include std/math.e

-- Mathematical Constants derived from std/math PI
public constant TWO_PI     = 2.0 * PI
public constant HALF_PI    = PI / 2.0
public constant DEG2RAD    = PI / 180.0
public constant RAD2DEG    = 180.0 / PI
public constant EPSILON    = 1.0e-9

-- Standard Oceanographic Physical Parameters
public constant SOUND_SPEED_DEFAULT  = 1500.0   -- Default sound speed in seawater (m/s)
public constant SEAWATER_DENSITY      = 1025.0   -- Nominal seawater density (kg/m^3)
public constant GRAVITY_STANDARD      = 9.80665  -- Standard gravitational acceleration (m/s^2)
public constant BULK_MODULUS_WATER    = 2.2e9    -- Bulk modulus of elasticity of seawater (Pa)
public constant KINEMATIC_VISCOSITY   = 1.05e-6  -- Kinematic viscosity of seawater at 10 deg C (m^2/s)
public constant DYNAMIC_VISCOSITY     = 1.08e-3  -- Dynamic viscosity of seawater (Pa*s)
public constant ATMOSPHERIC_PRESSURE  = 101325.0 -- Standard atmospheric pressure at sea level (Pa)
public constant DBAR2PA               = 10000.0  -- 1 decibar = 10,000 Pascals

-- WGS-84 Ellipsoid Geodetic Constants
public constant WGS84_A       = 6378137.0                -- Semi-major axis (meters)
public constant WGS84_F       = 1.0 / 298.257223563      -- Flattening factor
public constant WGS84_B       = 6356752.314245           -- Semi-minor axis (meters)
public constant WGS84_E2      = 6.69437999014e-3         -- First eccentricity squared (e^2)
public constant WGS84_E_PRIME2 = 6.73949674228e-3        -- Second eccentricity squared (e'^2)
public constant EARTH_ROTATION_RATE = 7.292115e-5        -- Earth angular velocity (rad/s)

-- Subsea Vehicle Standard Acoustic Limits
public constant MAX_ACOUSTIC_RANGE = 8000.0  -- Maximum tracking range for deepwater LBL/USBL (meters)
public constant MIN_ACOUSTIC_RANGE = 2.0     -- Near-field acoustic boundary limit (meters)

-- Pipeline Non-Destructive Testing Acoustic Velocity Constants
public constant SOUND_SPEED_STEEL_LONGITUDINAL = 5900.0 -- Speed of sound in API 5L steel (m/s)
public constant SOUND_SPEED_STEEL_SHEAR        = 3230.0 -- Shear wave velocity in steel (m/s)
