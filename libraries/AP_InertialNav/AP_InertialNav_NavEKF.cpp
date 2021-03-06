#include <AP_HAL/AP_HAL.h>
#include "AP_InertialNav.h"
#include <AP_HAL/AP_HAL.h>
#if AP_AHRS_NAVEKF_AVAILABLE
extern const AP_HAL::HAL& hal;
/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */

/**
   update internal state
*/
void AP_InertialNav_NavEKF::update(float dt)
{
    // get the NE position relative to the local earth frame origin
    Vector2f posNE;
    if (_ahrs_ekf.get_relative_position_NE_origin(posNE)) {
        _relpos_cm.x = posNE.x * 100; // convert from m to cm
        _relpos_cm.y = posNE.y * 100; // convert from m to cm
    }

    // get the D position relative to the local earth frame origin
    float posD;
    if (_ahrs_ekf.get_relative_position_D_origin(posD)) {
        _relpos_cm.z = - posD * 100; // convert from m in NED to cm in NEU
    }

    // get the absolute WGS-84 position
    _haveabspos = _ahrs_ekf.get_position(_abspos);

    // get the velocity relative to the local earth frame
    Vector3f velNED;
    if (_ahrs_ekf.get_velocity_NED(velNED)) {
        _velocity_cm = velNED * 100; // convert to cm/s
        _velocity_cm.z = -_velocity_cm.z; // convert from NED to NEU


    }

    // Get a derivative of the vertical position which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
    if (_ahrs_ekf.get_vert_pos_rate(_pos_z_rate)) {
        _pos_z_rate *= 100; // convert to cm/s
        _pos_z_rate = - _pos_z_rate; // InertialNav is NEU
    }
    _extNavPos = _extNav.get_position();
    _extNavPos.z *= -1;
    _extNavVel = _extNav.get_velocity();
    _extNavVel.z *= -1;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL


    if (AP_HAL::millis64() - lastPosVelAtt >= 10)
    {
        sitl = AP::sitl();
        Vector3f rpy;
        rpy.x = sitl->state.rollDeg;
        rpy.y = sitl->state.pitchDeg;
        rpy.z = sitl->state.yawDeg;
        Vector3f vel;
        vel.x = sitl->state.speedN*100;
        vel.y = sitl->state.speedE*100;
        vel.z = -sitl->state.speedD*100;

        Location_Class current;
        current.lat = sitl->state.latitude*1e7;
        current.lng = sitl->state.longitude*1e7;
        current.alt = sitl->state.altitude*100;


        Vector3f neu;
        current.get_vector_from_origin_NEU(neu);
        /*if(AP_HAL::millis64() - printCall >= 6000)
       {
           hal.console->printf("current.lat: %d, current.lng: %d, current.alt: %d\n", current.lat, current.lng, current.alt);
           hal.console->printf("neu.x: %f, neu.y: %f, neu.up: %f\n", neu.x, neu.y, neu.z);
           printCall = AP_HAL::millis();
       } */
        _extNav.setExtPosVelAtt(neu,vel, rpy);

        lastPosVelAtt = AP_HAL::millis64();
    }
#endif

}

/**
 * get_filter_status : returns filter status as a series of flags
 */
nav_filter_status AP_InertialNav_NavEKF::get_filter_status() const
{
    nav_filter_status status;
    _ahrs_ekf.get_filter_status(status);
    return status;
}

/**
 * get_origin - returns the inertial navigation origin in lat/lon/alt
 */
struct Location AP_InertialNav_NavEKF::get_origin() const
{
    struct Location ret;
     if (!_ahrs_ekf.get_origin(ret)) {
         // initialise location to all zeros if EKF origin not yet set
         memset(&ret, 0, sizeof(ret));
     }
    return ret;
}

/**
 * get_position - returns the current position relative to the home location in cm.
 *
 * @return
 */
const Vector3f &AP_InertialNav_NavEKF::get_position(void) const 
{
    if (_extNav.forcePosition() == 1)
    {
        return _extNavPos;
    }
    return _relpos_cm;

}

/**
 * get_location - updates the provided location with the latest calculated location
 *  returns true on success (i.e. the EKF knows it's latest position), false on failure
 */
bool AP_InertialNav_NavEKF::get_location(struct Location &loc) const
{
    return _ahrs_ekf.get_location(loc);
}

/**
 * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 */
int32_t AP_InertialNav_NavEKF::get_latitude() const
{
    return _abspos.lat;
}

/**
 * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 * @return
 */
int32_t AP_InertialNav_NavEKF::get_longitude() const
{
    return _abspos.lng;
}

/**
 * get_velocity - returns the current velocity in cm/s
 *
 * @return velocity vector:
 *      		.x : North  velocity in cm/s
 * 				.y : East velocity in cm/s
 * 				.z : Up  velocity in cm/s
 */
const Vector3f &AP_InertialNav_NavEKF::get_velocity() const
{
    if (_extNav.forceVelocity() == 1) return _extNavVel;
    return _velocity_cm;
}

/**
 * get_velocity_xy - returns the current horizontal velocity in cm/s
 *
 * @returns the current horizontal velocity in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_xy() const
{
    if (_extNav.forceVelocity() == 1) return norm(_extNavVel.x, _extNavVel.y);
    return norm(_velocity_cm.x, _velocity_cm.y);
}

/**
 * get_pos_z_derivative - returns the derivative of the z position in cm/s
*/
float AP_InertialNav_NavEKF::get_pos_z_derivative() const
{
    if (_extNav.forceVelocity() == 1) return _extNavVel.z;
    return _pos_z_rate;
}

/**
 * get_altitude - get latest altitude estimate in cm
 * @return
 */
float AP_InertialNav_NavEKF::get_altitude() const
{
    if (_extNav.forcePosition() == 1) return _extNavPos.z;

    return _relpos_cm.z;
}

/**
 * get_velocity_z - returns the current climbrate.
 *
 * @see get_velocity().z
 *
 * @return climbrate in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_z() const
{
    if (_extNav.forceVelocity() == 1) return _extNavVel.z;
    return _velocity_cm.z;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
