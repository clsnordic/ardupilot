/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */
#pragma once

#include <AP_NavEKF/AP_Nav_Common.h>              // definitions shared by inertial and ekf nav filters
#include <AC_Ext_Nav/AC_Ext_Nav.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

class AP_InertialNav_NavEKF : public AP_InertialNav
{
public:
    // Constructor
    AP_InertialNav_NavEKF(AP_AHRS_NavEKF &ahrs) :
        AP_InertialNav(),
        _haveabspos(false),
        _ahrs_ekf(ahrs),
        _extNav(AC_Ext_Nav::get_instance())
        {}

    /**
       update internal state
    */
    void        update(float dt);

    /**
     * get_filter_status - returns filter status as a series of flags
     */
    nav_filter_status get_filter_status() const;

    /**
     * get_origin - returns the inertial navigation origin in lat/lon/alt
     *
     * @return origin Location
     */
    struct Location get_origin() const;

    /**
     * get_position - returns the current position relative to the home location in cm.
     *
     * the home location was set with AP_InertialNav::set_home_position(int32_t, int32_t)
     *
     * @return
     */
    const Vector3f&    get_position() const;

    /**
     * get_llh - updates the provided location with the latest calculated location including absolute altitude
     *  returns true on success (i.e. the EKF knows it's latest position), false on failure
     */
    bool get_location(struct Location &loc) const;

    /**
     * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     */
    int32_t     get_latitude() const;

    /**
     * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
     * @return
     */
    int32_t     get_longitude() const;

    /**
     * get_velocity - returns the current velocity in cm/s
     *
     * @return velocity vector:
     *      		.x : North  velocity in cm/s
     * 				.y : East velocity in cm/s
     * 				.z : Up  velocity in cm/s
     */
    const Vector3f&    get_velocity() const;

    /**
     * get_pos_z_derivative - returns the derivative of the z position in cm/s
    */
    float    get_pos_z_derivative() const;

    /**
     * get_velocity_xy - returns the current horizontal velocity in cm/s
     *
     * @returns the current horizontal velocity in cm/s
     */
    float        get_velocity_xy() const;

    /**
     * get_altitude - get latest altitude estimate in cm
     * @return
     */
    float       get_altitude() const;

    /**
     * get_velocity_z - returns the current climbrate.
     *
     * @see get_velocity().z
     *
     * @return climbrate in cm/s
     */
    float       get_velocity_z() const;

private:



#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        //ENU
        Vector3f _latestPos;
        Vector3f _latestVel;
        //Roll pitch yaw
        Vector3f _latestAng;
        uint64_t lastPosVelAtt;


        SITL::SITL *sitl;
#endif

    Vector3f _extNavPos = Vector3f(0,0,0);
    Vector3f _extNavVel = Vector3f(0,0,0);
    uint32_t printCall;
    Vector3f _relpos_cm;   // NEU
    Vector3f _velocity_cm; // NEU
    float _pos_z_rate;
    struct Location _abspos;
    bool _haveabspos;
    AP_AHRS_NavEKF &_ahrs_ekf;
    AC_Ext_Nav      &_extNav;
};
