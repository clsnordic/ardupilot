/*
 * AC_Ext_Nav.h
 *
 *  Created on: 8 May 2018
 *      Author: Christoffer
 */
#pragma once
#ifndef AC_EXT_NAV_H_
#define AC_EXT_NAV_H_

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/AP_HAL.h>
//#include <Dataflash/Dataflash.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_Mavlink/GCS.h>
//#include <Filter/Filter.h>                     // Filter library
//#include <Filter/LowPassFilter2p.h>


extern const AP_HAL::HAL& hal;
class AC_Ext_Nav {
public:
    friend class GCS_MAVLINK;
    virtual ~AC_Ext_Nav();
    AC_Ext_Nav( const AC_Ext_Nav&) = delete;
    AC_Ext_Nav operator=(const AC_Ext_Nav&) = delete;

    void update();

    void init(const AP_SerialManager &serial_manager);

    //_latestPosition
    //      .x = Position in North (cm)
    //      .y = Position in East (cm)
    //      .z = Position in Up (cm)
    inline Vector3f get_position()  {

        return _latestPosition;
    }
    //_latestVelocity
    //      .x = Velocity in North (cm/s)
    //      .y = Velocity in East (cm/s)
    //      .z = Velocity in Up (cm/s)
    const inline Vector3f get_velocity()  {
        return _latestVelocity;
    }
    static const struct AP_Param::GroupInfo var_info[];

    inline bool extNavPosEnabled() const {
        if (!_hasReceivedPos) return false;
        //return (bool)_extNavPosEnabled;
        //TODOCLS testing aiding in the ATT_POS_MOCAP message, always return false here for now
        return false;
    }

    inline bool extNavCtrlEnabled() const {
        if (!_hasReceivedCtrl) return false;
        //return (bool)_extNavCtrlEnabled;
        //TODOCLS testing aiding in the ATT_POS_MOCAP message, always return false here for now
        return false;
    }
    /*inline AP_Int8 enableLowLevelCtrl() const {
        return _extLowLevelCtrlEnabled;
    } */
    inline bool aidingEnabled() const {

            //return (bool)_extNavPosEnabled;
            //TODOCLS testing aiding in the ATT_POS_MOCAP message, always return false here for now
            return (bool)_aidingEnabled;
        }

    inline Vector3f getLatestGyro() {
        return _latestGyroMeasurements;
    }

    inline float getYaw() {
        return _latestAngleMeasurement.z;
    }
    inline float getRoll() {
            return _latestAngleMeasurement.x;
    }
    inline float getPitch() {
            return _latestAngleMeasurement.y;
    }
    inline bool recCtrlData() const {
        return _hasReceivedCtrl;
    }
    inline bool recPosData() const {
        return _hasReceivedPos;
        }
    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    void setExtCtrl(Vector3f &gyro, Vector3f& accel);
    void setExtPosVelAtt(Vector3f &pos, Vector3f& vel, Vector3f& ang);
    #endif

    void handleMsg(mavlink_message_t *msg);
    void storeData(mavlink_message_t *msg);
    //AP_Int8 _extNavPosEnabled;

    static AC_Ext_Nav& get_instance() {
             static AC_Ext_Nav instance;
             return instance;
         }

    // GCS &gcs()
//extern const AP_HAL::HAL& hal;

private:
    // craete an instance with 100Hz sample rate and 30Hz cutoff

    AC_Ext_Nav();

    //AC_Ext_Nav(const AC_Ext_Nav&) = delete;
    //AC_Ext_Nav& operator=(const AC_Ext_Nav&) = delete;

    AP_HAL::UARTDriver *_port;                  // UART used to send data to external
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter

    uint32_t extNavCalled = 0;
    uint32_t _msLastPosRec = 0;
    uint32_t _msLastCtrlRec = 0;


    Vector3f _extNavRate;

    //_latestAngleMeasurement
    //      .x = Roll angle (rad)
    //      .y = Pitch angle (rad)
    //      .z = Yaw angle (rad)
    Vector3f _latestAngleMeasurement = Vector3f(0,0,0);;
    //_latestPosition
    //      .x = Position in North (cm)
    //      .y = Position in East (cm)
    //      .z = Position in Up (cm)
    Vector3f _latestPosition = Vector3f(0,0,0);;

    //_latestVelocity
    //      .x = Velocity in North (cm/s)
    //      .y = Velocity in East (cm/s)
    //      .z = Velocity in Up (cm/s)
    Vector3f _latestVelocity = Vector3f(0,0,0);;

    Vector3f _latestAccelerations = Vector3f(0,0,0);;
    //_latestGyroMeasurements
    //      .x = Angular velocity roll axis (rad/s)
    //      .y = Angular velocity pitch axis (rad/s)
    //      .z = Angular velocity yaw axis (rad/s)
    Vector3f _latestGyroMeasurements = Vector3f(0,0,0);;

    static AC_Ext_Nav _s_instance;

    uint32_t firstCall;

    bool verifyPV(const mavlink_ext_nav_posvelatt_t &packet);
    bool verifyRA(const mavlink_ext_nav_ctrl_t &packet);

    bool _hasReceivedPos = false;
    bool _hasReceivedCtrl = false;

    float _currYaw;

    AP_Int8 _extNavPosEnabled;
    AP_Int8 _extNavCtrlEnabled;
    AP_Int8 _aidingEnabled;
    uint32_t _lastAttPosMocap;



    int setTimer = 0;
    //AP_Int8 _extLowLevelCtrlEnabled;

    void storeAngRates(mavlink_ext_nav_ctrl_t &packet, Vector3f &gyroMeas);
    void storeAccel(mavlink_ext_nav_ctrl_t &packet, Vector3f &accel);
    void sendAttPosMsg(uint32_t& loggedTime, uint32_t& timestamp_ms, Vector3f& pos, Vector3f& ang);




  //  LowPassFilter2pVector3f low_pass_filter{100, 10};



};

#endif /* AC_EXT_NAV_H_ */
