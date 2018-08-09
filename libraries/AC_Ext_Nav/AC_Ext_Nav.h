/*
 * AC_Ext_Nav.h
 *
 *  Created on: 8 May 2018
 *      Author: Christoffer
 */

#ifndef AC_EXT_NAV_H_
#define AC_EXT_NAV_H_
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/AP_HAL.h>
#include <Dataflash/Dataflash.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>


class AC_Ext_Nav {
public:
    AC_Ext_Nav();
    virtual ~AC_Ext_Nav();

    void update();

    void init(const AP_SerialManager &serial_manager);
    inline Vector3f get_position() const {
        return _extNavPos;
    }
    inline Vector3f get_velocity() const {
        return _extNavVel;
    }
    static const struct AP_Param::GroupInfo var_info[];

    inline AP_Int8 extNavPosEnabled() const {
        return _extNavPosEnabled;
    }

    inline AP_Int8 extNavCtrlEnabled() const {
        return _extNavCtrlEnabled;
    }
    inline AP_Int8 enableLowLevelCtrl() const {
        return _extLowLevelCtrlEnabled;
    }

    inline Vector3f getLatestGyro() {
        return _latestGyroMeasurements;
    }
    inline float getYaw() {
        return _currYaw;
    }
    //AP_Int8 _extNavPosEnabled;


//extern const AP_HAL::HAL& hal;

private:
    AP_HAL::UARTDriver *_port;                  // UART used to send data to external
    AP_SerialManager::SerialProtocol _protocol; // protocol used - detected using SerialManager's SERIAL#_PROTOCOL parameter

    uint8_t extNavCalled;
    uint32_t extNavTimer;
    Vector3f _extNavPos;
    Vector3f _extNavVel;
    Vector3f _latestGyroMeasurements;

    float _currYaw;

    AP_Int8 _extNavPosEnabled;
    AP_Int8 _extNavCtrlEnabled;
    AP_Int8 _extLowLevelCtrlEnabled;


};

#endif /* AC_EXT_NAV_H_ */
