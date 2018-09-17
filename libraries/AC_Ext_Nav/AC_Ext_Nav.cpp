/*
 * AC_Ext_Nav.cpp
 *
 *  Created on: 8 May 2018
 *      Author: Christoffer
 */

#include "AC_Ext_Nav.h"
extern const AP_HAL::HAL& hal;

#define CDTORAD 5729.577951308232f

const AP_Param::GroupInfo AC_Ext_Nav::var_info[] = {
        // @Param: POS
        // @DisplayName: Use external navigation
        // @Description: This will allow for external positioning
        // @Values: 0:Disable,1:Enable
        // @User: Advanced
        AP_GROUPINFO("POS",   0, AC_Ext_Nav, _extNavPosEnabled, 0),

        // @Param: CTRL
        // @DisplayName: Use external navigation
        // @Description: This will allow for external low level navigation
        // @Units: Boolean
        // @Values: 0:Disable,1:Enable
        // @User: Advanced
        AP_GROUPINFO("CTRL",   1, AC_Ext_Nav, _extNavCtrlEnabled, 0),

        // @Param: AID
        // @DisplayName: Use external navigation
        // @Description: This will allow for external low level navigation
        // @Units: Boolean
        // @Values: 0:Disable,1:Enable
        // @User: Advanced
        AP_GROUPINFO("AID",   2, AC_Ext_Nav, _aidingEnabled, 0),
        // @Param: DELAY
        // @DisplayName: External navigation delay
        // @Description: delay the external navigation message by X ms
        // @Units: uint32_t
        // @Values: in 25
        // @User: Advanced
        AP_GROUPINFO("DELAY",   3, AC_Ext_Nav, _extNavDelay, 25),

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        // @Param: AID
        // @DisplayName: Use external navigation
        // @Description: This will allow for external low level navigation
        // @Units: Boolean
        // @Values: 0:Disable,1:Enable
        // @User: Advanced
        AP_GROUPINFO("DROPOUT",   4, AC_Ext_Nav, _simDropout, 0),

#endif

        AP_GROUPEND
};


AC_Ext_Nav::AC_Ext_Nav() {
    // TODO Auto-generated constructor stub
    AP_Param::setup_object_defaults(this, var_info);

    //_s_instance = this;
    firstCall = 0;

}

void AC_Ext_Nav::update() {
    //Retrieve data if _port pointer is not null
    //hal.console->printf("Trying to read\n");
    if (_port != nullptr && _port->available() > 0)

    {

       uint16_t buff_len = _port->available();

       mavlink_message_t msg;
       mavlink_status_t status;

       for(uint16_t i =0; i<buff_len; i++)
       {
           const uint8_t c = (uint8_t)_port->read();

           if (mavlink_parse_char(0, c, &msg, &status))
           {
               handleMsg(&msg);
               //hal.console->printf("Triggered\n");
           }

       }

    }

    if (AP_HAL::millis() - _msLastPosRec > 1000 && setTimer > 1)
        {
        //TODOCLS Sett COMPASS_USE til 1?
        //If this triggers we've lost communication with the localization board. Force GPS use and hope nothing happens.
            _hasReceivedPos = false;
            AP_Int8 val;
            val = 0;
            AP::ahrs_navekf().setFusionModeGps(val);
            if (AP::ahrs().get_origin(_ret)) setTimer = 1;
            else setTimer = 0;
            float paramVal = 3;
             AP::ahrs_navekf().setHorizPosNoise(paramVal);
             AP::ahrs_navekf().setAltPosNoise(paramVal);
             //AP::ahrs_navekf().setCompassParam(paramVal);
             gcs().send_text(MAV_SEVERITY_ERROR, "Timeout on localization board message, using GPS!");
        }
    if (AP_HAL::millis() - _msLastCtrlRec > 40) _hasReceivedCtrl = false;
}

void AC_Ext_Nav::init(const AP_SerialManager &serial_manager) {

    hal.console->printf("\nDoing init!\n");
    if(serial_manager.find_serial(AP_SerialManager::SerialProtocol_Ext_Nav, 0))
    {
      _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Ext_Nav, 0);
      DataFlash_Class::instance()->Log_Write("EXTF", "TimeUS,F", "QI",
                                             AP_HAL::micros64(),
                                             1);

      hal.console->printf("\nFound serial\n");
    } else {
        DataFlash_Class::instance()->Log_Write("EXTF", "TimeUS,F", "QI",
                                                    AP_HAL::micros64(),
                                                    0);
        hal.console->printf("\nDidn't find serial\n");
    }
    if(_aidingEnabled == 1 && AP::ahrs_navekf().getFusionModeGps() == 0)
    {
        AP_Int8 val;
        val = 3;
        AP::ahrs_navekf().setFusionModeGps(val);


    }

}
AC_Ext_Nav::~AC_Ext_Nav() {
    // TODO Auto-generated destructor stub
    //delete _s_instance;
}


bool AC_Ext_Nav::verifyPV(const mavlink_ext_nav_posvelatt_t &packet)
{
    if(packet.Yaw == 0.00 || packet.time_usec == 0.00) return false;
    return true;
}
bool AC_Ext_Nav::verifyRA(const mavlink_ext_nav_ctrl_t &packet)
{
    if(packet.time_usec == 0.00) return false;
    return true;

}
void AC_Ext_Nav::handleMsg(mavlink_message_t *msg)
{
     switch(msg->msgid)
      {

     case MAVLINK_MSG_ID_HEARTBEAT:
    {
        //hal.console->printf("Looking for heartbeat...");
        break;
    }

      case MAVLINK_MSG_ID_EXT_NAV_POSVELATT:
      {

         mavlink_ext_nav_posvelatt_t packet;
         mavlink_msg_ext_nav_posvelatt_decode(msg, &packet);


         if(!verifyPV(packet)) break;
         _currYaw = packet.Yaw;
         _latestPosition.x = packet.yPos;
         _latestPosition.y = packet.xPos;
         _latestPosition.z = -packet.zPos; // NED
         _latestAngleMeasurement.x = packet.Roll/CDTORAD;
         _latestAngleMeasurement.y = packet.Pitch/CDTORAD;
         _latestAngleMeasurement.z = wrap_2PI(-packet.Yaw/CDTORAD); //Opposite
         _latestVelocity.x = packet.yVel;
         _latestVelocity.y = packet.xVel;
         _latestVelocity.z = -packet.zVel; //NED

         _hasReceivedPos = true;
         _msLastPosRec = AP_HAL::millis();

         if(AP::ahrs().get_origin(_ret) && setTimer == 0 && aidingEnabled())
             {
                 firstCall = AP_HAL::millis();
                 ++setTimer;
                 //TODOCLS Set the EKF parameter to a high value before sending the first attPosMocap msg so the EKF doesnt go crazy
                 float paramVal = 7;
                 AP::ahrs_navekf().setHorizPosNoise(paramVal);
                 AP::ahrs_navekf().setAltPosNoise(paramVal);
                 gcs().send_text(MAV_SEVERITY_INFO, "Got first EXPV message, increasing noise parameters");
             }
         if(AP_HAL::millis() - _lastAttPosMocap >= 50 && aidingEnabled() && AP::ahrs().get_origin(_ret) && AP_HAL::millis() - firstCall >= 200)
         {


             uint32_t delayedMs = _msLastPosRec - _extNavDelay;
             uint32_t packetTime = packet.time_usec;

             sendAttPosMsg(packetTime,delayedMs,_latestPosition, _latestVelocity,_latestAngleMeasurement);
             _lastAttPosMocap = AP_HAL::millis();

             if(setTimer == 1)
             {
                 if(AP::ahrs_navekf().getFusionModeGps() == 0)
                 {
                     AP_Int8 val;
                     val = 3;
                     AP::ahrs_navekf().setFusionModeGps(val);
                 }
                 float velVar, posVar, hgtVar, tasVar = 0;
                 Vector3f magVar;
                 Vector2f offset;
                  AP::ahrs_navekf().get_variances(velVar,posVar,hgtVar,magVar,tasVar,offset);
                  /*if (AP_HAL::millis() - extNavCalled >= 1000)
                  {
                      hal.console->printf("posVar: %f", posVar);
                      extNavCalled = AP_HAL::millis();
                  }*/

                  if(posVar < 0.1 && AP_HAL::millis() - firstCall >= 5500 && hgtVar < 0.1)
                  {
                      gcs().send_text(MAV_SEVERITY_INFO, "Certain of position");
                      float paramVal = 0.1;
                       AP::ahrs_navekf().setHorizPosNoise(paramVal);
                       AP::ahrs_navekf().setAltPosNoise(paramVal);
                       setTimer++;
                  }



             }
             if (setTimer == 2 && AP_HAL::millis() - firstCall >= 20000)
             {
                 float velVar, posVar, hgtVar, tasVar = 0;
                  Vector3f magVar;
                  Vector2f offset;
                   AP::ahrs_navekf().get_variances(velVar,posVar,hgtVar,magVar,tasVar,offset);
                 if(posVar < 0.05)

                 {
                     Location temp_loc;
                     //hal.console->printf("home is: %d and get_loc: %d", (int)AP::ahrs().home_is_set(), (int)AP::ahrs_navekf().get_location(temp_loc));
                     if (AP::ahrs_navekf().get_location(temp_loc) && !AP::ahrs().home_is_set()) {
                         AP::ahrs_navekf().set_home(temp_loc);
                         AP::ahrs_navekf().lock_home();
                         gcs().send_text(MAV_SEVERITY_INFO, "Setting home to current location (for failsafe)");
                         ++setTimer;

                     }

                 }
             }
         }

         DataFlash_Class::instance()->Log_Write_EXPV(AP_HAL::micros64(), _latestPosition, _latestVelocity, _latestAngleMeasurement, aidingEnabled());
         break;
      }
      case MAVLINK_MSG_ID_EXT_NAV_CTRL:
      {

          mavlink_ext_nav_ctrl_t packet;
          mavlink_msg_ext_nav_ctrl_decode(msg, &packet);

          if(!verifyRA(packet)) break;

          storeAngRates(packet, _latestGyroMeasurements);
          storeAccel(packet, _latestAccelerations);

          _hasReceivedCtrl = true;

          _msLastCtrlRec = AP_HAL::millis();

          DataFlash_Class::instance()->Log_Write_EXRA(AP_HAL::micros64(), _latestGyroMeasurements, _latestAccelerations, extNavCtrlEnabled());

          break;
      }
      default:
          break;

  }
}

void AC_Ext_Nav::sendAttPosMsg(uint32_t& loggedTime, uint32_t& timestamp_ms, Vector3f& pos, Vector3f &vel, Vector3f& ang)
{

    // correct offboard timestamp to be in local ms since boot
    Quaternion attitude = Quaternion();
    attitude.from_euler((float)ang.x,(float)ang.y,(float)ang.z);

    const Vector3f sensor_offset = {};
       const float posErr = 0.1; // parameter required?
       const float angErr = 0.1; // parameter required?
    const uint32_t reset_timestamp_ms = 0; // no data available
    AP::ahrs().writeExtNavData(sensor_offset,
                                   pos/100,
                                   vel/100,
                                   attitude,
                                   posErr,
                                   angErr,
                                   timestamp_ms,
                                   reset_timestamp_ms);



        DataFlash_Class::instance()->Log_Write("VISP", "TimeUS,RemTimeUS,PX,PY,PZ,Roll,Pitch,Yaw,VX,VY,VZ,Q1,Q2,Q3,Q4",
                                               "ssmmmrrrmmm----", "FF0000000000000", "QQfffffffffffff",
                                               (uint64_t)AP_HAL::micros64(),
                                               (uint64_t)loggedTime,
                                               (double)pos.x/100,
                                               (double)pos.y/100,
                                               (double)pos.z/100,
                                               (double)ang.x,
                                               (double)ang.y,
                                               (double)ang.z,
                                               (double)vel.x/100,
                                               (double)vel.y/100,
                                               (double)vel.z/100,
                                               (double)attitude.q1,
                                               (double)attitude.q2,
                                               (double)attitude.q3,
                                               (double)attitude.q4);

}
void AC_Ext_Nav::storeAngRates(mavlink_ext_nav_ctrl_t &packet, Vector3f &gyroMeas)
{
    //Rollrate and pitch rate are misleading here.
    //rollrate is the angular velocity about the Y axis in our system, whereas it's about the x axis on the onboard system, so we swap them here.
    //temporarily remove the conversion from centi-degrees to see if we dont need filtering.
    gyroMeas.x = packet.pitchrate;// / CDTORAD;
    gyroMeas.y = packet.rollrate; // CDTORAD;
    gyroMeas.z = -packet.yawrate; // CDTORAD;

}
void AC_Ext_Nav::storeAccel(mavlink_ext_nav_ctrl_t &packet, Vector3f &accel)
{
    accel.x = packet.xacc * 100;
    accel.y = packet.yacc * 100;
    accel.z = -packet.zacc * 100;

}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void AC_Ext_Nav::setExtCtrl(Vector3f &gyro, Vector3f& accel) {
    //Generate a mavlink message and 'force it' to the correct one
    //TODO get this to go over serial link later
    mavlink_message_t msg;

    mavlink_msg_ext_nav_ctrl_pack(255,
                              200,
                              &msg,
                              AP_HAL::micros64(),
                              RadiansToCentiDegrees(gyro.x),
                              RadiansToCentiDegrees(gyro.y),
                              RadiansToCentiDegrees(gyro.z)*-1,
                              accel.x*100,
                              accel.y*100,
                              accel.z*-100);
    if(_simDropout == 0) handleMsg(&msg);
}
void AC_Ext_Nav::setExtPosVelAtt(Vector3f &pos, Vector3f& vel, Vector3f& ang) {
    //Generate a mavlink message and 'force it' to the correct one
    //TODO get this to go over serial link later
    mavlink_message_t msg;


    mavlink_msg_ext_nav_posvelatt_pack(255,
                              200,
                              &msg,
                              AP_HAL::micros64(),
                              pos.y, // East Pos (cm)
                              pos.x, // North pos (cm)
                              pos.z, // Up pos (cm)
                              vel.y,                      // East velocity (cm/s)
                              vel.x,                      // North velocity (cm/s)
                              vel.z,                      // Up velocity (cm/s)
                              ang.x*100,                      // Roll (centi-degree)
                              ang.y*100,                      // Pitch (centi-degree)
                              -ang.z*100);                      // Yaw (centi-degree)
    if(_simDropout == 0)
    {
        handleMsg(&msg);

    }


}
#endif

