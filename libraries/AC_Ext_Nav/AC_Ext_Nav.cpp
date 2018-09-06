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

        AP_GROUPEND
};


AC_Ext_Nav::AC_Ext_Nav() {
    // TODO Auto-generated constructor stub
    AP_Param::setup_object_defaults(this, var_info);

    //_s_instance = this;

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
    if (AP_HAL::millis() - _msLastPosRec > 40) _hasReceivedPos = false;
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
         _latestPosition.z = packet.zPos;
         _latestAngleMeasurement.x = packet.Roll/CDTORAD;
         _latestAngleMeasurement.y = packet.Pitch/CDTORAD;
         _latestAngleMeasurement.z = packet.Yaw/CDTORAD;
         _latestVelocity.x = packet.yVel;
         _latestVelocity.y = packet.xVel;
         _latestVelocity.z = packet.zVel;

         _hasReceivedPos = true;
         _msLastPosRec = AP_HAL::millis();

         //Now log the data and the time received

         DataFlash_Class::instance()->Log_Write_EXPV(AP_HAL::micros64(), _latestPosition, _latestAngleMeasurement, _latestVelocity, extNavPosEnabled());
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
          //hal.console->printf("time: %lu\n",AP_HAL::micros64());
          DataFlash_Class::instance()->Log_Write_EXRA(AP_HAL::micros64(), _latestGyroMeasurements, _latestAccelerations, extNavCtrlEnabled());
          /*if(extNavCalled==200)
          {
          hal.console->printf("extNavCtrlEnabled(): %d,_hasReceivedCtrl: %d,_extNavCtrlEnabled: %d\n",(int)extNavCtrlEnabled(),(int)_hasReceivedCtrl,(int)_extNavCtrlEnabled);
          extNavCalled = 0;
          } else ++extNavCalled; */
          break;
      }
      default:
          break;

  }
}
void AC_Ext_Nav::storeAngRates(mavlink_ext_nav_ctrl_t &packet, Vector3f &gyroMeas)
{
    gyroMeas.x = packet.rollrate / CDTORAD;
    gyroMeas.y = packet.pitchrate / CDTORAD;
    gyroMeas.z = -packet.yawrate / CDTORAD;

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
   handleMsg(&msg);
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
                              ang.z*100);                      // Yaw (centi-degree)
   handleMsg(&msg);
}
#endif

