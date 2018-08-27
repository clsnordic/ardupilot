/*
 * AC_Ext_Nav.cpp
 *
 *  Created on: 8 May 2018
 *      Author: Christoffer
 */

#include "AC_Ext_Nav.h"
extern const AP_HAL::HAL& hal;


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

}

void AC_Ext_Nav::update() {
    //Retrieve data if _port pointer is not null


    //Set the hasReceived Ctrl og pos to 0 if it's been too long since we received a message. Now set it to 40ms (4 cycles)
    if (AP_HAL::millis() - _msLastPosRec > 40) _hasReceivedPos = false;
    if (AP_HAL::millis() - _msLastCtrlRec > 40) _hasReceivedCtrl = false;
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
               switch(msg.msgid)
              {

              case MAVLINK_MSG_ID_EXT_NAV_POSVELATT:
              {
                  hal.console->printf("GOT MESSAGE!\n");
                 mavlink_ext_nav_posvelatt_t packet;
                 mavlink_msg_ext_nav_posvelatt_decode(&msg, &packet);

                 if(!verifyPV(packet)) break;
                 _currYaw = packet.Yaw;
                 _extNavPos.x = packet.xPos;
                 _extNavPos.y = packet.yPos;
                 _extNavPos.z = packet.zPos;
                 _extNavAng.x = packet.Roll;
                 _extNavAng.y = packet.Pitch;
                 _extNavAng.z = packet.Yaw;
                 _extNavVel.x = packet.xVel;
                 _extNavVel.y = packet.yVel;
                 _extNavVel.z = packet.zVel;

                 _hasReceivedPos = true;
                 _msLastPosRec = AP_HAL::millis();

                 //Now log the data and the time received

                 DataFlash_Class::instance()->Log_Write_EXPV(AP_HAL::micros64(), _extNavPos, _extNavAng, _extNavVel, extNavPosEnabled());
                 break;
              }
              case MAVLINK_MSG_ID_EXT_NAV_CTRL:
              {
                  hal.console->printf("GOT MESSAGE!\n");
                  mavlink_ext_nav_ctrl_t packet;
                  mavlink_msg_ext_nav_ctrl_decode(&msg, &packet);
                  if(!verifyRA(packet)) break;
                  _extNavRate.x = packet.rollrate;
                  _extNavRate.y = packet.pitchrate;
                  _extNavRate.z = packet.yawrate;

                  _extNavAcc.x = packet.xacc;
                  _extNavAcc.y = packet.yacc;
                  _extNavAcc.z = packet.zacc;

                  _hasReceivedCtrl = true;


                  DataFlash_Class::instance()->Log_Write_EXRA(AP_HAL::micros64(), _extNavRate, _extNavAcc, extNavCtrlEnabled());

                  _msLastCtrlRec = AP_HAL::millis();
                  break;
              }
              default:
                  break;

              }
           }

       }


    }

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
