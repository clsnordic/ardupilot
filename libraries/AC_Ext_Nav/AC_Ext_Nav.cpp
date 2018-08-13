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
    if (AP_HAL::millis() - _msLastPosRec > 40) _hasReceivedPos = 0;
    if (AP_HAL::millis() - _msLastCtrlRec > 40) _hasReceivedCtrl = 0;
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
               case MAVLINK_MSG_ID_HEARTBEAT:
               {
                   hal.console->printf("Looking for heartbeat...");
                   mavlink_message_t msg2;
                   mavlink_msg_heartbeat_pack(1,
                                              200,
                                              &msg2,
                                              MAV_TYPE_OCTOROTOR,
                                              MAV_AUTOPILOT_GENERIC,
                                              0,
                                              0,
                                              0);
                   uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
                   uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg2);
                   _port->write(buffer,len);

                   break;
               }

              case MAVLINK_MSG_ID_EXT_NAV_POSVELATT:
              {
                 mavlink_ext_nav_posvelatt_t packet;
                 mavlink_msg_ext_nav_posvelatt_decode(&msg, &packet);

                 _currYaw = packet.Yaw;
                 _extNavPos.x = packet.xPos;
                 _extNavPos.y = packet.yPos;
                 _extNavPos.z = packet.zPos;
                 _hasReceivedPos = TRUE;
                 _msLastPosRec = AP_HAL::millis();
                 break;
              }
              case MAVLINK_MSG_ID_EXT_NAV_CTRL:
              {
                  mavlink_ext_nav_ctrl_t packet;
                  mavlink_msg_ext_nav_ctrl_decode(&msg, &packet);
                  _hasReceivedCtrl = TRUE;
                  _msLastCtrlRec = AP_HAL::millis();
                  break;
              }
              default:
                  hal.console->printf("\nMessage not supported\n");
                  hal.console->printf("Got message: %u", msg.msgid);
              }
           }

       }

       //hal.console->printf("should have set values\n");*/




    } else {
        //hal.console->printf("No serial data!\n");
        DataFlash_Class::instance()->Log_Write("EXTNAV", "TimeUS,F", "QI",
                                                     AP_HAL::micros64(),
                                                     0);
    }
    _extNavPos.x = 0.0;
   _extNavPos.y = 0.0;
   _extNavPos.z = 0.0;
    // hal.console->printf("\nExiting loop\n");
}

void AC_Ext_Nav::init(const AP_SerialManager &serial_manager) {

    hal.console->printf("\nDoing init!\n");
    if(serial_manager.find_serial(AP_SerialManager::SerialProtocol_Ext_Nav, 0))
    {
      _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Ext_Nav, 0);
      DataFlash_Class::instance()->Log_Write("EXTNAV", "TimeUS,F", "QI",
                                             AP_HAL::micros64(),
                                             1);

      hal.console->printf("\nFound serial\n");
    } else {
        DataFlash_Class::instance()->Log_Write("EXTNAV", "TimeUS,F", "QI",
                                                    AP_HAL::micros64(),
                                                    0);
        hal.console->printf("\nDidn't find serial\n");
    }

}
AC_Ext_Nav::~AC_Ext_Nav() {
    // TODO Auto-generated destructor stub
}

