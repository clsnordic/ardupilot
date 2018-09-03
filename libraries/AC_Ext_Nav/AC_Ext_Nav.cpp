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
    //hal.console->printf("Handling the message!");

    if (AP_HAL::millis() - _msLastPosRec > 40)
        {
        //hal.console->printf("Triggered lastPosRec");
            _hasReceivedPos = false;
        }
    if (AP_HAL::millis() - _msLastCtrlRec > 40) _hasReceivedCtrl = false;

     switch(msg->msgid)
      {

     case MAVLINK_MSG_ID_HEARTBEAT:
    {
        hal.console->printf("Looking for heartbeat...");
        break;
    }

      case MAVLINK_MSG_ID_EXT_NAV_POSVELATT:
      {

         mavlink_ext_nav_posvelatt_t packet;
         mavlink_msg_ext_nav_posvelatt_decode(msg, &packet);

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

          mavlink_ext_nav_ctrl_t packet;
          mavlink_msg_ext_nav_ctrl_decode(msg, &packet);
          if(!verifyRA(packet)) break;
          _latestGyroMeasurements.x = packet.rollrate;
          _latestGyroMeasurements.y = packet.pitchrate;
          _latestGyroMeasurements.z = packet.yawrate;



          _extNavAcc.x = packet.xacc;
          _extNavAcc.y = packet.yacc;
          _extNavAcc.z = packet.zacc;

          _hasReceivedCtrl = true;

          _msLastCtrlRec = AP_HAL::millis();
          //hal.console->printf("time: %lu\n",AP_HAL::micros64());
          DataFlash_Class::instance()->Log_Write_EXRA(AP_HAL::micros64(), _latestGyroMeasurements, _extNavAcc, extNavCtrlEnabled());
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

/*static AC_Ext_Nav &AC_Ext_Nav::get_instance()
{
    return _s_instance;
} */
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void AC_Ext_Nav::setSitlGyro(Vector3f &gyro) {
    //Generate a mavlink message and 'force it' to the correct one
    //TODO get this to go over serial link later
    mavlink_message_t msg;

    mavlink_msg_ext_nav_ctrl_pack(255,
                              200,
                              &msg,
                              AP_HAL::micros64(),
                              gyro.x,
                              gyro.y,
                              gyro.z,
                              2,
                              2,
                              2);
   handleMsg(&msg);
}
#endif

