# Import DroneKit-Python
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import datetime
import math


# Connect to the Vehicle.
print("Connecting to vehicle on: 127.0.0.1:5762")
vehicle = connect('127.0.0.1:14550', wait_ready=True, rate=100)

currentDT = datetime.datetime.now()
print (str(currentDT))
lastCall = datetime.datetime.now()
@vehicle.on_message('RAW_IMU')
def sendExtCtrl(self, name, data):
    global lastCall
    #print(data.pitchspeed)
    sendMessage = vehicle.message_factory.ext_nav_ctrl_encode(
        1,
        data.xgyro,  # : angular velocity about x axis
        data.ygyro,  # : angular velocity about y axis
        data.zgyro,  # : angular velocity about z axis
        1.1,  # : linear acceleration in x axis
        1.2,  # : linear acceleration in y axis
        1.3  # : linear acceleration in z axis 
    )
    vehicle.send_mavlink(sendMessage)
 #   print("Time since last call: ")
 #   print(datetime.datetime.now() - lastCall)
 #   lastCall = datetime.datetime.now()
#def rawImuData(self, name, msg):
#    sendExtCtrl(msg)
    

try:
    while True:
        pass
except KeyboardInterrupt:
    pass
#    msg2 = vehicle.message_factory.ext_nav_posvelatt_encode(
#                1,
#                i,                     # : Position local East relative to the home point (cm) (float)
#                0.2,                     # : Position local North relative to the home point (cm) (float)
#                0.3,                     # : Position local Vertical relative to the home point (cm) (float)
#                1.1,                     # : Velocity local East (cm/s) (float)
#                1.2,                     # : Velocity local North (cm/s) (float)
#                1.3,                     # : Velocity local Up (cm/s) (float)
#                2.1,                     # : Roll (centi-degree) (float)
#                2.2,                     #: Pitch (centi-degree) (float)
#                2.3                     #  : Yaw (centi-degree) (float)
#    )
    
    #vehicle.send_mavlink(msg)
    #vehicle.send_mavlink(msg2)
    #time.sleep(0.009)
    #currentDT = datetime.datetime.now()
    #print (str(currentDT))
 
    
currentDT = datetime.datetime.now()
print (str(currentDT))
#print("Landing")
#vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
vehicle.close()

print("Completed")