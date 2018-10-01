# Import DroneKit-Python
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# Connect to the Vehicle.
print("Connecting to vehicle on: 127.0.0.1:5762")
vehicle = connect('127.0.0.1:14551', wait_ready=True)
#vehicle = connect('COM12', baud=57600, wait_ready=True)

#0b0000111111111000

#Tak Sandnes
lat4=58.853844
lon4=5.743075
alt4=20.0

lat5=58.853101
lon5=5.742735


lat3 = 58.813170
lon3 = 5.649660
alt3 = 10.00

lat2 = 58.814901
lon2 = 5.645911
alt2 = 52.00

lat1 = 58.815032
lon1 = 5.646454
alt1 = 50.00
msg = vehicle.message_factory.set_gps_global_origin_encode(
        0,
        lat1*1e7, lon1*1e7, alt2*1e3)    # param 5 ~ 7 latitude, longitude, altitude
vehicle.send_mavlink(msg)

time.sleep(0.5)
#msg = vehicle.message_factory.command_long_encode(
#        0, 0,    # target system, target component
#        mavutil.mavlink.MAV_CMD_DO_SET_HOME, #command
#        0,    #confirmation
#        1,    # param 1, (1=use current location, 0=use specified location)
#        0,    # param 2, unused
#        0,    # param 3,unused
#        0,    # param 4, unused
#        lat4, lon4, alt4)    # param 5 ~ 7 latitude, longitude, altitude
#vehicle.send_mavlink(msg)
#print("Landing")
#vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
time.sleep(60)
vehicle.close()

print("Completed")