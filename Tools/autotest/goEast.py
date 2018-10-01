# Import DroneKit-Python
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math

# Connect to the Vehicle.
print("Connecting to vehicle on: 127.0.0.1:5762")
vehicle = connect('127.0.0.1:14551', wait_ready=True)

#0b0000111111111000
#print "\nSet Vehicle.armed=True (currently: %s)" % vehicle.armed 
#vehicle.armed = True
#vehicle.simple_takeoff(10) # Take off to target altitude
#time.sleep(100)
# speed yawrate 0b0000011111000111

#(math.cos(2*pi/n*x)*r,math.sin(2*pi/n*x)*r) for x in xrange(0,n+1)
x=100
for i in range(0,x):
    if 50 <= i < 75:
        fast = 0b0000011111111000
        
    else:
        fast = 0b0000111111111000
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        fast, # type_mask (only speeds enabled)
        0, i, -10, # x, y, z positions
        #0, 0, -10,  # x, y, z positions
        0, 1, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        (3*3.14/2), 0.2)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    loc = vehicle.location.local_frame
    dNorth = loc.north - 0
    dEast = loc.east - i
    while math.sqrt(math.pow(dNorth,2) + math.pow(dEast,2)) > 1:
        loc = vehicle.location.local_frame
        dNorth = loc.north - 0
        dEast = loc.east - i
        time.sleep(0.1)
    print vehicle.location.local_frame
   # print("message send")
    #time.sleep(2)

#print("Landing")
#vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
vehicle.close()

print("Completed")