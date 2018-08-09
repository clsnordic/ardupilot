# Import DroneKit-Python
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# Connect to the Vehicle.
print("Connecting to vehicle on: 127.0.0.1:5762")
vehicle = connect('127.0.0.1:14550', wait_ready=True)



print "\nSet Vehicle.mode = GUIDED (currently: %s)" % vehicle.mode.name 
#time.sleep(30)
vehicle.mode = VehicleMode("GUIDED")
while not vehicle.mode.name=='GUIDED':  #Wait until mode has changed
    print " Waiting for mode change ..."
    time.sleep(1)


# Check that vehicle is armable
while not vehicle.is_armable:
    print " Waiting for vehicle to initialise..."
    time.sleep(1)
    # If required, you can provide additional information about initialisation
    # using `vehicle.gps_0.fix_type` and `vehicle.mode.name`.
    
print "\nSet Vehicle.armed=True (currently: %s)" % vehicle.armed 
vehicle.armed = True
while not vehicle.armed:
    print " Waiting for arming..."
    time.sleep(1)
print " Vehicle is armed: %s" % vehicle.armed 

print "Taking off!"
vehicle.simple_takeoff(10) # Take off to target altitude

while True:
    print " Altitude: ", vehicle.location.global_relative_frame.alt
    #Break and return from function just below target altitude.
    if vehicle.location.global_relative_frame.alt>=10*0.95:
        print "Reached target altitude"
        break
    time.sleep(1)


nPos = 5
ePos = 0
msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,       # time_boot_ms (not used)
    0, 0,    # target_system, target_component
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
    0b0000101111111000, # type_mask (only speeds enabled)
    nPos, ePos, -10, # x, y, z positions
    0, 0, 0, # x, y, z velocity in m/s
    0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
    0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
# send command to vehicle
vehicle.send_mavlink(msg)
time.sleep(15)
msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,       # time_boot_ms (not used)
    0, 0,    # target_system, target_component
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
    0b0000101111111000, # type_mask (only speeds enabled)
    nPos, ePos, -10, # x, y, z positions
    0, 0, 0, # x, y, z velocity in m/s
    0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
    1.5707, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
# send command to vehicle
vehicle.send_mavlink(msg)
#print("Landing")
#vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
vehicle.close()

print("Completed")