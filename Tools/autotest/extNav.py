# Import DroneKit-Python
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# Connect to the Vehicle.
print("Connecting to vehicle on: 127.0.0.1:5762")
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

print "\nSet Vehicle.mode = GUIDED (currently: %s)" % vehicle.mode.name 
time.sleep(30)
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

for i in range(0,10):

	msg2 = vehicle.message_factory.ext_nav_posvelatt_encode(
                1,
                0.1                      : Position local East relative to the home point (cm) (float)
                0.2                      : Position local North relative to the home point (cm) (float)
                0.3                      : Position local Vertical relative to the home point (cm) (float)
                1.1                      : Velocity local East (cm/s) (float)
                1.2                      : Velocity local North (cm/s) (float)
                1.3                      : Velocity local Up (cm/s) (float)
                2.1                      : Roll (centi-degree) (float)
                2.2                     : Pitch (centi-degree) (float)
                2.3                       : Yaw (centi-degree) (float)
	    )

	vehicle.send_mavlink(msg)
	time.sleep(0.1)

#print("Landing")
#vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
vehicle.close()

print("Completed")