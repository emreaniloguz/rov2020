# Import mavutil
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udpin:0.0.0.0:15000')
master.wait_heartbeat()

master.arducopter_arm()

master.mav.manual_control_send(
    master.target_system,
    0, # forward
    250, # lateral
    500, # up-down
    0, # yaw
    0)

time.sleep(5)

master.mav.manual_control_send(
    master.target_system,
    0, # forward
    -250, # lateral
    500, # up-down
    0, # yaw
    0)

time.sleep(5)

master.arducopter_disarm()
