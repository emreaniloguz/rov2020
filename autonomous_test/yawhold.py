import time
from pymavlink import mavutil
import matplotlib.pyplot as plt
import numpy as np

master = mavutil.mavlink_connection('udpin:0.0.0.0:15000')
master.wait_heartbeat()

master.arducopter_arm()

bool = False
start = time.time()

while True:
    try:
        msg = master.recv_match().to_dict()

        if bool == False:
            yaw = msg["yaw"] * 60
            yaw_final = yaw + 30
            bool = True
        yaw = msg["yaw"]*60
        yaw_diff = yaw_final - yaw
        if yaw_diff<0:
            thrust = 500
        else :
            thrust = -500
        print("yaw difference : ", yaw_diff)
        if round(thrust)!=0:
            master.mav.manual_control_send(
                master.target_system,
                0,
                0,
                500,
                thrust,
                0)
        else:
            master.mav.manual_control_send(
                master.target_system,
                0,
                0,
                500,
                0,
                0)
            break

    except Exception as msg:
        # print(msg)      # print exception msg
        pass
