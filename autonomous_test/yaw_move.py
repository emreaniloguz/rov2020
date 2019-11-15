import time
# Import mavutil
from pymavlink import mavutil

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
        print ("yaw_diff : ", yaw_diff)
        #print("yaw: ", yaw)

        if(yaw_diff <2.5 and yaw_diff>-2.5):
            yaw_diff = 0

        elif(yaw_diff<=20 and yaw_diff>=0):
            yaw_diff = 20

        elif(yaw_diff>=-20 and yaw_diff<0):
            yaw_diff = -20

        elif(yaw_diff>=60):
            yaw_diff = 60
        elif(yaw_diff<=-60):
            yaw_diff = -60

        if(time.time()-start>0.2):
            #print(time.time()-start)
            start = time.time()
            master.mav.manual_control_send(
                        master.target_system,
                        0,
                        0,
                        500,
                        int(yaw_diff)*10,
                        0)

    except Exception as msg:
        #print(msg)      # print exception msg
        pass
