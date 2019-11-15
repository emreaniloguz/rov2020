import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
#  If using a companion computer
#  the default connection is available
#  at ip 192.168.2.1 and the port 14550
# Note: The connection is done with 'udpin' and not 'udpout'.
#  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
#  uses a 'udpbcast' (client) and not 'udpin' (server).
#  If you want to use QGroundControl in parallel with your python script,
#  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
#  E.g: --out udpbcast:192.168.2.255:yourport
master = mavutil.mavlink_connection('udpin:0.0.0.0:15000')
master.wait_heartbeat()
master.arducopter_arm()

# Get some information !
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
        #print("yaw: ", yaw)

        if(time.time()-start>0.2):
            print(time.time()-start)
            start = time.time()
            master.mav.manual_control_send(
                        master.target_system,
                        int(yaw_diff)*10,
                        0,
                        500,
                        0,
                        0)

        #print("debug")

        #print("pitch: ", pitch*60)
    except Exception as msg:
        #print(msg)      # print exception msg
        pass
