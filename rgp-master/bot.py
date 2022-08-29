import rospy as rp
import jupyros as jr
import numpy as np

from geometry_msgs.msg import Pose, Twist
import math
from sensor_msgs.msg import Image,Illuminance,LaserScan

from functools import partial 

import time

class PID:
    def __init__(self,Kp,Ki,Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.lastError = 0
        self.I = 0
        
    def value(self,e):
        P = e
        self.I = self.I + e
        D = e - self.lastError
        
        self.lastError = e
        
        return P*self.Kp+self.I*self.Ki+D*self.Kd

track_treshold = 80

def drive(light_data):    
    if min(light_data) > track_treshold:
        speed = -0.1
        turn = 0
    else:
        error = (8-np.argmin(light_data))
        turn =lfPID.value(error)
        
        speed = max(0.3-abs(turn)*0.15,0)
        
    return speed,turn

d = True

def handle_scan(ranges,ls,a):
    global d

    left = ranges[60]
    right = ranges[-60]

    val = np.min(ranges[:30]+ranges[-30:])

    if a != 1 and val < 0.35:
        if left < right:
            if a == 0:
                d=False
            return 1,0.0,0.0
        else:
            if a == 0:
                d=True
            return 1,0.0,0.0
    elif a == 1:
        val = np.min(ranges[:30]+ranges[-30:])

        if val < 0.35:
            return a,-0.1,0.0
        else:
            return a+1,0,0
    elif a == 2:
        mi = np.argmin(ranges)

        if mi > 80 and mi < 100 or mi > 260 and mi < 280:
            return a+1,0.2,0
        elif not d:
            return a,0.0,-0.5
        else:
            return a,0.0,0.5
    elif a == 3:
        if np.min(ls) < 50:
            return a,0.2,0
        else:
            return a+1,0,0
    elif a == 4:
        #return a,0,0
        if np.min(ls) < 30:
            return a+1,0,0

        idx = 90 if not d else 270

        r = np.array([v if v < 1 else 10 for v in ranges[idx:]+ranges[:idx]])

        error =  (180-np.argmin(r-0.2))+(np.min(r)-0.2)*1500*(-1 if d else 1)
        turn = oPID.value(error)

        if turn > 1:
            turn = 1
        elif turn < -1:
            turn = -1

        speed = max(0.3-abs(turn)*1.2,0.1)

        return a,speed,turn
    elif a == 5:
        mi = np.argmin(ranges)

        if mi > 160 and mi < 200 or min(ls) > 50:
            return 0,0,0
        elif not d:
            return a,0.05,-0.5
        else:
            return a,0.05,0.5

    return 0,0,0

def getTwist(speed,turn):
    twist = Twist()
    twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
   
    return twist

lfPID = PID(0.35,1e-10,-0.2)
oPID = PID(0.65e-2,1e-5,1e-4)

def main():
    global light_sensors
    global lidar_distances

    rp.init_node('runner')
    
    stop = False

    n_sensors = 16

    pub = rp.Publisher('/cmd_vel', Twist,queue_size=10)

    light_sensors  = [0]*n_sensors

    def setLightSensor(x,i):
        global light_sensors
        light_sensors[i] = x.illuminance
    
    lidar_distances = None

    def setLidar(x):
        global lidar_distances
        lidar_distances = x.ranges
    
    cams = [rp.Subscriber("/camera_{}/rgb/image_raw".format(i),Image) for i in range(n_sensors)]
    subs = [rp.Subscriber("/light_sensor_plugin/lightSensor/camera_{}".format(i),Illuminance,partial(lambda x,k:setLightSensor(x,k),k=i)) for i in range(n_sensors)]
    scans = rp.Subscriber("/scan",LaserScan,setLidar)

    while lidar_distances == None:
        print("\r","waiting for sensor data",end="")
        time.sleep(0.1)

    mode = 0

    print("starting")

    while not stop:
        mode,control_speed,control_turn = handle_scan(lidar_distances,light_sensors,mode)

        if mode == 0:
            (control_speed,control_turn) = drive(light_sensors)


        pub.publish(getTwist(control_speed,control_turn))

        time.sleep(0.1)

    pub.publish(getTwist(0,0))

    print("unregistering")
    for cam in cams:
        cam.unregister()

    for sub in subs:
        sub.unregister()
    pub.unregister()
    scans.unregister()
    print("done")

if __name__ == "__main__":
    main()
