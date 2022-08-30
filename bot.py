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
        self.Kp = Kp # Initialise the proportional gain.
        self.Ki = Ki # Initialise the integral gain.
        self.Kd = Kd # Initialise the derivative gain.
        self.lastError = 0 # Set the previous error to zero.
        self.I = 0 # Initialise the integral to zero.
        
    def value(self,e):
        P = e
        self.I = self.I + e
        D = e - self.lastError
        
        self.lastError = e
        
        return P*self.Kp+self.I*self.Ki+D*self.Kd # Return the controller output.
        
track_treshold = 80 # The sensor reading.

def drive(light_data):    
    if min(light_data) > track_treshold:
        speed = -0.1
        turn = 0
    else:
        error = (8-np.argmin(light_data))
        turn = lfPID.value(error)
        
        speed = max(0.3-abs(turn)*0.15,0)
        
    return speed,turn

d = True

def handle_scan(ranges,ls,a):
    global d

    left = ranges[60] # The ranges where the left light sensor can read.
    right = ranges[-60] # The ranges where the right light sensor can read.

    val = np.min(ranges[:30]+ranges[-30:])  # The ranges where the light sensor overall can read.

    if a != 1 and val < 0.35:
        if left < right:
            if a == 0:
                d = False # Not update the desired angular position.
            return 1,0.0,0.0
        else:
            if a == 0:
                d = True # Update the desired angular position.
            return 1,0.0,0.0
    elif a == 1:
        val = np.min(ranges[:30]+ranges[-30:])
        # The robot will move backwards if it detects an obstacle in front and goes within a minimum distance of 0.35.
        if val < 0.35:
            return a,-0.1,0.0
        else:
            return a+1,0,0
    # At stage two the robot starts turning left in front of the obstacle.
    elif a == 2:
        mi = np.argmin(ranges)

        # The robot's turning angle is set to be either around 90 degrees or 270 degrees.
        if mi > 80 and mi < 100 or mi > 260 and mi < 280:
            return a+1,0.2,0
        elif not d:
            return a,0.0,-0.5
        else:
            return a,0.0,0.5
    # The robot is circumnavigating the obstacle at the following two stages.
    elif a == 3:
        if np.min(ls) < 50:
            return a,0.2,0
        else:
            return a+1,0,0
    elif a == 4:
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

        speed = max(0.3-abs(turn)*1.2,0.1) # Update the linear/angular velocity of the robot by firstly multiplying the turning angle and then subtracting this part from our minimum velocity.

        return a,speed,turn
   # At stage five the robot will move back onto the track.
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
    # x, y, z corresponds to roll, pitch and yaw axis.
    twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
   
    return twist

lfPID = PID(0.3,0,-0.08) # Kp = 0.3, Ki = 0, Kd = -0.08. 
oPID = PID(1e-2,1e-5,1e-4) # Kp = 1e-2, Ki = 1e-5, Kd = 1e-4.

def main():
    global light_sensors
    global lidar_distances # LIDAR sensor is used to compute the distance between the light sensor and the obstacle.
    rp.init_node('runner')
    
    stop = False

    n_sensors = 16 # The number of light sensors for this robot.

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

        time.sleep(0.1) # The delay of light sensors.

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
