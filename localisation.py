import numpy as np
import rospy as rp
from geometry_msgs.msg import Pose,Twist
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

reading = False
sensor_acc = 0.9  # The probability that the sensor will give the correct reading.
move_acc = 0.95   # The probability that the robot will move accurately.
speed = 0.05
turn_speed = np.pi/16
ts = 0.1/speed  # The twisting speed of the robot.

global states
global probabilities
global line

states = []
probabilities = []
line = []

def mod(n,base): 
    return n - int(n/base) * base

def init_localisation():
    global line
    global pos
    global probabilities
    global states 
    
    # The line variable corresponds to the grid of the map, which is indexed from 0 up to and including 24.
    line = np.array([False]*2+  # The black grid which corresponds to 2 units.
            [True]+  # The white grid which corresponds to 1 unit.
            [False]*3+
            [True]*2+
            [False]*2+
            [True]+
            [False]*3+
            [True]*2+
            [False]*3+
            [True]*2+
            [False]*2+
            [True])
    n = len(line)
    probabilities = np.array([1.0/n]*n)
    pos = n//2
    states = list(range(len(probabilities)))
    

def moveStraight(pub,s):
    twist = Twist()
    twist.linear.x = s; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
    
def moveStraightForTime(pub,s,t):
    moveStraight(pub,s)
    rp.sleep(t)
    moveStraight(pub,0)
    rp.sleep(0.5)  # The delay for the robot.

def sensorToBool(qs):
    return np.mean(qs) > 200  # The light sensor must be bright enough here.

# The following three methods works out the probability distribution.
def p_y(e):
    global probabilities
    global states
    
    s = 0
    for i in states:
        s+= p_y_x(e,i)*probabilities[i]
        
    return s

def p_y_x(y,x):
    inv = line[x] != y
    
    if inv:
        return 1-sensor_acc
    return sensor_acc

def p_x_y(x,y):
    global probabilities
    
    return (p_y_x(y,x) * probabilities[x])/p_y(y)

def updateState(qs):
    global reading
    global probabilities
    
    reading = sensorToBool(qs)
    
    probabilities = np.array(list(map(lambda x:p_x_y(x,reading),states)))

def moveLeft(pub,qs):
    global probabilities
    
    moveStraightForTime(pub,speed,ts)
    
    # There is no chance of the robot moving onto the rightmost square.
    for i in states[:-1]:  
        # The probability of the robot at position i is updated by the probability of the robot in the current location plus the probability of the robot in the next location.
        probabilities[i] = probabilities[i] * (1-move_acc) + probabilities[i+1] * move_acc
        
    probabilities[-1] = probabilities[-1] * (1-move_acc)
    updateState(qs)
    
def moveRight(pub,qs):
    global probabilities
    
    moveStraightForTime(pub,-speed,ts)
    
    # There is no chance of the robot moving onto the leftmost square.
    for i in states[1:]:  
        # The probability of the robot at position i is updated by the probability of the robot in the current location plus the probability of the robot in the previous location.
        probabilities[i] = probabilities[i] * (1-move_acc) + probabilities[i-1] * move_acc
        
    probabilities[0] = probabilities[0] * (1-move_acc)
    updateState(qs)
    
def locate(pub,qs):
    init_localisation()
    updateState(qs)

    maximum = np.max(probabilities)

    while maximum < 0.8:  # We won't stop updating the probability as long as it doesn't exceed 0.8.
        idx = np.where(probabilities >= maximum)[0]

        l_idx = [k-1 for k in idx if k > 0]
        t_left = line[l_idx].sum()
        f_left = len(l_idx) - t_left 

        r_idx = [k+1 for k in idx if k < len(probabilities)-1]
        t_right = line[r_idx].sum()
        f_right = len(r_idx) - t_right 

        left = np.argmin([k if k > 0 else np.inf for k in [t_left,f_left,t_right,f_right]]) < 2


        # risky = np.where(probabilities >= 0.25)[0]

        # if 0 in risky:
        #    left = False

        # if 9 in risky:
        #    left = True

        if left:
            moveLeft(pub,qs)
        else:
            moveRight(pub,qs)

        maximum = np.max(probabilities)
        
def location_found(t=0.8):
    return np.max(probabilities) >= t

def get_location():
    return np.argmax(probabilities)

offset = R.from_euler('z', -90, degrees=True)
def get_rotation (odom):
    orientation_q = odom.pose.pose.orientation # Get the robot's orientation by using Odom topic.
    r = R.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]) * offset
    
    return -r.as_euler(seq="xyz")[-1]

# The robot's rotation angle is bounded between -2pi and 2pi. 
def abs_rotate(pub,a,odom):
    angle = mod(a,2*np.pi)
    
    if abs(angle) > np.pi:
        angle = (-2*np.pi + angle*np.sign(angle))
    
    twist = Twist()
    # x, y, z correspond to roll, pitch and yaw axis.
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -turn_speed*np.sign(angle)
    pub.publish(twist)

    rp.sleep(abs(angle/turn_speed))

    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)

    rp.sleep(0.5)

def rotate(pub,a,odom):
    angle = mod((a - get_rotation(odom)),2*np.pi)
    
    if abs(angle) > np.pi:
        angle = (angle+2*np.pi*np.sign(-angle))
        
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -turn_speed*np.sign(angle)
    pub.publish(twist)

    rp.sleep(abs(angle/turn_speed))

    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)

    rp.sleep(0.5)

def localisation_step(pub,qs,odom):
    maximum = np.max(probabilities)
    idx = np.where(probabilities >= maximum)[0]

    l_idx = [k-1 for k in idx if k > 0]
    t_left = line[l_idx].sum()
    f_left = len(l_idx) - t_left 

    r_idx = [k+1 for k in idx if k < len(probabilities)-1]
    t_right = line[r_idx].sum()
    f_right = len(r_idx) - t_right 

    left = np.argmin([k if k > 0 else np.inf for k in [t_left,f_left,t_right,f_right]]) < 2
    
    # risky = np.where(probabilities >= 0.25)[0]
    
    # if 0 in risky:
    #    left = False

    # if 9 in risky:
    #    left = True

    
    if left:
        moveLeft(pub,qs)
    else:
        moveRight(pub,qs)
        
    rotate(pub,0,odom)
    
def plot_location():
    global states
    global probabilities
    
    f, ax = plt.subplots()
    ax.bar(states+[len(states)+1],list(probabilities)+[sum(probabilities)])   
    return f   
    
