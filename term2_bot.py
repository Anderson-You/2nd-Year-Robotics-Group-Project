import numpy as np
from sensor_msgs.msg import LaserScan, Illuminance,Image
from geometry_msgs.msg import Pose,Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import rospy as rp
from functools import partial
import sys

additional = sys.argv[1:]
All = len(additional) == 0 

A = "A" in additional or All
B = "B" in additional or All
C = "C" in additional or All
D = "D" in additional or All
E = "E" in additional or All

rp.init_node('runner')
pub = rp.Publisher('/cmd_vel', Twist,queue_size=10)

n_light_sensors = 3

qs = [0]*3

def setQi(x,i):
    global qs
    qs[i] = x.illuminance
    
odom = None

def setOdom(o):
    global odom
    odom = o
    
sc = []

def setSc(x):
    global sc
    sc = x.ranges
    
scans = rp.Subscriber("/scan",LaserScan,setSc)
subs = [rp.Subscriber("/light_sensor_plugin/lightSensor/camera_{}".format(i),Illuminance,partial(lambda x,k:setQi(x,k),k=i)) for i in range(n_light_sensors)]
cams = [rp.Subscriber("/camera_{}/rgb/image_raw".format(i),Image) for i in range(n_light_sensors)]
os = rp.Subscriber("/odom",Odometry,setOdom)

from localisation import *
from planning import *

def drive(pub,v,odom):
    
    a =  np.arctan2(v[0],-v[1])
    rotate(pub,a,odom)
   
    d = np.sqrt((v**2).sum())
    moveStraightForTime(pub,0.1,1*d)

def navigateToGoal(pos,goal,illegal=[],distance=2):
    grid[:,:] = 0
    setGoal(goal[0],goal[1])
    updateGrid(pos[0],pos[1],odom,sc)
    
    while True:
        updateGrid(pos[0],pos[1],odom,sc)
        path = getRoute(illegal,distance)
        
        if path == None:
            grid[:,:] = 0
            setGoal(goal[0],goal[1])
            continue

        if len(path) < 3:
            break        

        closest = path[-6:]

        a = closest[0]
        b = closest[-1]
        v = -np.array((b[1]-a[1],b[0]-a[0]))*fidelty/10

        drive(pub,v,odom)

        pos[0] += v[0]/fidelty
        pos[1] += v[1]/fidelty
        
    return pos

def driveInBox(th,v):
    while True:
        d = abs(np.mean(sc[:10]+sc[-10:])-th)

        if d < 0.1:
            break

        t = 1

        if d > 0.5:
            t = 4

        left = np.mean([min(v,1) for v in sc[80:100]])
        right = np.mean([min(v,1) for v in sc[-100:-80]])

        if left < 0.2:
            rotate(pub,np.pi/4+(0.2*v),odom)
        elif right < 0.2:
            rotate(pub,np.pi/4-(0.2*v),odom)
        else:
            rotate(pub,np.pi/4,odom)

        moveStraightForTime(pub,0.05*v,t)    

    rotate(pub,np.pi/4,odom)
    
def mask_in(xf,xt,yf,yt):
    xf = int(xf*fidelty)
    xt = int(xt*fidelty)
    yf = int(yf*fidelty)
    yt = int(yt*fidelty)
    
    n_x = (xt-xf)+1
    n_y = (yt-yf)+1
    x = np.linspace(xf,xt,n_x).astype(np.int)
    y = np.linspace(yf,yt,n_y).astype(np.int)
    
    X,Y = np.meshgrid(x,y)
    
    return list(zip(Y.ravel(),X.ravel()))
                
barrier = mask_in(2,4,2.6,2.75)
mask_right = mask_in(3.5,5.9,2,5.9)
mask_left = mask_in(0,2.5,2.5,5.9)   
                
######################################### 

## Task 1
if A:
    print("Starting task 1")

    rotate(pub,0,odom)
    init_localisation()
    updateState(qs)
    rotate(pub,0,odom)

    while not location_found():
        localisation_step(pub,qs,odom)
        rotate(pub,0,odom)

    location = get_location()
    pos = [3.1,2.8+location/10.0]

    print("I'm in square",location)
else:
    pos = [3.1,2.8+7/10.0]
## Task 2
if B:    
    print("Starting task 2")
    illegal = barrier+mask_right

    rotate(pub,0,odom)
    navigateToGoal(pos,(2.2,1.35),illegal)
    rotate(pub,np.pi/4,odom)

    # drive into box
if C:
    print("Driving into box")
    rotate(pub,np.pi/4,odom)
    driveInBox(0.2,1)
    print("Color in box",qs)

## Task 3
if D:
    print("Starting task 3")   
    rotate(pub,np.pi/4,odom)
    driveInBox(1.6,-1)


    # planning 
if E:
    print("Outside box")
    pos = [1.8,1.85]
    illegal = mask_left + barrier

    rotate(pub,np.pi/4,odom)
    pos = navigateToGoal(pos,(3,5.2),illegal,3)
    rotate(pub,0,odom)

print("finished")

os.unregister()
scans.unregister()
for sub in subs:
    sub.unregister()

for cam in cams:
    cam.unregister()