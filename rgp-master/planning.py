import numpy as np
from scipy.spatial.transform import Rotation as R
fidelty = 10
goal = None
pos = None
grid = np.ones((6*fidelty,6*fidelty))

x_max = 6*fidelty-1
y_max = 6*fidelty-1

g_values = np.ndarray((6*fidelty,6*fidelty))
f_values = np.ndarray((6*fidelty,6*fidelty))
closed_list = np.ndarray((6*fidelty,6*fidelty),dtype=np.bool)
parents = np.ndarray((6*fidelty,6*fidelty),dtype=object)

def dist(x,y,a,b):
    return np.sqrt((x-a)**2+(b-y)**2)

def get_rotation (odom):
    orientation_q = odom.pose.pose.orientation
    r = R.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    return r.as_euler(seq="xyz",degrees=True)[-1]

def setGoal(x,y):
    global goal
    
    if goal is not None:
        grid[goal[0],goal[1]] = 1
    
    goal = np.array([y*fidelty,x*fidelty]).astype(np.int)
    grid[goal[0],goal[1]] = -10

def updateGrid(x,y,odom,scans):
    global grid
    global pos 
    global goal
        
    if pos is not None:
        grid[pos[0],pos[1]] = 1
        
    pos = np.array([y*fidelty,x*fidelty]).astype(np.int)
    grid[pos[0],pos[1]] = 0

    offset = int(get_rotation(odom))-90

    for alpha in range(len(scans)):    
        c = scans[alpha]

        rad = np.deg2rad((alpha+offset)%360)

        if np.abs(c) == np.inf:
            continue

        a = (c * np.sin(rad))*fidelty
        b = (c * np.cos(rad))*fidelty

        idx = np.round(pos-np.array([b,a])).astype(np.int)

        grid[idx[0],idx[1]] = 10
        
def heuristic(p):
    return dist(*p,*goal)

def getChildren(p):
    x,y = p
    if x == 0 or y == 0 or x == x_max or y == y_max:
        return [n for n in [
            (x+1,y),
            (x,y+1),
            (x-1,y),
            (x,y-1),
            (x+1,y+1),
            (x-1,y-1),
            (x-1,y+1),
            (x+1,y-1),
        ] if n[0] >= 0 and n[0] < x_max and n[1] >= 0 and n[1] < y_max]

    else:
        return [
             (x+1,y),
            (x,y+1),
            (x-1,y),
            (x,y-1),
            (x+1,y+1),
            (x-1,y-1),
            (x-1,y+1),
            (x+1,y-1)
        ]
    
import matplotlib.pyplot as plt
    
def getPath(start,end):
    global goal
    global parents
    
    path = []
    c = end

    while c != start:
        p = parents[c]
        
        if p == None:
            print("no path found")
            return None
            
        
        path.append(p)
        c = p
        
    return path+[start]
        
def getRoute(fake_obstacles=[],distance=2):
    global grid
    global pos
    global goal
    global g_values
    global f_values
    global parents
    global closed_list
    
    start = (pos[0],pos[1])
    goal = (goal[0],goal[1])
    (xDim,yDim) = np.where(grid==10)
    obstacles = np.concatenate([np.column_stack([xDim,yDim]),fake_obstacles])
    
    too_close = []

    for i in range(distance):
        for n in obstacles:
            too_close += getChildren(n)

        obstacles = np.unique(np.concatenate([obstacles,too_close]),axis=0)
        
    g_values[:,:] = np.inf
    f_values[:,:] = np.inf
    parents[:,:] = None
    closed_list[:,:] = False

    g_values[start] = 0
    f_values[start] = heuristic(start)
    closed_list[obstacles[:,0],obstacles[:,1]] = True
    
    closed_list[start] = False
    closed_list[goal] = False
    open_list = [start]
    
    while len(open_list) > 0:
        i = np.argmin([f_values[x] for x in open_list])
        current = open_list[i]

        del open_list[i]
        closed_list[current] = True

        if current == goal:
            break

        children = getChildren(current)

        g_parent = g_values[current]

        for c in children:
            if not closed_list[c]:
                g = g_parent + dist(*c,*current)
                if g < g_values[c]:
                    parents[c] = current
                    g_values[c] = g
                    f_values[c] = g + heuristic(c)

                    if c not in open_list:
                        open_list.append(c)

    return getPath(start,goal)#,open_list,closed_list