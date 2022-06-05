import rospy
import math
import numpy as np
import dubins
from geometry_msgs.msg import *
from tf.transformations import *
#generates a path to follow in the reference frame of the end waypoint given a starting point, a vector gen function, a max velocity, a duration to generate the path over, and optional dt (the intervals at which it calculates the path) 
def genpath(start, vectorgen, maxvel, horizon = 4.0, dt = 0.2):
    current = Point(x=start.x,y=start.y)
    path = PoseArray()
    radius = 1.0
    dubin = False
    for i in range(int(horizon/dt)):
        if(math.sqrt(current.x**2 + current.y**2)<radius):
            dubin = True
            break
        currentpose = Pose()
        currentpose.position = Point(x=current.x,y=current.y)
        forceatcurrent = vectorgen(current)
        currentpose.orientation = quaternion_from_euler(0,0, math.atan2(forceatcurrent.y,forceatcurrent.x))
        path.poses.append(currentpose)
     #   print(current.x)
    # print(current.y)
    #    print(forceatcurrent.x * maxvel*dt)
        #doesnt actually obey max velocity, need to cap somehow
        current.x = current.x  + (forceatcurrent.x * maxvel * dt)
        current.y = current.y  + (forceatcurrent.y * maxvel * dt)
    if(dubin):
        lastpose = path.poses[len(path.poses)-1]
        (lastposeroll, lastposepitch,lastposeyaw) = euler_from_quaternion(lastpose.orientation)
        print(lastposeyaw)
        dubinpath = dubins.shortest_path((lastpose.position.x, lastpose.position.y, lastposeyaw), (0,0,0),0.5)
        configs, _ = dubinpath.sample_many(0.01)
        for pos in configs:
            currentpose = Pose()
            currentpose.position = Point(x=pos[0],y=pos[1])
            currentpose.orientation = quaternion_from_euler(0,0, pos[2])
            path.poses.append(currentpose)
    return path

