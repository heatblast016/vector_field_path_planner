import numpy as np
import rospy
import math
from geometry_msgs.msg import *

def sumForce(a, b):
    total = Vector3()
    total.x = a.x+b.x
    total.y = a.y+b.y
    total.z = a.z+b.z
    return total
#return forces given a vector3 describing robot position (in the reference frame of waypoint)

#return the "point force"
def pointForce(pos):
    position = Point()
    position.x = -1 * pos.x
    position.y = -1 * pos.y
    xcomp = position.x/math.hypot(position.x, position.y)
    ycomp = position.y/math.hypot(position.x, position.y)
    pf = Vector3()
    pf.x=xcomp
    pf.y=ycomp
    pf.z=0.0
    return pf

#return the "line force"
def lineForce(pos):
    position = Point()
    position.x = -1 * pos.x
    position.y = -1 * pos.y
    ycomp = 1.0 * math.copysign(min(1.0, abs(position.y)) * min(1.0, abs(position.x)), position.y) * math.copysign(1.0, position.x)
    lf = Vector3()
    lf.x = 0.0
    lf.y=ycomp
    lf.z = 0.0
    return lf

#return the "rotational force" component given a position
#TODO: determine whether to adjust offset as a constant in this file or as a parameter passed into the function

def rotationForce(pos):
    position = Point()
    position.x = -1 * pos.x
    position.y = -1 * pos.y
    offset = 1.7
    xcomp1 = -1.0 * (position.y-offset)/(position.x**2 + (position.y - offset)**2)
    xcomp2 = 1.0 * (position.y+offset)/(position.x**2 + (position.y + offset)**2)
    ycomp1 = 1.0 * position.x/(position.x**2 + (position.y - offset)**2)
    ycomp2 = -1.0 * position.x/(position.x**2 + (position.y + offset)**2)
    force1 = Vector3()
    force1.x = xcomp1
    force1.y = ycomp1
    force1.z = 0.0
    force2 = Vector3()
    force2.x = xcomp2
    force2.y = ycomp2
    force2.z = 0.0
    return sumForce(force1, force2)

#return net force at a point
def netForceForWaypoint(position):
    return sumForce(pointForce(position), sumForce(lineForce(position), rotationForce(position)))
def netForce(position):
    secondary = Point(x=position.x+2.0, y=position.y)
    return netForceForWaypoint(position)
