#!/usr/bin/env python 

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from matplotlib import pyplot as plt

def scan2cart(Z_p,X):
    n = len(Z_p.ranges); i = np.arange(len(Z_p.ranges))
    angle = np.zeros(n); x = np.zeros(n); y = np.zeros(n)
    
    angle = np.add(Z_p.angle_min, np.multiply(i, Z_p.angle_increment))
    x = np.multiply(Z_p.ranges, np.cos(angle))
    y = np.multiply(Z_p.ranges, np.sin(angle))

    Z_c = np.column_stack ((x,y,angle))

    print np.column_stack ((x,y))

    #y[y > 800] = 0
    #x[x > 800] = 0
    #print np.amax(y)
    #print np.amax(x)
    #plt.plot (-y,x)
    #plt.show() 

def get_scan (Z_p):
    X = [0,0,0]
    y = scan2cart(Z_p,X)
    
def scan():
    rospy.init_node('laser_scan', anonymous = True)
    rospy.Subscriber('/scan',LaserScan,get_scan)

if __name__ == "__main__":
    scan()
    rospy.spin()
    pass