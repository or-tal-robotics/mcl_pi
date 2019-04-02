#!/usr/bin/env python 

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
import numpy as np
from matplotlib import pyplot as plt

class MapClientLaserScanSubscriber(object):

    def __init__(self):
        self.static_map = rospy.ServiceProxy('static_map',GetMap)
        self.map = self.static_map()
        self.laser_scan_sub = rospy.Subscriber('/scan',LaserScan,self.get_scan)

        self.map_info = self.map.map.info
        self.map_data = self.map.map.data
        self.map_width = np.array(self.map_info.width) # map width
        self.map_heghit = np.array(self.map_info.height) # map heghit
        self.occupancy_grid = np.transpose(np.array(self.map_data).reshape(self.map_width, self.map_heghit)) # map
        self.map_origin = [self.map_info.origin.position.x, self.map_info.origin.position.y, self.map_info.origin.orientation.z] # map orientation in relation to the world [m,m,rad]
        self.resolution = self.map_info.resolution # map resolution

        self.matrix = np.zeros (shape = (self.map_width,self.map_heghit))

    def get_scan(self, msg):  # callback function for LaserScan topic
        self.z = msg

    def scan2cart(self, robot_origin = [0,0,0]):

        self.mu_x = self.map_origin[0] + robot_origin[0]; self.mu_y = self.map_origin[1] + robot_origin[1]; self.theta = self.map_origin[2] + robot_origin[2] # the robot orientation in relation to the world [m,m,rad] 
        ####################################################### need to add the robot localisation!!!! 

        # converting the laserscan measurements from polar coordinate to cartesian and create matrix from them.
        self.n = len(self.z.ranges); i = np.arange(len(self.z.ranges))
        self.angle = np.zeros(self.n); x = np.zeros(self.n); y = np.zeros(self.n)
    
        self.angle = np.add(self.z.angle_min, np.multiply(i, self.z.angle_increment))
        self.x = np.multiply(self.z.ranges, np.cos(self.angle + self.theta)) + self.mu_x
        self.y = np.multiply(self.z.ranges, np.sin(self.angle + self.theta)) + self.mu_y

        self.x = -np.rint (self.x / self.resolution) 
        #x_r = np.abs(x_r)
        self.x = self.x.astype (int)
        self.x[self.x > self.map_width] = -1
        self.x[self.x < -self.map_width] = -1
        self.y = -np.rint (self.y / self.resolution)
        #y_r = np.abs(y_r)
        self.y = self.y.astype (int)
        self.y[self.y < -self.map_heghit] = -1
        self.y[self.y > self.map_heghit] = -1

        self.Y = np.stack((self.x,self.y))

        return self.Y 
        
    def loction_based(self):
        self.scan2cart()
        # the round values of x,y coordinate from the laserscan are the indexes of the matrix which contains 0 by default and 100 for detecting an object
        self.matrix = np.zeros ((self.map_width,self.map_heghit))
        for ii in range (len(self.x)):
            if self.Y[0,ii] != -1 and self.Y[1,ii] != -1:
                self.matrix[self.Y[0,ii],self.Y[1,ii]] = 100
        return self.matrix

if __name__ == "__main__":
    rospy.init_node('laser_scan', anonymous = True)
    StaticMap = rospy.ServiceProxy('static_map',GetMap)

    scan = MapClientLaserScanSubscriber()
    r_first = rospy.Rate(10)
    r = rospy.Rate(240)
    r_first.sleep()
    OC = scan.occupancy_grid

    plt.ion()
    fig = plt.figure()
    while not rospy.is_shutdown():
        M = scan.loction_based()

        r.sleep()
        plt.imshow(-M+OC)
        fig.canvas.draw()
        print('Drawing!')

    rospy.spin()
    pass