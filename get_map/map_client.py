#!/usr/bin/env python 

import rospy
from nav_msgs.srv import GetMap
import numpy as np
import matplotlib.pyplot as plt

def map_client():
    rospy.wait_for_service('static_map')
    
    try:
        static_map = rospy.ServiceProxy('static_map',GetMap)
        map = static_map()
        map_width = np.array(map.map.info.width)
        map_heghit = np.array(map.map.info.height)
        OccupancyGrid = np.array(map.map.data).reshape(map_width, map_heghit)
        print map.map.info

        plt.imshow (np.transpose(OccupancyGrid))
        plt.show()
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    map_client()
    
    pass