#!/usr/bin/env python 

import rospy
import numpy as np
from geometry_msgs.msg import  PoseStamped, Point, Pose, PointStamped, PoseArray
import tf_conversions
import pandas as pd

save_arg = []

class Particles(object):

    def __init__(self):
        self.p = rospy.Publisher('/reference_pose', PointStamped, queue_size = 6)
        rospy.Subscriber('/particlecloud', PoseArray, self.particlecloud)
        rospy.Subscriber('/vrpn_client_node/robot_name/pose', PoseStamped, self.get_reference)
        rospy.wait_for_message('/vrpn_client_node/robot_name/pose', PoseStamped)
        rospy.wait_for_message('/particlecloud', PoseArray)
        

    def get_reference(self, msg): # callback function from topic /vrpn_client_node/robot_name/pose
        self.reference = np.empty(2)
        reference_point = msg
        
        self.reference[0] = -reference_point.pose.position.y- 0.5
        self.reference[1] = reference_point.pose.position.x-0.2

    def particlecloud(self,msg): # callback function from topic /particlecloud
        global save_arg

        self.particles = np.empty((len(msg.poses),3))
        for i in range(len(msg.poses)):
            self.particles[i,0] = msg.poses[i].position.x
            self.particles[i,1] = msg.poses[i].position.y
            angle = (
                msg.poses[i].orientation.x,
                msg.poses[i].orientation.y,
                msg.poses[i].orientation.z,
                msg.poses[i].orientation.w) 
            self.particles[i,2] = tf_conversions.transformations.euler_from_quaternion(angle)[2]

        reference_point = self.reference
        E = np.mean(self.particles, axis=0)
        self.Error = np.linalg.norm(np.abs(E[0:2]-reference_point))

        E_pub = rospy.Publisher('/Expection', PointStamped, queue_size = 6)
        point = PointStamped()
        point.header.frame_id = "map"
        point.point.x = E[0]
        point.point.y = E[1]
        point.point.z = 0
        E_pub.publish(point)

        point = PointStamped()
        point.header.frame_id = "map"
        point.point.x = self.reference[0]
        point.point.y = self.reference[1]
        point.point.z = 0
        self.p.publish(point)

        save_arg.append((self.reference,E,self.Error))


def text():
    global save_arg
    
    text_file_name = str(rospy.get_param('~text_file'))
    file_name = "/home/lab/Desktop/"+text_file_name+".csv"
    pd.DataFrame(save_arg).to_csv(file_name)

def main():
    p = Particles()

    rospy.on_shutdown(text)

if __name__ == "__main__":
    rospy.init_node('mse_calc')
    main()

    rospy.spin()