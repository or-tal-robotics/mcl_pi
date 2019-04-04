#!/usr/bin/env python 

import rospy
import numpy as np
from geometry_msgs.msg import  PoseStamped, Point, Pose, PointStamped, PoseArray
import tf_conversions

class Reference(object):

    def __init__(self):
        self.p = rospy.Publisher('/reference_pose', PointStamped, queue_size = 6)
        rospy.Subscriber('/vrpn_client_node/robot_name/pose', PoseStamped, self.get_reference)
        rospy.wait_for_message('/vrpn_client_node/robot_name/pose', PoseStamped)
        

    def get_reference(self, msg):
        self.reference = np.empty(2)
        reference_point = msg
        
        self.reference[0] = reference_point.pose.position.x
        self.reference[1] = reference_point.pose.position.y

        point = PointStamped()
        point.header.frame_id = "map"
        point.point.x = self.reference[0]
        point.point.y = self.reference[1]
        point.point.z = 0
        self.p.publish(point)

class Particles(object):

    def __init__(self):
        rospy.Subscriber('/particlecloud', PoseArray, self.particlecloud)
        rospy.wait_for_message('/particlecloud', PoseArray)

    def particlecloud(self,msg): # callback function from topic /particlecloud
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

def main():
    r = Reference()
    p = Particles()
    
    while not rospy.is_shutdown():

        reference_point = r.reference
        E = np.mean(p.particles, axis=0)

        Error = np.linalg.norm(np.abs(E[0:2]-reference_point))

        E_pub = rospy.Publisher('/Expection', PointStamped, queue_size = 6)
        point = PointStamped()
        point.header.frame_id = "map"
        point.point.x = E[0]
        point.point.y = E[1]
        point.point.z = 0
        E_pub.publish(point)

if __name__ == "__main__":
    rospy.init_node('mse_calc')
    main()

    rospy.spin()