import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, Point, PoseArray, Point32, PolygonStamped
from sensor_msgs.msg import PointCloud
from laser_scan_get_map import MapClientLaserScanSubscriber 
from particle_filter import ParticleFilter
from particlesintersection import RobotFusion, particles2fuse
from sklearn.neighbors import NearestNeighbors as KNN
import tf_conversions

particles = np.empty((200,3))
recive_particles = 0

def main():
    global particles
    global recive_particles
    rospy.init_node('particle_filter', anonymous = True)
    PF_l = ParticleFilter()
    PI_t = np.random.randn(200,3)
    fusion = RobotFusion(PF_l.particles, PI_t)
    particles2fuse_cb = lambda x: particles2fuse(x,PF_l,fusion)
    rospy.Subscriber('/particlecloud2fuse_in', PoseArray, particles2fuse_cb)
    r = rospy.Rate(5)
    time_last= rospy.Time.now()
    
    while not rospy.is_shutdown():
        time_now = rospy.Time.now()
        r.sleep()
        fusion.PI_s = PF_l.particles
        fusion.robot_tracking()
        PF_l.pub()
        if time_now.to_sec() - time_last.to_sec() > 2:
            PF_l.pub2fuse()
            time_last = time_now

        if recive_particles:
            fusion.PI_t = particles
            PF_l.weights = fusion.update_weight()
            PF_l.resampling()
            recive_particles = 0
            
        fusion.send_transfered_particles()
        r.sleep()


    rospy.spin()


if __name__ == "__main__":
    main()
