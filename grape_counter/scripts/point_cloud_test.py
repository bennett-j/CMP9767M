#!/usr/bin/env python

# Python libs
import sys, time

# OpenCV
import cv2

# Ros libraries
import roslib, rospy, image_geometry, tf

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import random as rng

class PointCloudTest:
    
    def __init__(self):    

        
        self.pc2_pub = rospy.Publisher('/thorvald_001/point_cloud_test', PointCloud2, queue_size=10)

        self.rate = rospy.Rate(1)  # hz

    def run(self):
        # keep re-iterating until the node is terminated,
        # either by pressing Ctrl-C or rosnode kill command
        while not rospy.is_shutdown():
            

            points = []
            
            for i in range(100):
                x = rng.uniform(-10.0,10.0)
                y = rng.uniform(-10.0,10.0)
                z = rng.uniform(0.0,2.0)
                        
                points.append([x,y,z])

            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1)]

            # either do pc2 = PointCloud2() then fill in the fields manually 
            # or use Tim Field's point_cloud2.create_cloud() which returns a PointCloud2 obj

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map" # this is what space/frame the points are defined in reference to. ie. robot vs map
            pc2 = point_cloud2.create_cloud(header, fields, points)

            # publish so we can see that in rviz
            self.pc2_pub.publish(pc2)
            
            print(pc2)
            print('--- \n')

            self.rate.sleep()

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('pc2_test', anonymous=True)
    ic = PointCloudTest()
    ic.run()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
