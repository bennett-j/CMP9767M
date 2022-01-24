#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt


class CountingManager:
    """
    Once a go to goal in topo nav has been started, this will be started to control the stop-start to get images and send them on to be processed.
    """

    def __init__(self):
        """
        """

        # reference position to use to count distance travelled
        self.odom_ref = None
        self.distance = 0.0

        # distance to travel between slowing to take picture
        self.spacing = 1
        
        rospy.Subscriber("/thorvald_001/odometry/gazebo", Odometry, self.odom_callback)
        rospy.Subscriber("/count_status", String, self.status_callback)

        # 
        self.cmd_vel_pub = rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel', Twist, queue_size=1)
        self.stationary_msg = Twist()
        self.cmd_vel_rate = rospy.Rate(10.0) #Hz
        self.curr_vel = 0
        self.stationary_thresh = 0.01 # m/s

        #
        self.img_pub = rospy.Publisher('/process_img', String, queue_size=1)


    def odom_callback(self, odom_msg):
        if self.odom_ref is None:
            self.odom_ref = odom_msg.pose.pose.position

        else:
            # calculate distance between current odom and reference
            # assume z remains 0
            self.distance = sqrt((odom_msg.pose.pose.position.x - self.odom_ref.x) ** 2
                                +(odom_msg.pose.pose.position.y - self.odom_ref.y) ** 2)

            print("d: ", self.distance)

            # calculate current velocity
            self.curr_vel = sqrt(odom_msg.twist.twist.linear.x ** 2 
                               + odom_msg.twist.twist.linear.y ** 2)
            print("v: ", self.curr_vel)

    def reset_odom(self):
        self.odom_ref = None
        self.distance = 0.0

    def status_callback(self, msg):
        self.status = msg.data

        print("status callback: ", msg.data)

        if self.status == 'start':
            self.do_counting()

    def do_counting(self):
        while self.status == 'start':
            # enter if statement when specified distance has been travelled
            if self.distance > self.spacing:
                # come to a stop
                while self.curr_vel > self.stationary_thresh:
                    self.cmd_vel_pub.publish(self.stationary_msg)
                    self.cmd_vel_rate.sleep()

                # sleep just to let everything settle (5*10Hz = 0.5s)
                for i in range(5):
                    self.cmd_vel_pub.publish(self.stationary_msg)
                    self.cmd_vel_rate.sleep()
                                    
                # cmd to do an img
                self.img_pub.publish('process_image')
                print("img cmd sent")
                
                # reset odom
                self.reset_odom()
                
                # by not sending the stationary msg to cmd_vel thorvald will start moving again

            # put a sleep here?


if __name__ == '__main__':
    rospy.init_node('counting_mgr')
    CountingManager()
    rospy.spin()