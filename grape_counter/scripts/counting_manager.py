#!/usr/bin/env python

# Author: James Bennett
# Jan 2022

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt

from grape_counter.srv import SetMode, SetModeRequest, SetModeResponse


class CountingManager:
    """
    Once a go to goal in topo nav has been started, this will be started to control the stop-start to get images and send them on to be processed.
    """
    
    def __init__(self):
           
        # == CONSTANTS ==
        # distance to travel between slowing to capture images
        self.DISTANCE_TO_TRAVEL = 1 # metre
        # below this velocity, robot considered stationary
        self.STATIONARY_THRESH = 0.01 # m/s
        # this creates a message with all values set to 0
        self.STATIONARY_MSG = Twist()

        # == INITIALISE VALUES ==
        # self.distance stores distance wrt self.odom_ref
        self.distance = 0.0
        self.odom_ref = None
        self.curr_vel = 0.0
        # mode set by the service SetMode
        self.mode = 0

        # rate for publishing cmd_vel and loop rate
        self.rate = rospy.Rate(10.0) #Hz

        # == TOPICS AND SERVICES ==
        # cmd_vel publisher to take priority over the nav_vel to enable momentry stationary
        self.cmd_vel_pub = rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel', Twist, queue_size=1)
        # publisher to instruct image node to process the image
        self.img_pub = rospy.Publisher('/process_img', String, queue_size=1)
        # service server to set mode
        rospy.Service('set_mode', SetMode, self.handle_set_mode)
        # subscribe to odometry to calculate distance travelled and current velocity        
        rospy.Subscriber("/thorvald_001/odometry/gazebo", Odometry, self.odom_callback)
        

    def odom_callback(self, odom_msg):
        # set reference position if there is none
        if self.odom_ref is None:
            self.odom_ref = odom_msg.pose.pose.position

        else:
            # calculate distance between current odom and reference
            # assume z remains 0
            self.distance = sqrt((odom_msg.pose.pose.position.x - self.odom_ref.x) ** 2
                                +(odom_msg.pose.pose.position.y - self.odom_ref.y) ** 2)

            # calculate current velocity
            self.curr_vel = sqrt(odom_msg.twist.twist.linear.x ** 2 
                               + odom_msg.twist.twist.linear.y ** 2)

    def reset_odom(self):
        # set ref to none so that a new reference position is created
        self.odom_ref = None
        self.distance = 0.0

    def handle_set_mode(self, req):
        # set mode from service and return mode to indicate success
        self.mode = req.mode
        print("Mode set to: ", req.mode, " by server")
        return SetModeResponse(req.mode)

    def run(self):
        """Runs continuously and executes slowing behaviour when COUNT mode enabled."""
        
        while not rospy.is_shutdown():
            #print('status: ', self.mode)
            
            if self.mode == SetModeRequest.COUNT:
                
                if self.distance > self.DISTANCE_TO_TRAVEL:
                    
                    # when specified distance has been travelled,
                    # come to a stop by publishing zero velocity messages
                    while self.curr_vel > self.STATIONARY_THRESH:
                        self.cmd_vel_pub.publish(self.STATIONARY_MSG)
                        self.rate.sleep()
                    
                    # sleep just to let everything settle (5*10Hz = 0.5s)
                    for i in range(5):
                        self.cmd_vel_pub.publish(self.STATIONARY_MSG)
                        self.rate.sleep()
                                        
                    # send command to do an img
                    self.img_pub.publish('process_image')
                    print("img cmd sent")
                    
                    # reset odom
                    self.reset_odom()
                    
                    # by not sending the stationary msg to cmd_vel thorvald will start moving again

            # regulate frequency of loop
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('counting_mgr')
    mgr = CountingManager()
    print("Counting Manager Initialised. Now running...")
    mgr.run()