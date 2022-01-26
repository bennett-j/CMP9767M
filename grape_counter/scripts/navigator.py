#! /usr/bin/env python

# using template of uol_cmp9767m_tutorial/scripts/set_topo_nav_goal.py
# by gpdas, email: pdasgautham@gmail.com

import queue
#from urllib import response
import rospy
import actionlib

from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

from std_msgs.msg import String

from grape_counter.srv import SetMode, SetModeRequest

#from gc_params import Modes

# action travel, count, new_count

mission = (
    {"goal": "Home", "action": SetModeRequest.TRAVEL},
    {"goal": "WP6", "action": SetModeRequest.TRAVEL},
    {"goal": "WP8", "action": SetModeRequest.COUNT},
    {"goal": "WP9", "action": SetModeRequest.TRAVEL},
    {"goal": "WP7", "action": SetModeRequest.COUNT},
    {"goal": "Home", "action": SetModeRequest.TRAVEL},
)



class Navigator:
    def __init__(self):
        #rospy.init_node('topological_navigation_client')

        self.count_status = rospy.Publisher("/count_status", String, queue_size=1)

        self.client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
        self.client.wait_for_server()

        self.total_count = 0

        rospy.wait_for_service('set_mode')
        self.set_mode_srv = rospy.ServiceProxy('set_mode', SetMode)
        print("service set up")

    def run(self):
        
        for i, leg in enumerate(mission):
            # create a goal message and set target from mission list
            goal = GotoNodeGoal()
            goal.target = leg["goal"]
            
            
            
            print(goal.target)

            # if want to start a new count (for a different vine) get and sum curret count before reset
            if leg["action"] == "new_count":
                # read count from topic
                count = 0
                self.total_count += count

            # tell manager node what to do
            # print(String(leg["action"]))
            # self.count_status.publish(String(leg["action"]))
            
            response = self.set_mode_srv(leg["action"])
            print(response.result)
           
            # send goal and wait for it to finish
            self.client.send_goal(goal)
            status = self.client.wait_for_result() # wait until the action is complete
            
            # get result, log and continue to next leg
            result = self.client.get_result()
            rospy.loginfo("status is %s", status)
            rospy.loginfo("result is %s", result)


if __name__=="__main__":
    rospy.init_node('grape_navigator', anonymous=True)
    navigator = Navigator()
    navigator.run()
