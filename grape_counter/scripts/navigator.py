#! /usr/bin/env python

# using template of uol_cmp9767m_tutorial/scripts/set_topo_nav_goal.py
# by gpdas, email: pdasgautham@gmail.com


import rospy
import actionlib

from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

from std_msgs.msg import Int32

from grape_counter.srv import SetMode, SetModeRequest



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
        
        self.total_count = 0

        # set up action client for sending nav goals
        self.nav_client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
        self.nav_client.wait_for_server()

        # set up service to send mode to manager node
        rospy.wait_for_service('set_mode')
        self.set_mode_srv = rospy.ServiceProxy('set_mode', SetMode)

        # subscribe to current grape count
        rospy.Subscriber("/grape_count", Int32, self.update_count)

    
    def update_count(self, msg):
        self.total_count = msg.data
        print("Current count: {}.".format(self.total_count))
        

    def run(self):
        
        for i, leg in enumerate(mission):
            # create a goal message and set target from mission list
            goal = GotoNodeGoal()
            goal.target = leg["goal"]
            goal.no_orientation = False
            
            # only attempt looking at next leg if we're not on the final leg
            if not (i == len(mission)-1):
                if mission[i+1]["action"] == SetModeRequest.TRAVEL:
                    # if the action of the next leg is travel 
                    # then don't worry about orientation at goal
                    goal.no_orientation = True
            
            print("Next waypoint is {} with mode {}.".format(goal.target, leg["action"]))

            # if want to start a new count (for a different vine) get and sum curret count before reset
            if leg["action"] == "new_count":
                # read count from topic
                count = 0
                self.total_count += count

            # tell manager node what to do
            response = self.set_mode_srv(leg["action"])
            #print(response.result)
           
            # send goal and wait for it to finish
            self.nav_client.send_goal(goal)
            self.nav_client.wait_for_result()
            
            # get result, log and continue to next leg
            result = self.nav_client.get_result()
            print("Reached {} : {}.".format(goal.target, result))

        # print finish
        print("Mission complete. The total grape bunches counted was {}.".format(self.total_count))


if __name__=="__main__":
    rospy.init_node('grape_navigator', anonymous=True)
    navigator = Navigator()
    print("Navigator Initialised. Starting Navigation...")
    navigator.run()
