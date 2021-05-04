#! /usr/bin/env python

import rospy
import actionlib
from move_tb3 import MoveTB3
from com2009_actions.msg import SearchAction, SearchGoal

class action_client(object):

    def feedback_callback(self, feedback_data):
        self.distance = feedback_data.current_distance_travelled
        if self.i < 100:
            self.i += 1
        else:
            self.i = 0
            print("FEEDBACK: Currently travelled {:.3f} m".format(self.distance))


    def __init__(self):
        #Initialise some variables
        self.action_complete = False
        self.distance = 0.0
        self.i = 0
        self.closest_object_distance = 0

        #Initialise the node
        rospy.init_node("search_action_client")

        self.rate = rospy.Rate(1)

        #Create action goal object, populated later
        self.goal = SearchGoal()

        #Create connection to the action server
        self.client = actionlib.SimpleActionClient("/search_action_server",
                    SearchAction)
        self.client.wait_for_server()

        #Function to be executed when the node is stopped
        rospy.on_shutdown(self.shutdown_ops)


    # Makes sure current goal cancelled when action completes or shut down
    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled")


    # Define goal and issue to server
    def send_goal(self, velocity, approach):
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = approach

        #Send the goal to the action server:
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)


    # Call the Goal
    # Monitor state action with while loop
    def main(self):
        self.send_goal(velocity = 0.19, approach = 0.5)
        prempt = False
        while self.client.get_state() < 2:
            print("FEEDBACK: Currently travelled {:.3f} m, STATE: Current state code is {}".format(self.distance, self.client.get_state()))
            if self.distance >= 5:
                rospy.logwarn("Cancelling goal now...")
                self.client.cancel_goal()
                rospy.logwarn("Goal Cancelled")
                prempt = True
                break

        self.rate.sleep()

        self.action_complete = True
        print("RESULT: Action State = {}".format(self.client.get_state()))
        if prempt:
            print("RESULT: Action preempted after travelling 2 meters")
        else:
            result = self.client.get_result()
            print("RESULT: closest object {:.3f} m away at a location of {:.3f} degrees".format(result.closest_object_distance, result.closest_object_angle))

# Launches the action server then runs main()
if __name__ == '__main__':
    ac_object = action_client()
    try:
        ac_object.main()
    except rospy.ROSInterruptException:
        pass
