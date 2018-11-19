#! /usr/bin/env python

import rospy
#import sys

import matplotlib.pyplot as plt
import actionlib
import thorvald_penetrometer.msg


class penetrometer_probe_client(object):
    
    def __init__(self) :
        self.plotted=False
        rospy.on_shutdown(self._on_node_shutdown)
        self.client = actionlib.SimpleActionClient('/thorvald_penetrometer', thorvald_penetrometer.msg.ProbeSoilAction)
        
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")
    
        probegoal = thorvald_penetrometer.msg.ProbeSoilGoal()
    
    
        probegoal.order = 0
        #navgoal.origin = orig
    
        # Sends the goal to the action server.
        self.client.send_goal(probegoal)#,self.done_cb, self.active_cb, self.feedback_cb)
    
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()
    
        # Prints out the result of executing the action
        ps = self.client.get_result()  # A FibonacciResult
        print ps
        if ps.result:
            plt.plot(ps.depth, ps.force)
            self.plotted=True
            plt.show()
        else:
            rospy.logerr("Probe Failed")

    def _on_node_shutdown(self):
        self.client.cancel_all_goals()
        if self.plotted:
            plt.close('all')
        #sleep(2)


if __name__ == '__main__':
    rospy.init_node('penetrometer_client')
    ps = penetrometer_probe_client()