#! /usr/bin/env python

import rospy

import actionlib

import my_first_ros_pkg.msg

class chargingAction(object):
    # create messages that are used to publish feedback/result
    _feedback = my_first_ros_pkg.msg.TesttFeedback()
    _result = my_first_ros_pkg.msg.TesttResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, my_first_ros_pkg.msg.TesttAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating charging sequence of order %s ' % (self._action_name, goal.order))
        idx = 0
        self._feedback.feedback = my_first_ros_pkg.msg.TesttActionFeedback()
        #write to charging flow
        while(True):
            self._feedback.feedback  = str(idx)
            self._as.publish_feedback(self._feedback)
            r.sleep()
            if idx >10 :
                success = True
                break
            idx = idx + 1
        
        # start executing the action
        # for i in range(1, goal.order):
        #     # check that preempt has not been requested by the client
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         success = False
        #         break
        #     self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
        #     # publish the feedback
        #     self._as.publish_feedback(self._feedback)
        #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #     r.sleep()
          
        if success:
            self._result.result = "end action"
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('charging_server_example')
    server = chargingAction("charging_action")
    rospy.spin()
