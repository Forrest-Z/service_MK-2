#! /usr/bin/env python
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the charging action, including the
# goal message and the result message.
import my_first_ros_pkg.msg

def charging_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (chargingAction) to the constructor.
    client = actionlib.SimpleActionClient('charging_action', my_first_ros_pkg.msg.TesttAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = my_first_ros_pkg.msg.TesttGoal(order="test1234")

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A chargingResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('charging_client')
        result = charging_client()
        print("Result:" + result.result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
