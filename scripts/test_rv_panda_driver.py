#!/usr/bin/env python3

import rospy
import time
import actionlib
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal

# initialise ros node
rospy.init_node('rv_panda_test', anonymous=True)

'''
cartesian pose example
'''
# Create a ros action client to communicate with the driver
client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)

client.wait_for_server()
# Create a target pose
target = PoseStamped()
target.header.frame_id = 'panda_link0'
# Populate with target position/orientation (READY POSE)
target.pose.position.x = 0.400
target.pose.position.y = 0.000
target.pose.position.z = 0.350
target.pose.orientation.x = -1.00
target.pose.orientation.y = 0.00
target.pose.orientation.z = 0.00
target.pose.orientation.w = 0.00
# Create goal from target pose
goal = MoveToPoseGoal(goal_pose=target)
# Send goal and wait for it to finish
client.send_goal(goal)
client.wait_for_result()


'''
cartesian velocity example
'''
# publisher = rospy.Publisher('/arm/cartesian/velocity', TwistStamped, queue_size=1)
# # Create an initial start time
# start_timer = time.time()
# velocity = TwistStamped()
# velocity.twist.linear.z = -0.02
# # frequency of 100Hz
# while (time.time() - start_timer) < 5:
#   publisher.publish(velocity)
#   rospy.sleep(0.01)
# # Publish an empty TwistStamped to ensure that the arm stops moving
# publisher.publish(TwistStamped())
