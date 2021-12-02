#!/usr/bin/env python3
import math
import rospy
from moveit_commander import MoveGroupCommander

if __name__ == '__main__':
  rospy.init_node('move_to_start')
  commander = MoveGroupCommander('panda_arm')
  joint_goal = commander.get_current_joint_values()

  # OCT scan home configuration
  joint_goal[0] = 0.0000
  joint_goal[1] = 0.0000  # -math.pi/6
  joint_goal[2] = 0.0000
  joint_goal[3] = -math.pi/2  # -2*math.pi/3
  joint_goal[4] = 0.0000
  joint_goal[5] = math.pi/2
  joint_goal[6] = math.pi/6

  commander.go(joint_goal, wait=True)
  commander.stop()
