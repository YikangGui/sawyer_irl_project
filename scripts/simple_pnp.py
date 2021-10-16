#!/usr/bin/env python

from PickandPlace import PickAndPlace

import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Int8MultiArray
from operator import mod
import random
import sys
import rospy
from gripper_to_position import reset_gripper, activate_gripper, gripper_to_pos
import numpy as np

pnp = PickAndPlace(target_location_x = 0.9, target_location_y = 0.25)

def main():
  try:

  
    # reset_gripper()

    # activate_gripper()

  #   # 255 = closed, 0 = open
    # gripper_to_pos(0, 60, 200, False)    # OPEN GRIPPER

    # rospy.sleep(1.0)
    # gripper_to_pos(255, 255, 200, False)    # GRIPPER TO POSITION 50
    group = pnp.group
    current_pose = group.get_current_pose().pose
    allow_replanning = False
    planning_time = 15
    # pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
    # status = pnp.go_to_pose_goal(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w, 0.75, 0.0, 0.08, allow_replanning, planning_time, thresh = 0.001)
    status = pnp.go_to_pose_goal(pnp.q[0], pnp.q[1], pnp.q[2], pnp.q[3], 0.75, 0.0, 0.078, allow_replanning, planning_time, thresh = 0.001)

    # state = pnp.robot.get_current_state()
    # group.set_start_state(state)
    # for i in range(5):
    #   pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
    #   status = pnp.go_to_pose_goal(pnp.q[0], pnp.q[1], pnp.q[2], pnp.q[3], 0.9, 0.25, 0.2, allow_replanning, planning_time, thresh = 0.001)
    #   pnp.staticDip(z_pose = 0.10)
    #   pnp.liftgripper()
    #   pnp.goto_bin(usePoseGoal = False)
    #   pnp.goto_home(0.3, goal_tol=0.01, orientation_tol=0.1)
    #   status = pnp.go_to_pose_goal(pnp.q[0], pnp.q[1], pnp.q[2], pnp.q[3], 0.9, 0.25, 0.2, allow_replanning, planning_time, thresh = 0.001)
    #   pnp.staticDip(z_pose = 0.10)
    #   pnp.liftgripper()
    #   pnp.goto_placeOnConv()
    #   rospy.sleep(0.1)
    print "\n",group.get_current_pose().pose.position

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
	try:
  		main()
	except rospy.ROSInterruptException:
	    pass
