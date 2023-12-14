#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
import rospy
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class Proprioception(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(Proprioception, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('proprioception', anonymous=True)
    pub = rospy.Publisher('/proprioception_ee', Pose, queue_size=10)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()
    self.pub = pub
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    #self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names




  def publishEEPose(self):
      wpose = self.move_group.get_current_pose().pose
      self.pub.publish(wpose)



def main():
  try:
    prop = Proprioception()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        prop.publishEEPose()
        rate.sleep()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
