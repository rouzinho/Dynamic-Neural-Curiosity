#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
import rospy
import rosbag
roslib.load_manifest('dmp')
from dmp.srv import *
from dmp.msg import *
import numpy as np
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
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

#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException:
        print("Service call failed: %s")


#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print("Starting DMP planning...")
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException:
        print("Service call failed: %s")
    print("DMP planning done")   
            
    return resp;


class Motion(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(Motion, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion', anonymous=True)
    rate = rospy.Rate(100)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    rospy.Subscriber('/path_ee', Pose, self.callbackPath)
    rospy.Subscriber('/home_position', Bool, self.callbackHome)
    pub = rospy.Publisher('/ee_moving', Bool, queue_size=10)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.pub = pub
    self.pose_goal = geometry_msgs.msg.Pose()
    self.init_pose = geometry_msgs.msg.Pose()
    self.pose_home = geometry_msgs.msg.Pose()
    self.current_pose = geometry_msgs.msg.Pose()
    self.move = False
    self.home = False
    self.count = 0
    self.path = []
    self.dims = 3 
    self.dt = 1.0                
    self.K = 100              
    self.D = 2.0 * np.sqrt(self.K)      
    self.num_bases = 4
    
  def callbackOneAction(self,data):
    self.pose_goal.orientation.x = -0.38
    self.pose_goal.orientation.y = 0.92
    self.pose_goal.orientation.z = 0.0
    self.pose_goal.orientation.w = 0.0
    self.pose_goal.position.x = data.position.x
    self.pose_goal.position.y = data.position.y
    self.pose_goal.position.z = data.position.z
    self.move = True

  def callbackPath(self,data):
    self.pose_goal.orientation.x = 0.92
    self.pose_goal.orientation.y = -0.38
    self.pose_goal.orientation.z = 0.0
    self.pose_goal.orientation.w = 0.0
    self.pose_goal.position.x = data.position.x
    self.pose_goal.position.y = data.position.y
    self.pose_goal.position.z = data.position.z
    self.count = self.count + 1
    if self.count == 2:
      self.move = True
      self.count = 0
    if self.count == 1:
      self.path = []
      self.init_pose.orientation.x = 1.0
      self.init_pose.orientation.y = 0.0
      self.init_pose.orientation.z = 0.0
      self.init_pose.orientation.w = 0.0
      self.init_pose.position.x = data.position.x
      self.init_pose.position.y = data.position.y
      self.init_pose.position.z = 0.3
      #self.pose_goal.position.z = 0.3
      #self.path.append(copy.deepcopy(self.pose_goal))
      #self.pose_goal.position.z = data.position.z
    self.path.append(copy.deepcopy(self.pose_goal))

  def callbackHome(self,is_home):
    if is_home.data == True:
      self.count = 0
      self.home = True

  def go_to_pose_goal(self):
    move_group = self.move_group
    move_group.set_pose_target(self.init_pose)
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    move_group.clear_pose_targets()
    self.move = False
    current_pose = self.move_group.get_current_pose().pose

    return all_close(self.pose_goal, current_pose, 0.01)

  def go_to_home_pose(self):
    self.pose_home.orientation.x = 1.0
    self.pose_home.orientation.y = 0.0
    self.pose_home.orientation.z = 0.0
    self.pose_home.orientation.w = 0.0
    self.pose_home.position.x = 0.3
    self.pose_home.position.y = 0.0
    self.pose_home.position.z = 0.3
    move_group = self.move_group
    move_group.set_pose_target(self.pose_home)
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    move_group.clear_pose_targets()
    self.move = False
    self.count = 0
    current_pose = self.move_group.get_current_pose().pose

    return all_close(self.pose_goal, current_pose, 0.01)

  def get_current_pose(self):
    self.current_pose = self.move_group.get_current_pose().pose
    return self.current_pose

  def getMove(self):
    return self.move

  def setMove(self,status):
    self.move = status

  def setIsMoving(self,status):
    isMoving = Bool()
    isMoving.data = status
    self.pub.publish(isMoving)

  def getHome(self):
    return self.home

  def setHome(self,status):
    self.home = status

  def printStatus(self):
      print("============ Printing robot state")
      #wpose = self.move_group.get_current_pose().pose
      print(self.path)
      print("============")

  def plan_cartesian_path(self, scale=1):
    move_group = self.move_group
    (plan, fraction) = move_group.compute_cartesian_path(
                                       self.path,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    self.setIsMoving(True)
    move_group.execute(plan, wait=True)
    self.setIsMoving(False)

  def plan_path_DMP(self, p, scale=1):
    move_group = self.move_group
    dmp_path = []
    goal = geometry_msgs.msg.Pose()
    for i in p:
        #print("adding goal",i)
        goal.orientation.x = 0.92
        goal.orientation.y = -0.38
        goal.orientation.z = 0.0
        goal.orientation.w = 0.0
        goal.position.x = i[0]
        goal.position.y = i[1]
        goal.position.z = i[2]
        #goal.position.z = 0.140
        dmp_path.append(copy.deepcopy(goal))
    #print(dmp_path)
    (plan, fraction) = move_group.compute_cartesian_path(
                                       dmp_path,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    self.setIsMoving(True)
    move_group.execute(plan, wait=True)
    self.setIsMoving(False)


def main():
  try:

    motion_planning = Motion()
    #rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if motion_planning.getMove() == True:
          #go the first pose to avoid squeezing the object and make it fly away
          motion_planning.go_to_pose_goal()
          #motion_planning.setIsMoving(True)
          motion_planning.plan_cartesian_path()
          #motion_planning.setIsMoving(False)
          motion_planning.printStatus()
          motion_planning.setMove(False)
          motion_planning.go_to_home_pose()
        if motion_planning.getHome() == True:
          motion_planning.go_to_home_pose()
          motion_planning.setHome(False)
        #rate.sleep()



  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
