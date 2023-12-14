#!/usr/bin/env python


from os import name
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import roslib
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
from perception.msg import ObjectGoal
import glob
import os.path

 
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
    rospy.Subscriber('/motion_panda/path_ee', Pose, self.callbackPath)
    rospy.Subscriber('/motion_panda/home_position', Bool, self.callbackHome)
    rospy.Subscriber('/motion_panda/go_to_dmp_pose', Pose, self.go_to_dmp_pose_goal)
    rospy.Subscriber('/motion_panda/dmp_object_goal', ObjectGoal, self.callbackDMPObjectGoal)
    pub = rospy.Publisher('/motion_panda/ee_moving', Bool, queue_size=10)
    pub_end = rospy.Publisher('/motion_panda/end_action', Bool, queue_size=10)
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
    self.pub_end = pub_end
    self.pose_goal = geometry_msgs.msg.Pose()
    self.init_pose = geometry_msgs.msg.Pose()
    self.pose_home = geometry_msgs.msg.Pose()
    self.current_pose = geometry_msgs.msg.Pose()
    self.move = False
    self.move_dmp = False
    self.home = False
    self.activate_dmp = False
    self.dmp = ObjectGoal()
    self.goal_dmp = geometry_msgs.msg.Pose()
    self.count = 0
    self.path = []
    

  def callbackPath(self,data):
    if self.count == 1:
      self.pose_goal.orientation.x = 0.93
      self.pose_goal.orientation.y = -0.38
      self.pose_goal.orientation.z = 0.0
      self.pose_goal.orientation.w = 0.0
      self.pose_goal.position.x = data.position.x
      self.pose_goal.position.y = data.position.y
      self.pose_goal.position.z = 0.140
      self.count += 1
      self.path.append(copy.deepcopy(self.pose_goal))
    if self.count == 0:
      self.path = []
      self.pose_goal.orientation.x = 1.0
      self.pose_goal.orientation.y = 0.0
      self.pose_goal.orientation.z = 0.0
      self.pose_goal.orientation.w = 0.0
      self.pose_goal.position.x = data.position.x
      self.pose_goal.position.y = data.position.y
      self.pose_goal.position.z = 0.3
      self.path.append(copy.deepcopy(self.pose_goal))
      self.pose_goal.position.z = 0.140
      self.pose_goal.orientation.x = 0.93
      self.pose_goal.orientation.y = -0.38
      self.path.append(copy.deepcopy(self.pose_goal))
      self.count += 1
    if self.count == 2:
      self.count = 0
      self.move = True

  def callbackHome(self,is_home):
    if is_home.data == True:
      self.count = 0
      self.home = True

  def callbackCustomPose(self,cp):
    move_group = self.move_group
    move_group.set_pose_target(cp)
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    move_group.clear_pose_targets()
    self.move = False
    current_pose = self.move_group.get_current_pose().pose

  def go_to_dmp_pose_goal(self,p):
    self.goal_dmp.position.x = p.position.x
    self.goal_dmp.position.y = p.position.y
    self.goal_dmp.position.z = 0.140
    self.goal_dmp.orientation.x = 0.93
    self.goal_dmp.orientation.y = -0.38
    self.goal_dmp.orientation.z = 0.0
    self.goal_dmp.orientation.w = 0.0
    self.move_dmp = True
    #print(self.goal_dmp)
    

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
    self.setIsEEMoving(False)

    return all_close(self.pose_goal, current_pose, 0.01)

  def callbackDMPObjectGoal(self,og):
    self.dmp.goal = og.goal
    self.dmp.object = og.object
    print("PANDA ACTIVATE DMP : ",self.dmp)

  def get_current_pose(self):
    self.current_pose = self.move_group.get_current_pose().pose
    return self.current_pose

  def getMove(self):
    return self.move

  def setMove(self,status):
    self.move = status

  def getMoveDMP(self):
    return self.move_dmp

  def setMoveDMP(self,status):
    self.move_dmp = status

  def resetDMP(self):
    self.move_dmp = False
    self.dmp = ObjectGoal()

  def setIsEEMoving(self,status):
    isMoving = Bool()
    isMoving.data = status
    self.pub.publish(isMoving)

  def setEndMotion(self,status):
    end = Bool()
    end.data = status
    self.pub_end.publish(end)

  def getHome(self):
    return self.home

  def setHome(self,status):
    self.home = status

  def plan_cartesian_path(self, scale=1):
    move_group = self.move_group
    (plan, fraction) = move_group.compute_cartesian_path(
                                       self.path,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    self.setIsEEMoving(True)
    move_group.execute(plan, wait=True)
    
    move_group.stop()
    move_group.clear_pose_targets()

  def plan_path_DMP(self, p, scale=1):
    move_group = self.move_group
    dmp_path = []
    goal = geometry_msgs.msg.Pose()
    for i in p:
        #print("adding goal",i)
        goal.orientation.x = 0.93
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
    self.setIsEEMoving(True)
    move_group.execute(plan, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

  def getDMP(self,name):
    bag = rosbag.Bag(name)
    for topic, msg, t in bag.read_messages(topics=['dmp_pos']):
      resp = msg
      bag.close()

    return resp

  def playMotionDMP(self):
    curr = self.get_current_pose()
    goal_sup = self.dmp.goal + 1
    goal_inf = self.dmp.goal - 1
    name_dmp = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/dmp/dmp_" + str(int(self.dmp.object)) + "_" + str(int(self.dmp.goal)) + ".bag"
    if not os.path.isfile(name_dmp):
      name_dmp = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/dmp/dmp_" + str(int(self.dmp.object)) + "_" + str(int(goal_sup)) + ".bag"
      if not os.path.isfile(name_dmp):
        name_dmp = "/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/dmp/dmp_" + str(int(self.dmp.object)) + "_" + str(int(goal_inf)) + ".bag"
    #files = glob.glob(os.path.expanduser("/home/altair/PhD/Codes/catkin_noetic/rosbags/experiment/dmp/*"))
    #sorted_by_mtime_ascending = sorted(files, key=lambda t: os.stat(t).st_mtime)
    #print(sorted_by_mtime_ascending)
    resp = self.getDMP(name_dmp)
    #print("Get DMP :")
    #print(resp)
    makeSetActiveRequest(resp.dmp_list)
    goal = [self.goal_dmp.position.x,self.goal_dmp.position.y,self.goal_dmp.position.z]
    goal_thresh = [0.1]
    x_0 = [curr.position.x,curr.position.y,curr.position.z]         #Plan starting at a different point than demo 
    x_dot_0 = [0.0,0.0,0.0]   
    t_0 = 0                
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * resp.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1  
    planned_dmp = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, seg_length, tau, dt, integrate_iter)
    path_dmp = []
    for i in planned_dmp.plan.points:
        tmp = []
        tmp.append(i.positions[0])
        tmp.append(i.positions[1])
        tmp.append(i.positions[2])
        path_dmp.append(tmp)

    #print("path dmp :")
    #print(path_dmp)
    
    self.plan_path_DMP(path_dmp)



if __name__ == '__main__':
  motion_planning = Motion()
  while not rospy.is_shutdown():
      # if it's exploring
      #print(motion_planning.getMoveDMP())
      if motion_planning.getMove() == True:
        motion_planning.resetDMP()
        #go the first pose to avoid squeezing the object and make it fly away
        #motion_planning.go_to_pose_goal()
        motion_planning.plan_cartesian_path()
        motion_planning.setMove(False)
        motion_planning.go_to_home_pose()
        motion_planning.setEndMotion(True)
        rospy.sleep(1)
        motion_planning.setEndMotion(False)
      #if it's exploiting
      if motion_planning.getMoveDMP() == True:
        print("Motion PANDA : moving with DMP !")
        motion_planning.playMotionDMP()
        motion_planning.setMoveDMP(False)
        motion_planning.go_to_home_pose()
        motion_planning.setEndMotion(True)
        rospy.sleep(1)
        motion_planning.setEndMotion(False)