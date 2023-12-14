#!/usr/bin/env python3


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rosbag
import roslib; 
roslib.load_manifest('dmp')
import numpy as np
from dmp.srv import *
from dmp.msg import *
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
import os.path
from os import path
 
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

def formDatasEE(n):
    name = n
    tot = []

    bag = rosbag.Bag(name)
    for topic, msg, t in bag.read_messages(topics=['ee_pos']):
        ee = []
        x = msg.position.x
        ee.append(x)
        y = msg.position.y
        ee.append(y)
        z = msg.position.z
        ee.append(z)
        tot.append(ee)

    bag.close() 

    return tot

def formDatasJointsEE():
    name = "/home/altair/PhD/Codes/catkin_motion/rosbags/dataset/clean_bags/totop/1_motionEE.bag"
    tot = []
    bag = rosbag.Bag(name)
    for topic, msg, t in bag.read_messages(topics=['ee_pos']):
      tot.append(msg)

    return tot

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                   D_gain, num_bases):
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        
    print ("Starting LfD...")
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException:
        print("Service call failed: %s")
        print("LfD done")    
            
    return resp;


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
    rospy.Subscriber('/cylinder_position', Pose, self.callbackCube)
    rospy.Subscriber('/ball_position', Pose, self.callbackCube)
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
    self.cube = geometry_msgs.msg.Pose()
    self.wpose = geometry_msgs.msg.Pose()
    self.move = False
    self.home = False
    self.count = 0
    self.path = []

  def callbackCube(self,data):
    self.cube.position.x = data.position.x
    self.cube.position.y = data.position.y
    self.cube.position.z = data.position.z
    
  def callbackAction(self,data):
    self.pose_goal.orientation.x = 0.93
    self.pose_goal.orientation.y = -0.38
    self.pose_goal.orientation.z = 0.0
    self.pose_goal.orientation.w = 0.0
    self.pose_goal.position.x = data.position.x
    self.pose_goal.position.y = data.position.y
    self.pose_goal.position.z = data.position.z
    self.move = True

  def callbackPath(self,data):
    self.pose_goal.orientation.x = 0.93
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
    plan = move_group.go()
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
    self.pose_home.position.x = 0.37
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

  def getMove(self):
    return self.move

  def setMove(self,status):
    self.move = status

  def get_current_pose(self):
    self.wpose = self.move_group.get_current_pose().pose
    return self.wpose

  def getCube(self):
    return self.cube

  def setIsMoving(self,status):
    isMoving = Bool()
    isMoving.data = status
    self.pub.publish(isMoving)

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
    self.setIsMoving(True)
    move_group.execute(plan, wait=True)
    self.setIsMoving(False)

  def plan_pathEE(self, p, scale=1):
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
    #move_group.plan(plan)
    self.setIsMoving(True)
    move_group.execute(plan, wait=True)
    self.setIsMoving(False)
    


def writeDMPBag(data):
  name = "rosbags/experiment/dmp/dmp_motion.bag"
  exist = path.exists(name)
  opening = ""
  if(exist == True):
      opening = "a"
  else:
      opening = "w"
  bag = rosbag.Bag(name, opening)
  try:
      bag.write("dmp_pos",data)
  finally:
      bag.close()

def getDMP(name):
  bag = rosbag.Bag(name)
  for topic, msg, t in bag.read_messages(topics=['dmp_pos']):
    resp = msg
    
    bag.close()
  #print(resp)
  return resp



def main():
  try:
   
    motion_planning = Motion()
    #rate = rospy.Rate(10)
    
    
    t = formDatasEE("rosbags/experiment/motion_EE.bag")
    #print(t)
    #print("---------------------------")
    print("HOME")
    #motion_planning.go_to_home_pose()
    #print("replay bag")
    #motion_planning.plan_dmp_path(t)
    #Create a DMP from a 2-D trajectory
    #print("go home pose")
    #motion_planning.go_to_home_pose()
    #rate.sleep()
    
    dims = 3 
    dt = 1.0                
    K = 100              
    D = 2.0 * np.sqrt(K)      
    num_bases = 4
    #c = motion_planning.getCube()
    #traj = [[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
    p = motion_planning.get_current_pose()
    #print(p)
    
    #traj = [[p.position.x,p.position.y],[c.position.x+0.1,c.position.y]]

    #response = makeLFDRequest(dims, t, dt, K, D, num_bases)
    #writeDMPBag(response)
    response = getDMP("rosbags/experiment/dmp/dmp_60_62.bag")
    #response = getDMP("rosbags/experiment/dmp/dmp_motion.bag")

    #Set it as the active DMP
    makeSetActiveRequest(response.dmp_list)

    x_0 = [p.position.x,p.position.y,p.position.z]         #Plan starting at a different point than demo 
    x_dot_0 = [0.0,0.0,0.0]   
    t_0 = 0                
    #goal = [c.position.x,c.position.y,0.140]         #Plan to a different goal than demo
    goal = [0.45,0.05,0.140]
    goal_thresh = [0.1]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * response.tau       #Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1  
    planned_dmp = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)

    #print(resp.plan.points.positions)
    path_dmp = []
    for i in planned_dmp.plan.points:
        tmp = []
        tmp.append(i.positions[0])
        tmp.append(i.positions[1])
        tmp.append(i.positions[2])
        path_dmp.append(tmp)


    print(len(path_dmp))
    print("play DMP")
    #motion_planning.plan_pathEE(path_dmp)
    #motion_planning.go_to_home_pose()



  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
