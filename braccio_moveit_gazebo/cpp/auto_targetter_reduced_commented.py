#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import time

from gazebo_msgs.msg import LinkStates, ModelState
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState

## END_SUB_TUTORIAL
import numpy as np
import scipy.optimize
import cv2
import json

import InvKin #as Arm3Link
# from InvKin import get_xy
# from InvKin import inv_kin

THETA_EXT = 0.27
THETA_RET = np.pi/4

L_FUDGE = 0.08

Z_MAX_SIDE = -0.03
Z_MAX_DOWN = 0
Z_MIN = -0.045

CLOSE_ENOUGH = 0.02
DEFAULT_ROT = 0

S_SIDE_MAX = 0.4
S_SIDE_MIN = 0.161
S_TOP_MAX = 0.29

def cart2pol(x, y):
    """helper, convert cartesian to polar coordinates"""
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    """helper,convert polar to cartesian"""
    x = rho*np.cos(phi)
    y = rho*np.sin(phi)
    return(x, y)

def get_other_angles(theta_shoulder):
  """helper, converting some angles"""
  theta_wrist = theta_shoulder + np.pi/2
  theta_elbow = np.pi/2 - 2*theta_shoulder
  return theta_wrist, theta_elbow

class BraccioObjectTargetInterface(object):
  """BraccioXYBBTargetInterface"""
  def __init__(self):
    super(BraccioObjectTargetInterface, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('braccio_xy_bb_target', anonymous=True)

    group_name = "braccio_arm"
    self.move_group = moveit_commander.MoveGroupCommander(group_name)
    self.gripper_group = moveit_commander.MoveGroupCommander("braccio_gripper")

    self.homography = None

    self.kinematics = InvKin.Arm3Link()
    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.linkstate_callback)

  def linkstate_callback(self, data):
    """callback to get link location for cube from gazebo"""
    try:
      self.linkstate_data = data
    except ValueError:
      pass

  #DONE!
  # #method to transform the coordinates with the homography method
  # def transform(self, x1, y1, r):
  #   """transform from gazebo coordinates into braccio coordinates"""
  #   if self.homography is not None:
  #     a = np.array([[x1, y1]], dtype='float32')
  #     res = cv2.perspectiveTransform(a[None, :, :], self.homography)[0][0]
  #     return float(res[0]), float(res[1]), DEFAULT_ROT
  #   else:
  #     raise ValueError('run or load calibration first!')

  #method to get the choosen fragment position
  def get_box_position(self):
    # x, y, r = self.get_link_position(['unit_box_1::link'])
    x, y, r = self.get_link_position([self.link_choose])
    return self.transform(x,y,r)

  #method to choose the fragment
  def get_link_choose(self, lk):
    self.link_choose = lk
    print(lk)
    print(self.link_choose)

  #method to get ALL the positions of the fragments
  def get_link_position(self, link_names):
    """get mean position of a list of links"""
    x = 0
    y = 0
    n = 0
    for l in link_names:
      ind = self.linkstate_data.name.index(l)
      res = self.linkstate_data.pose[ind].position
      x += res.x
      y += res.y
      n += 1
    return x/n, y/n, DEFAULT_ROT

   #DONE!
  #method to store the current joint values and to call the method go_to_joint
  # def go_to_j(self, j0=None, j1=None, j2=None, j3=None):
  #   """update arm joints"""
  #   joint_goal = self.move_group.get_current_joint_values()
  #   if j0 is not None:
  #     joint_goal[0]=j0
  #   if j1 is not None:
  #     joint_goal[1]=j1
  #   if j2 is not None:
  #     joint_goal[2]=j2
  #   if j3 is not None:
  #     joint_goal[3]=j3
  #   self.go_to_joint(joint_goal)

  # #method which calls the move_group.go funct with the joint values defined in go_to_j method
  # def go_to_joint(self, joint_targets):
  #   joint_goal = self.move_group.get_current_joint_values()
  #   joint_goal[0] = joint_targets[0]
  #   joint_goal[1] = joint_targets[1]
  #   joint_goal[2] = joint_targets[2]
  #   joint_goal[3] = joint_targets[3]
  #   joint_goal[4] = 1.5708
  #   ret = self.move_group.go(joint_goal, wait=True)
  #   self.move_group.stop()

  #DONE!
  # #method to set the gripper for closure
  # def gripper_close(self):
  #   self.go_gripper(1.2)

  # #method to set the gripper open
  # def gripper_open(self):
  #   self.go_gripper(0.9)

  # #method to set the gripper in the middle
  # def gripper_middle(self):
  #   self.go_gripper(1.05)

  # #method which moves the gripper (using what is defined in the methods above)
  # def go_gripper(self, val):
  #   joint_goal = self.gripper_group.get_current_joint_values()
  #   joint_goal[0] = val
  #   joint_goal[1] = val
  #   self.gripper_group.go(joint_goal, wait=True)
  #   self.gripper_group.stop()

  #DONE!
  # def go_to_raise(self):
  #   self.go_to_j(j1=1.15,j2=0.13,j3=2.29)
  #   # self.go_to_j(j1=1.0,j2=0.13,j3=2.29)

  # def go_to_pick(self,):
  #   self.go_to_j(j2=0.8)

  # def go_to_pull(self, phi):
  #   self.go_to_raise()
  #   self.gripper_close()
  #   if phi:
  #     self.go_to_j(j0=float(phi))
  #   self.go_to_j(j1=0.3,j2=1.7,j3=1.8)
  #   self.go_to_j(j1=0.3,j2=1.5,j3=0.3)
  #   self.go_to_j(j1=1.2,j2=0.3,j3=0.1)
  #   self.go_to_j(j1=1.5,j2=0.3,j3=0.1)
  #   self.go_to_j(j1=0.3,j2=1.8,j3=1.8)

  # def go_to_push(self, phi):
  #   self.go_to_raise()
  #   self.gripper_close()
  #   if phi:
  #     self.go_to_j(j0=float(phi))
  #   self.go_to_j(j1=2.7,j2=0.01,j3=0.01)
  #   self.go_to_j(j1=1.6,j2=0.01,j3=0.01)
  #   self.go_to_j(j1=0.3,j2=1.8,j3=0.1)
  #   self.go_to_j(j1=2.1,j2=0.01,j3=0.01)
  #   self.go_to_j(j1=2.7,j2=0.01,j3=0.01)

  def get_targets(self,x,y):
    s, phi = cart2pol(x,y)
    q = self.kinematics.inv_kin(s, Z_MIN, Z_MAX_SIDE, 0)
    xy = self.kinematics.get_xy(q)
    if np.abs(xy[0]-s) > CLOSE_ENOUGH:
      print('NO SOLUTION FOUND')
      print('goal distance = '+str(s))
      print('closest solution = '+str(xy[0]))
      return s, [phi, np.NaN, np.NaN, np.NaN]
    return s, [phi, q[0], q[1]+np.pi/2, q[2]+np.pi/2]

  def get_down_targets(self,x,y):
    s, phi = cart2pol(x,y)
    print(s, phi)
    q = self.kinematics.inv_kin(s, Z_MIN, Z_MAX_DOWN, -np.pi/2)
    xy = self.kinematics.get_xy(q)
    if np.abs(xy[0]-s) > CLOSE_ENOUGH:
      print('NO SOLUTION FOUND')
      print('goal distance = '+str(s))
      print('closest solution = '+str(xy[0]))
      return s, [phi, np.NaN, np.NaN, np.NaN]
    return s, [phi, q[0], q[1]+np.pi/2, q[2]+np.pi/2]

  def go_to_xy(self, x, y, r, how, bowl):
    if how=='top':
      s, joint_targets = self.get_down_targets(x,y)
      print(joint_targets)
      if joint_targets[0]<0 or joint_targets[0]>3.14:
        print('++++++ Not in reachable area, aborting ++++++')
        return -1
      # if np.isnan(joint_targets[1]) and s < S_TOP_MAX and s > S_SIDE_MIN:
      #   print('++++++ Too far out, pulling backwards +++++')
      #   # self.go_to_pull(joint_targets[0])
      #   # return 1
      #   return -1
      if np.isnan(joint_targets[1]):
        print('++++++ Not in reachable area, aborting ++++++')
        return -1
    # elif how=='side':
    #   s, joint_targets = self.get_targets(x,y)
    #   print(joint_targets)
    #   if joint_targets[0]<0 or joint_targets[0]>3.14:
    #     print('++++++ Not in reachable area, aborting ++++++')
    #     return -1
    #   if np.isnan(joint_targets[1]) and s < S_SIDE_MAX and s > S_SIDE_MIN:
    #     print('++++++ Too close, pushing backwards +++++')
    #     self.go_to_push(joint_targets[0])
    #     return 1
    #   if np.isnan(joint_targets[1]):
    #     print('++++++ Not in reachable area, aborting ++++++')
    #     return -1

    self.go_to_raise()

    self.gripper_open()
    self.go_to_j(j0=float(joint_targets[0]))

    self.go_to_j(j1=float(joint_targets[1]),
                j2=float(joint_targets[2]),
                j3=float(joint_targets[3]))

    self.gripper_close()
    # if how=='top' and joint_targets[2]<3:
    #   self.go_to_j(j2=float(joint_targets[2])+0.1)
    # self.go_to_home()
    home_ch = getattr(self, bowl)
    home_ch()
    return 0

  def go_to_manual_joint(self):
    joint_goal = self.move_group.get_current_joint_values()
    for i in range(len(joint_goal)):
      print('joint' + str(i) + ' ' + str(joint_goal[i]))
      tst = input()
      if tst!='':
          joint_goal[i] = float(tst)
    self.go_to_joint(joint_goal)

  def go_to_manual(self, how):
    print('pos x?')
    tst = input()
    if tst!='':
        x = float(tst)
    print('pos y?')
    tst = input()
    if tst!='':
        y = float(tst)
    return self.go_to_xy(x, y, DEFAULT_ROT, how)

  def go_to_manual_gripper(self):
    print('grip position?')
    tst = input()
    if tst!='':
        v = float(tst)
    self.go_gripper(v)

  def go_to_target(self, how, bowl):
    x,y,r = self.get_box_position()
    print(x, y, r)
    return self.go_to_xy(x, y, r, how, bowl)
  
  # def go_link_choose(self, lk):
  #     self.link_choose = lk
  #     print(lk)
  #     print(self.link_choose)
  
  def go_to_home(self):
    self.go_to_raise()
    self.go_to_j(j0=3.14)
    self.gripper_open()
    self.gripper_open()

  def go_to_home_0(self):
    # self.gripper_close()
    # self.go_to_raise()
    self.go_to_pick()
    self.go_to_j(j0=2.355)
    self.go_to_j(j1 = 1.67, j2 = 0.10, j3 = 0.5)
    self.gripper_open()
    self.gripper_open()

  def go_to_home_1(self):
    # self.gripper_close()
    # self.go_to_raise()
    self.go_to_pick()
    self.go_to_j(j0=0.785)
    self.go_to_j(j1=1.57, j2 = 3.00, j3 = 2.55)
    self.gripper_open()
    self.gripper_open()

  def go_to_home_2(self):
    # self.gripper_close()
    # self.go_to_raise()
    self.go_to_pick()
    self.go_to_j(j0 = 2.355)
    self.go_to_j(j1 = 1.47, j2 = 3.14, j3 = 2.50)
    self.gripper_open()
    self.gripper_open()

  def go_to_up(self):
    self.go_to_j(j0=1.5708,j1=1.5708,j2=1.5708,j3=1.5708)

  def linkch(self):
    with open ("choosen.txt") as g:
      groups = np.array([[x for x in line.split()] for line in g])
    #creating an array of the sherds on the table with (class, conf_lev, (x,y)_cent, link_name)
    with open ("posizioni_mm.txt") as pos:
        posizioni = np.array([[x for x in line.split()] for line in pos])
    link_choose = []
    #creating a list with the choosen link name
    for i in range(len(posizioni)):
        if (np.in1d(posizioni[i,0], groups)): 
            # print(posizioni[i]) 
            # link_choose.append(posizioni[i, 4])
            link_choose.append(posizioni[i, 0].astype(int), posizioni[i,4], np.where([groups==posizioni[i,0]])[1])
                
    self.link_choose = link_choose
    return self.link_choose
