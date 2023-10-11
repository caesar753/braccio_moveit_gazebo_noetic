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

# import InvKin #as Arm3Link
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


class SherdCreation(object):
  """BraccioXYBBTargetInterface"""
  def __init__(self):
    super(SherdCreation, self).__init__()

    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('braccio_xy_bb_target', anonymous=True)

    # group_name = "braccio_arm"
    # self.move_group = moveit_commander.MoveGroupCommander(group_name)
    # self.gripper_group = moveit_commander.MoveGroupCommander("braccio_gripper")

    # self.homography = None

    # self.kinematics = InvKin.Arm3Link()
    self.sherd_pub = rospy.Publisher("/sherd_pub", )
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
  
  # def go_link_choose(self, lk):
  #     self.link_choose = lk
  #     print(lk)
  #     print(self.link_choose)
  
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
