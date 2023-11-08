#!/usr/bin/env python3

import sys
import rospy

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

def pol2cart(rho, phi):
    """helper,convert polar to cartesian"""
    x = rho*np.cos(phi)
    y = rho*np.sin(phi)
    return(x, y)

def load_calibrate():
    """load mapping points from gazebo to robot frame, estimate l and L, generate homography map"""
    try:
      with open('calibration.json', 'r') as f:
        calib = json.load(f)
      src_pts = calib['src_pts']
      dst_angs = calib['dst_angs']
      print(f'dst_angs are {dst_angs}')

      s_ret_pts = src_pts[1::2]
      s_ext_pts = src_pts[2::2]
      arr = np.array(s_ret_pts)-np.array(s_ext_pts)
      L = np.sqrt((arr*arr).sum(axis=1)).mean()/(np.cos(THETA_EXT)-np.cos(THETA_RET))
      print(f'L is {L}')
      arr = np.array(s_ret_pts)-np.array(src_pts[0])
      l1 = np.sqrt((arr*arr).sum(axis=1)).mean() - L*np.cos(THETA_RET)
      print(f'l1 is {l1}')
      arr = np.array(s_ext_pts)-np.array(src_pts[0])
      l2 = np.sqrt((arr*arr).sum(axis=1)).mean() - L*np.cos(THETA_EXT)
      print(f'l2 is {l2}')
      l = (l1+l2)/2
      print(f'l is {l}')

      dst_pts = [[0,0]]
      for i in range(len(dst_angs)):
        phi = dst_angs[i][0]
        print(f'phi is {phi}')
        rho = L*np.cos(dst_angs[i][1]) + l
        print(f'rho is {rho}')
        x, y = pol2cart(rho, phi)
        dst_pts.append([x,y])

      src_pts = np.array(src_pts)
      dst_pts = np.array(dst_pts)
      print(f'src_pts is \n \
            {src_pts}\n \
            and type is {type(src_pts)}\n \
            and shape is {src_pts.shape}')
      
      print(f'dst_pts is \n \
            {dst_pts}\n \
            and type is {type(dst_pts)}\n \
            and shape is {dst_pts.shape}')

      h, status = cv2.findHomography(src_pts, dst_pts)
    #   print(h)
      homography = h


      # kinematics = InvKin.Arm3Link(L=[L/2,L/2,l+L_FUDGE])
      print('calibration loaded.')
      print('estimated l = ' + str(l))
      print('estimated L = ' + str(L))
      cv2.destroyAllWindows()
    except:
      print('calibration.json not in current directory, run calibration first')

if __name__ == '__main__':
   load_calibrate()