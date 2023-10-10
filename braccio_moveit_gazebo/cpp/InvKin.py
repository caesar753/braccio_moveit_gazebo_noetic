#!/usr/bin/env python3
import sys
import time
import os
import  numpy as np
import scipy


class Arm3Link:
    """
    A simple inverse kinematics solver for a should-elbow-wrist robot arm
    credit: https://github.com/studywolf/blog/tree/master/InvKin
    """
    def __init__(self, L=None):
        # initial joint angles
        self.q = [0, 0, 0]
        # some default arm positions
        self.L = np.array([1, 1, 0.8]) if L is None else L
        self.max_y = 1
        self.min_y = 0

        self.end_angle_tol = 0.05
        self.end_angle = -np.pi/2
        self.max_angles = [1.6, np.pi/2, np.pi/2]
        self.min_angles = [0.27, -np.pi/2, -np.pi/2]

    def get_xy(self, q=None):
        if q is None:
            q = self.q

        x = self.L[0]*np.cos(q[0]) + \
            self.L[1]*np.cos(q[0]+q[1]) + \
            self.L[2]*np.cos(np.sum(q))

        y = self.L[0]*np.sin(q[0]) + \
            self.L[1]*np.sin(q[0]+q[1]) + \
            self.L[2]*np.sin(np.sum(q))

        return [x, y]

    def inv_kin(self, x, min_y, max_y, end_angle):

        def distance_to_default(q, x):
            x = (self.L[0]*np.cos(q[0]) + self.L[1]*np.cos(q[0]+q[1]) +
                 self.L[2]*np.cos(np.sum(q))) - x
            return x**2

        def y_upper_constraint(q, *args):
            y = (self.L[0]*np.sin(q[0]) + self.L[1]*np.sin(q[0]+q[1]) +
                 self.L[2]*np.sin(np.sum(q)))
            return self.max_y - y

        def y_lower_constraint(q, *args):
            y = (self.L[0]*np.sin(q[0]) + self.L[1]*np.sin(q[0]+q[1]) +
                 self.L[2]*np.sin(np.sum(q)))
            return y - self.min_y

        def joint_limits_upper_constraint(q, *args):
            return self.max_angles - q

        def joint_limits_lower_constraint(q, *args):
            return q - self.min_angles

        def joint_limits_last_orientation(q, *args):
            return self.end_angle_tol - np.abs(np.sum(q)-self.end_angle)

        self.min_y = min_y
        self.max_y = max_y
        if end_angle is not None:
            self.end_angle = end_angle
        q = scipy.optimize.fmin_slsqp(func=distance_to_default, x0=self.q, args=(x,), iprint=0,
                                      ieqcons=[joint_limits_last_orientation,
                                               joint_limits_upper_constraint,
                                               joint_limits_lower_constraint,
                                               y_upper_constraint,
                                               y_lower_constraint])
        self.q = q
        return self.q