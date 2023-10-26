#!/usr/bin/env python3

import numpy as np
import rospy

import auto_targetter


# def read_target():
#     target_file = open("targets.txt",'r')
#     targets = target_file.read()
#     target_file.close()

#     return targets

auto_targetter = auto_targetter.BraccioObjectTargetInterface()

auto_targetter.load_calibrate()


if __name__ == '__main__':


    target_file = "targets.txt"
    with open (target_file) as t:
        tar_arr = np.array([[x for x in line.split()] for line in t])
        print(tar_arr)

    for j in range(len(tar_arr)):
        inp_ch = tar_arr[j,0].astype(str)# + "::link"
        inp_ch = inp_ch.replace('\'','').replace('\'','')
        print(inp_ch)
        auto_targetter.get_link_choose(inp_ch)
        bowl_ch = tar_arr[j,1]
        bowl_ch = bowl_ch.replace('\'','').replace('\'','')
        print(bowl_ch)
        print(bowl_ch)
        auto_targetter.go_to_target('top', bowl_ch)
