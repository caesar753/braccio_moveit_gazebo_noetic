#!/usr/bin/env python3

import rospy
from custom_msgs.msg import matrix
import auto_targetter

if __name__ == '__main__':
    
    targetter = auto_targetter.BraccioObjectTargetInterface()

    targetter.load_calibrate()

    rate = rospy.Rate(100)

    for j in range(targetter.i+1):
        # targetter.get_link_choose(targetter.targets_list[j].sherd)
        targetter.go_to_target('top', targetter.targets_list[j].home, targetter.targets_list[j].sherd)
