#!/usr/bin/env python3

import numpy as np

import auto_targetter
import measure_inference
import image_listener
import vision_utils
from custom_msgs.msg import target, matrix

import open3d as o3d

from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
# import argparse
import imutils
import cv2
import os


import torch
import torch.nn as nn
# from torch.utils.data import DataLoader
# import torch.optim as optim
# from torch.optim import lr_scheduler
# import torch.backends.cudnn as cudnn


from torchvision import models
# import torchvision.utils
# import torchvision.datasets as dsets
import torchvision.transforms as transforms

from PIL import Image


import rospy
from gazebo_msgs.msg import LinkState, ModelState
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SpawnModel

import time

# auto_targetter = auto_targetter.BraccioObjectTargetInterface()

# auto_targetter.load_calibrate()

RosPub = measure_inference.PositionPub()
camera = image_listener.CameraShooter()
vision_path = "../vision/"

def main():
    rospy.init_node('auto_main')

    print("do you want to use [c]amera or [i]mage?")
    initial_choose = input()
    
    if initial_choose == "i":
        print("choose image \n")
        ch_img = input()
    elif initial_choose == "c":
        camera.shoot()
        ch_img = "camera_image.jpg"
    else:
        print("No valid choice")
        return 1


    print("which model?")
    md = input()

    print ("Do you want to show the image during sherd creation?")
    im_ch =input()
    
    segmentation = measure_inference.segmeasure(ch_img, md)

    segmentation.load_model()
    segmentation.readimage()
    segmentation.transimage()

    n = 0
    position_file = os.path.join(vision_path, "posizioni_mm.txt")
    
    if os.path.isfile(position_file): 
        os.remove(position_file)
    
    for c in segmentation.cnts:
        # if the contour is not sufficiently large, ignore it
        if cv2.contourArea(c) < 100:
            continue

        # compute the rotated bounding box of the contour
        orig = segmentation.thresholded.copy()
        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")
        # print(f'BoxPoint are {box}')
        # print(f'BoxPoints for the x are {box[:,0]}')
        # print(f'BoxPoints for the x are {box[:,1]}')
        
        #Store extreme coordinates of the box
        x_min, y_min = np.amin(box[:,0]), np.amin(box[:,1])
        x_max, y_max = np.amax(box[:,0]), np.amax(box[:,1])
        # print(f'x_min is {x_min}, x_max is {x_max}')
        # print(f'y_min is {y_min}, y_max is {y_max}')

        box = perspective.order_points(box)

        # unpack the ordered bounding box, then compute the midpoint
        # between the top-left and top-right coordinates, followed by
        # the midpoint between bottom-left and bottom-right coordinates
        (tl, tr, br, bl) = box
        (tlX, tlY) = tl
        (trX, trY) = tr
        (brX, brY) = br
        (blX, blY) = bl
        # print((tl, tr, br, bl))
        (tltrX, tltrY) = segmentation.midpoint(tl, tr)
        (blbrX, blbrY) = segmentation.midpoint(bl, br)
        # compute the midpoint between the top-left and top-right points,
        # followed by the midpoint between the top-righ and bottom-right
        (tlblX, tlblY) = segmentation.midpoint(tl, bl)
        (trbrX, trbrY) = segmentation.midpoint(tr, br)
        #compute the midpoint between diagonal ((top-left, bottom-right), (bottom-left, top-right))
        (tlbrX, tlbrY) = segmentation.midpoint(tl, br)
        print(f'Center of sherd is {(tlbrX, tlbrY)}')
        segmentation.x_center = (tlbrX, tlbrY)[0]
        segmentation.y_center = (tlbrX, tlbrY)[1]

        # compute the Euclidean distance between the midpoints
        dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
        dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
        
        # if the pixels per metric has not been initialized, then
        # compute it as the ratio of pixels to supplied metric
        # (in this case, mm)
        if segmentation.pixelsPerMetric is None:
            segmentation.pixelsPerMetric = dB / segmentation.width

        # compute the size of the object
        dimA = dA / segmentation.pixelsPerMetric
        dimB = dB / segmentation.pixelsPerMetric
        
        # n = n+1
        # if n > 0: #n == 0 is the measure specimen, no extraction and no prediction
        #uses the extreme coordinate of the box to extract the fragment as a ROI (Region Of Interest)
        segmentation.ROI = segmentation.thresholded[y_min:y_max, x_min:x_max]
        
        segmentation.regint()

        # RosPub.print_coord(segmentation.x_center, segmentation.y_center)
    

        if n > 0:
            segmentation.infer()
        
            # distance of center of sherd from image origin
            centX = segmentation.x_center / segmentation.pixelsPerMetric
            centY = segmentation.y_center / segmentation.pixelsPerMetric
            print(f"Center of sherds from origin in mm is {centX}, {centY}")
            position_mm = open(position_file, "a")
            conf = repr(segmentation.confidence)
            lab = repr(segmentation.prediction)
            class_sherd = ("class" + lab)
            x_mm = repr(round(centX,2))
            y_mm = repr(round(centY,2))
            nome = ("sherd_" + str(n))
            position_mm.write(lab + " " + conf + " " + x_mm + " " + y_mm + " " + nome + "\n")
            position_mm.close()
            
            # segmentation.add_link(x_mm, y_mm)
            
            segmentation.model_creation(dimA, dimB, n, class_sherd)
            
            RosPub.add_link(nome, n, (centX/1000), (centY/1000))
            
            if im_ch == "y":
                segmentation.image_show(orig, box, dimA, dimB, tltrX, tltrY,\
                     trbrX, trbrY, blbrX, blbrY, tlblX, tlblY, blX, blY, brX, brY,  nome)        

        segmentation.ROI_number += 1
        n += 1
    
    with open(position_file) as f:
        array = np.array([[x for x in line.split()] for line in f])

    # print(array) 

    array = array[array[:, 0].astype(int).argsort()]
    print(f'sorted is \n \
        {array}')

    groups = np.split(array[:, 0:], np.cumsum(np.unique(array[:, 0].astype(int), return_counts=True)[1])[:-1])
    group_stats = np.array([(g[0, 0], len(g), np.mean(g[:, 1].astype(float)).round(2), np.std(g[:, 1].astype(float)).round(2)) for g in groups])
    stat_data = np.array2string(group_stats, suppress_small = True)
    stat_data = stat_data.replace('[','').replace(']','').replace('\'','')   
    
    stat_file = os.path.join(vision_path, "groups.txt")
    # stat = open(os.path.join(vision_path, "groups.txt"), "w+")
    stat = open(stat_file, "w+")
    # stat_data = np.array2string(group_stats, suppress_small = True)
    stat.write(stat_data)
    stat.close()

    with open(stat_file) as s:
        stat_array = np.array([[x for x in line.split()] for line in s])
        
    print(stat_array)

    variables = np.zeros((len(group_stats),2))
    n = 0

    for i in range(len(group_stats)):
        if group_stats[i,2].astype(float) > 0.30 and group_stats[i,3].astype(float) < 0.25: 
            variables[n,0] = group_stats[i,0]
            variables[n,1] = group_stats[i,1]
            # print(variables[n])
            n += 1

    variables = variables.astype(int)     
    # print(variables)
    variables = variables[variables[:,1].argsort()[::-1]]
    # print(variables)
    choosen = variables[:3]
    choosen = np.array2string(choosen)
    choosen = choosen.replace('[','').replace(']','').replace('\'','')   
    print(choosen)

    
    choose = open (os.path.join(vision_path,"choosen.txt"), "w+")
    choose.write (choosen)
    choose.close()

    #creating an array of the sherds on the table with (class, conf_lev, (x,y)_cent, link_name)
    with open (position_file) as pos:
        posizioni = np.array([[x for x in line.split()] for line in pos])
        print(posizioni)

    choosen_file = os.path.join(vision_path, "choosen.txt")
    # os.chdir(vision_path)
    # with open (os.path.join(vision_path, "cazzo.txt")) as g:
    with open (choosen_file) as g:
        groups = np.array([[x for x in line.split()] for line in g])
        print(groups)

    link_choose = []

    #creating a list with the choosen link name
    for i in range(len(posizioni)):
        if (np.in1d(posizioni[i,0], groups)): 
            lk = (posizioni[i, 0].astype(int), posizioni[i,4], np.array2string(np.where([groups==posizioni[i,0]])[1]))
            print(lk)
            # print(np.where([groups==posizioni[i,0]])[0].astype(int))
            link_choose.append(lk)
                
    # converting the list in a numpy array
    link_array = np.array(link_choose)
    print(f"selected are {np.array2string(link_array)}")

    ###POINTCLOUD SEGMENTATION - START ###
    # print("starting pointcloud segmentation")
    # debug = True
    # pcd = vision_utils.get_point_cloud_from_ros(debug)

    # print ('Table Segmentation')
    # table_cloud, object_cloud = vision_utils.segment_table(pcd)

    # voxel_pc = object_cloud.voxel_down_sample(voxel_size=0.001)

    # object_cloud, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.005)
    # object_cloud.paint_uniform_color([0, 1, 0])
    # table_cloud.paint_uniform_color([1, 0, 0])

    # if debug:
    #     o3d.visualization.draw_geometries([table_cloud, object_cloud])
    ###POINTCLOUD SEGMENTATION - END ###

    #HERE WE USE CUSTOM TARGET AND MATRIX MESSAGES
    # List with the target messages
    target_msg_arr = []
    for j in range(len(link_array)):
        #Generating the target message, fields are: nr[int], sherd[str], home[str]
        target_msg = target()
        target_msg.nr = j
        inp_ch = link_array[j,1].astype(str) + "::link"
        print(inp_ch)
        target_msg.sherd = inp_ch
        bowl_ch = link_array[j,2].astype(str)
        bowl_ch = bowl_ch.replace('\'','').replace('\'','')
        bowl_ch = bowl_ch.replace('[','').replace(']','')
        bowl_ch = "go_to_home_" + bowl_ch
        print(bowl_ch)
        target_msg.home = bowl_ch
        #Appending the target message to the list
        target_msg_arr.append(target_msg)
    
    # Converting the target_msg_arr list in a (custom) matrix_msg
    matrix_msg = matrix()
    matrix_msg.targets = target_msg_arr
    print(matrix_msg)

    rate = rospy.Rate(10)

    # creating the target_pub publisher, topic is /targets, message type is matrix
    target_pub = rospy.Publisher('/targets', matrix, queue_size=10)

    # Publishing the matrix message with the targets
    while not rospy.is_shutdown():
        target_pub.publish(matrix_msg)
        # rospy.spin()
        rate.sleep()
        

if __name__ == "__main__":
  main()