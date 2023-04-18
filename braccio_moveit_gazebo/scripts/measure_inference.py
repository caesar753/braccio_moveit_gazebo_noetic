
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
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

# def transformation(self, image):
infer_transform = transforms.Compose([
    transforms.Resize((224,224)),
    # transforms.RandomHorizontalFlip(),
    transforms.ToTensor(), # ToTensor : [0, 255] -> [0, 1]
    transforms.Normalize(mean = [0.485, 0.456, 0.406],
                        std = [0.229, 0.224, 0.225])
])

vision_path = "../vision/"


class segmeasure():
    def __init__(self, img, ld_model, method = "bottom-to-top", wdt = 100):
        # super(segmeasure, self).__init__()
        self.img = img
        self.width = int(wdt)
        self.method = method
        self.ld_model = ld_model
        self.pippo = None
        self.topolino = None

    def midpoint(self, ptA, ptB):
        return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)
    
    def readimage(self):
        self.image = cv2.imread(self.img)
        # self.image = imutils.resize(self.image, width=640, height=480)

    def transimage(self):
        self.ROI_number = 0
        Contours_number = 0
        
        original = self.image.copy()
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        # gray = cv2.medianBlur(gray,5)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)
        thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        # Morph open to remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=15)
        # apply mask to image
        self.thresholded = cv2.bitwise_and(self.image, self.image, mask=closing)

        # perform edge detection, then perform a dilation + erosion to
        # close gaps in between object edges
        edged = cv2.Canny(gray, 50, 100)
        edged = cv2.dilate(edged, None, iterations=1)
        edged = cv2.erode(edged, None, iterations=1)
        # find contours in the edge map
        cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # sort the contours from left-to-right and initialize the
        # 'pixels per metric' calibration variable
        # method = str(args["method"])
        (cnts, _) = contours.sort_contours(cnts, self.method)
        self.cnts = cnts
        self.pixelsPerMetric = None

    
    #load model
    def load_model(self):
        model = torch.load(self.ld_model, map_location=torch.device('cpu'))
        # device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        if torch.cuda.is_available():
            model = model.cuda()
        #   model.to(device)
        model.eval()
        print('model is loaded')
        self.model = model
        return self.model

    def regint(self):
        # print(f'ROI is {self.ROI}')
#       #Save the fragment ROI image
        # print(f'ROI number is {self.ROI_number}')
        cv2.imwrite(os.path.join(vision_path, "segmentation/ROI_{}.png".format(self.ROI_number)), self.ROI)

    def return_pippo_topolino(self):
        # print(self.pippo, self.topolino)
        return self.pippo, self.topolino 

    def position(self):
        PositionPub.print_coord(self)                          
                

    def infer(self):
      
        #re-opens as a PIL image the fragment ROI image
        # infim = Image.open("segmentation/ROI_{self.ROI_number}.png".format(self.ROI_number), self.ROI)
        infim = Image.open(os.path.join(vision_path,"segmentation/ROI_{}.png".format(self.ROI_number)))
        # cv2.imshow('ROI', infim)

        #transforms the image into a tensor, unsqueezing dimension 0, and pases it to cuda()
        if torch.cuda.is_available():
            infim = infer_transform(infim).unsqueeze(0).cuda()
        else:
            infim = infer_transform(infim).unsqueeze(0)

        # #get the prediction from the loaded model
        output = self.model(infim)
        # print(output)

        #Only prediction
        # prediction = int(torch.max(output.data, 1)[1].cpu().numpy())

        #Prediction with confidence level
        probs = nn.functional.softmax(output, dim=1)
        # print(probs)
        confidence = (torch.max(probs.data, 1))[0].cpu().numpy()
        prediction = (torch.max(probs.data, 1))[1].cpu().numpy()
        print('The prediction is %d with a confidence level of %.2f %%' % (prediction, (100* confidence)))
        self.confidence = round(float(confidence), 2)
        self.prediction = int(prediction)
        return self.confidence, self.prediction

    def return_center_xy(self):
        # print(self.pippo, self.topolino)
        return self.pippo, self.topolino 

    def print_file(self):
        compl_file  = open("posizioni_completo.txt", "a")
        
        
    def position(self):
        PositionPub.print_coord(self)


class PositionPub():  
    # def __init__(self, seg): 
    #     # super().__init__(self, 'segmeasure')
    
    
    def print_coord(self, x, y):
        # print(f"Coordinates are {seg.pippo, seg.topolino}")
        print(f"Coordinates are {x, y}") 

        positions = open("posizioni.txt", "a")
        x_repr = repr(x)
        y_repr = repr(y)
        positions.write(x_repr + " ")# + y_repr + "\n")
        positions.write(y_repr)
        positions.write("\n") 
        positions.close
        
        
    def add_link(self, name, x, y):
        initial_pose = Pose()
        initial_pose.position.x = float(x)
        initial_pose.position.y = float(y)
        initial_pose.position.z = 0.0
        initial_pose.orientation.x = 0.0
        initial_pose.orientation.y = 0.0
        initial_pose.orientation.z = 0.55
        initial_pose.orientation.w = 1.0

        # f = open('/home/benb/Dropbox/RoboticsResearch/WAMInProgress/tp/AllInOne/Trajectory_Phonebook/ros_stuff/src/iiim_wam_description/wam.sdf','r')
        f = open('model.sdf', 'r')
        sdff = f.read()

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_model_prox(name, sdff, "", initial_pose, "")
        except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

