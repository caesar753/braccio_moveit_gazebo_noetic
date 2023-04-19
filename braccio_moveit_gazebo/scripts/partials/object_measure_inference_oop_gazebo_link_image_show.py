#!/usr/bin/env python3

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

class segmeasure():
    def __init__(self, img, ld_model, method = "bottom-to-top", wdt = 100):
        # super(segmeasure, self).__init__()
        self.img = img
        self.width = int(wdt)
        self.method = method
        self.ld_model = ld_model
        self.pippo = None
        self.topolino = None
        self.prediction = 0
        self.confidence = 0.0
        self.sherd_name = None

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
        cv2.imwrite("segmentation/ROI_{}.png".format(self.ROI_number), self.ROI)

    def return_pippo_topolino(self):
        # print(self.pippo, self.topolino)
        return self.pippo, self.topolino 

    def position(self):
        PositionPub.print_coord(self)                          
                

    def infer(self):
      
        #re-opens as a PIL image the fragment ROI image
        # infim = Image.open("segmentation/ROI_{self.ROI_number}.png".format(self.ROI_number), self.ROI)
        infim = Image.open("segmentation/ROI_{}.png".format(self.ROI_number))
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

    def image_show(self, orig, box, dimA, dimB, tltrX, tltrY, trbrX, trbrY, blbrX, blbrY,\
                   tlblX, tlblY, blX, blY, brX, brY,  nome):
        
        cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)

        # loop over the original points and draw them
        for (x, y) in box:
            cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)

        # draw the object sizes on the image
        cv2.putText(orig, "{:.1f}mm".format(dimA),
            (int(tltrX - 15), int(tltrY - 10)), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (255, 255, 255), 2)
        cv2.putText(orig, "{:.1f}mm".format(dimB),
            (int(trbrX + 10), int(trbrY)), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (255, 255, 255), 2)
        
        # draw the midpoints on the image
        cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)
        # draw lines between the midpoints
        cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),
            (255, 0, 255), 2)
        cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),
            (255, 0, 255), 2)
        cv2.putText(orig, "class: {:d}".format(self.prediction),
            (int(blX - 35), int(blY + 25)), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (255, 255, 255), 2)
        cv2.putText(orig, "conf_lev: {:.2f}".format(self.confidence),
            (int(brX - 25), int(brY + 25)), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (255, 255, 255), 2)
        cv2.putText(orig, "{:s}".format(nome),
            (int(blbrX - 25), int(blbrY + 50)), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (255, 255, 255), 2)

        #   # show the output image
        orig = cv2.resize(orig, (800,600))
        cv2.imshow("Image", orig)
        cv2.waitKey(0)


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


def main():
    print("choose image \n")
    ch_img = input()

    print("choose method")
    meth = input()

    print("choose sample width")
    wdt = input()

    print("which model?")
    md = input()

    print("Do you want to show the image?")
    im_ch = input()

    segmentation = segmeasure(ch_img, md, meth , wdt)
    RosPub = PositionPub()
    
    segmentation.load_model()
    segmentation.readimage()
    segmentation.transimage()

    n = 0
    if os.path.isfile("posizioni_mm.txt"): os.remove("posizioni_mm.txt")
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
        segmentation.pippo = (tlbrX, tlbrY)[0]
        segmentation.topolino = (tlbrX, tlbrY)[1]

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

        segmentation.return_center_xy()
        
        RosPub.print_coord(segmentation.pippo, segmentation.topolino)
        
        if n > 0:
            segmentation.infer()
        
            # distance of center of sherd from image origin
            centX = segmentation.pippo / segmentation.pixelsPerMetric
            centY = segmentation.topolino / segmentation.pixelsPerMetric
            print(f"Center of sherds from origin in mm is {centX}, {centY}")
            position_mm = open("posizioni_mm.txt", "a")
            conf = repr(segmentation.confidence)
            lab = repr(segmentation.prediction)
            x_mm = repr(round(centX,2))
            y_mm = repr(round(centY,2))
            nome = ("sherd_" + str(n))
            # nome = ("link_" + str(n))
            # position_mm.write(lab + " " + conf + " " + nome + " " + x_mm + " " + y_mm + "\n")
            position_mm.write(lab + " " + conf + " " + x_mm + " " + y_mm + " " + nome + "\n")
            position_mm.close()
            
            # segmentation.add_link(x_mm, y_mm)
            
            RosPub.add_link(nome, (centX/1000), (centY/1000))
            
            if im_ch == "y":
                segmentation.image_show(orig, box, dimA, dimB, tltrX, tltrY, trbrX, trbrY, blbrX, blbrY,\
                    tlblX, tlblY, blX, blY, brX, brY,  nome)
        
    

    #   # show the output image
        # cv2.imshow("Image", orig)
        # cv2.waitKey(0)

        segmentation.ROI_number += 1
        n += 1

    
    # RosPub = PositionPub(segmentation)
    # RosPub.print_coord()
    # RosPub.publisher()



if __name__ == '__main__':
  main()
