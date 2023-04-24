#!/usr/bin/env python3

import rospy
import unittest
import simplejson
import array
import pprint
import numpy as np
import cv2
import json
import PIL
import time
from PIL import Image
import torch
import torchvision.transforms as T
import matplotlib.pyplot as plt


from  std_msgs.msg                   import String
from  sensor_msgs.msg                import Imu,PointCloud2,CompressedImage
from  std_msgs.msg                   import Float64
from  visualization_msgs.msg         import MarkerArray
from  nav_msgs.msg                   import Odometry
from  sensor_msgs.msg                import CompressedImage,NavSatFix
from  sensor_msgs.msg                import BatteryState
from  sensor_msgs.msg                import Image
from  std_msgs.msg                   import String
import message_filters

import torch.nn as nn
import torch.nn.functional as F

from cv_bridge import CvBridge, CvBridgeError




global arrayflag 
global array
global done_flag_

arrayflag = True

global lidar_pub_
global radar_pub_
global image_pub_
global im
global count
global model
global train_on_gpu

#####----BUILD THE MODEL----#####
import torch.nn as nn
import torch.nn.functional as F

# define the CNN architecture
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        
        #ENCODING
       
        self.conv11 = nn.Conv2d(3, 64, 3, padding=1)
        self.BNEn11 = nn.BatchNorm2d(64,momentum=0.9)
      
        self.conv12 = nn.Conv2d(64, 64, 3, padding=1)
        self.BNEn12 = nn.BatchNorm2d(64,momentum=0.9)
        
      
        self.conv21 = nn.Conv2d(64, 128, kernel_size=3, padding=1)
        self.BNEn21 = nn.BatchNorm2d(128,momentum=0.9)
       
        self.conv22 = nn.Conv2d(128, 128, kernel_size=3, padding=1)
        self.BNEn22 = nn.BatchNorm2d(128,momentum=0.9)
        
       
        self.conv31 = nn.Conv2d(128, 256, kernel_size=3, padding=1)
        self.BNEn31 = nn.BatchNorm2d(256,momentum=0.9)
       
        self.conv32 = nn.Conv2d(256, 256, kernel_size=3, padding=1)
        self.BNEn32 = nn.BatchNorm2d(256,momentum=0.9)
        self.conv33 = nn.Conv2d(256, 256, kernel_size=3, padding=1)
        self.BNEn33 = nn.BatchNorm2d(256,momentum=0.9)

        self.conv41 = nn.Conv2d(256, 512, kernel_size=3, padding=1)
        self.BNEn41 = nn.BatchNorm2d(512,momentum=0.9)
     
        self.conv42 = nn.Conv2d(512, 512, kernel_size=3, padding=1)
        self.BNEn42 = nn.BatchNorm2d(512,momentum=0.9)
        self.conv43 = nn.Conv2d(512, 512, kernel_size=3, padding=1)
        self.BNEn43 = nn.BatchNorm2d(512,momentum=0.9)
   
        self.conv51 = nn.Conv2d(512, 512, kernel_size=3, padding=1)
        self.BNEn51 = nn.BatchNorm2d(512,momentum=0.9)
      
        self.conv52 = nn.Conv2d(512, 512, kernel_size=3, padding=1)
        self.BNEn52 = nn.BatchNorm2d(512,momentum=0.9)
        self.conv53 = nn.Conv2d(512, 512, kernel_size=3, padding=1)
        self.BNEn53 = nn.BatchNorm2d(512,momentum=0.9)
      
        self.fc1 = nn.Linear(98304, 2048) 
       
        self.fc2 = nn.Linear(2048,1024)
      
        self.fc3 = nn.Linear(1024, 1*512*8*8) 
        
        self.dropout = nn.Dropout(0.5)
    
        # max pooling layer
      
        self.pool      = nn.MaxPool2d(2, stride=2, return_indices=True)
       
        self.unpool    = nn.MaxUnpool2d(2, stride=2) 
        
        self.convdec53 = nn.Conv2d(512, 512, kernel_size=3, padding=1)
        self.BNDe53    = nn.BatchNorm2d(512,momentum=0.9)
        self.convdec52 = nn.Conv2d(512, 512, kernel_size=3, padding=1)
        self.BNDe52    = nn.BatchNorm2d(512,momentum=0.9)
        self.convdec51 = nn.Conv2d(512, 512, kernel_size=3, padding=1)
        self.BNDe51    = nn.BatchNorm2d(512,momentum=0.9)

        self.convdec43 = nn.Conv2d(512, 512, kernel_size=3, padding=1)
        self.BNDe43    = nn.BatchNorm2d(512,momentum=0.9)
        self.convdec42 = nn.Conv2d(512, 512, kernel_size=3, padding=1)
        self.BNDe42    = nn.BatchNorm2d(512,momentum=0.9)
        self.convdec41 = nn.Conv2d(512, 256, kernel_size=3, padding=1)
        self.BNDe41    = nn.BatchNorm2d(256,momentum=0.9)
        
        self.convdec33 = nn.Conv2d(256, 256, kernel_size=3, padding=1)
        self.BNDe33    = nn.BatchNorm2d(256,momentum=0.9)
        self.convdec32 = nn.Conv2d(256, 256, kernel_size=3, padding=1)
        self.BNDe32    = nn.BatchNorm2d(256,momentum=0.9)
        self.convdec31 = nn.Conv2d(256, 128, kernel_size=3, padding=1)
        self.BNDe31    = nn.BatchNorm2d(128,momentum=0.9)
        
        self.convdec22 = nn.Conv2d(128, 128, kernel_size=3, padding=1)
        self.BNDe22    = nn.BatchNorm2d(128,momentum=0.9)
        self.convdec21 = nn.Conv2d(128, 64, kernel_size=3, padding=1)
        self.BNDe21    = nn.BatchNorm2d(64,momentum=0.9)
        
        self.convdec12 = nn.Conv2d(64, 64, kernel_size=3, padding=1)
        self.BNDe12    = nn.BatchNorm2d(64,momentum=0.9)
        self.convdec11 = nn.Conv2d(64, 3, kernel_size=3, padding=1)
        self.BNDe11    = nn.BatchNorm2d(3,momentum=0.9)
        
        
        
    def forward(self, x,y,z):
    
        x = F.relu(self.BNEn11(self.conv11(x)))
        x = F.relu(self.BNEn12(self.conv12(x))) 
        x, indx1 = self.pool(x) 
      
        
        y = F.relu(self.BNEn11(self.conv11(y)))
        y = F.relu(self.BNEn12(self.conv12(y))) 
        y, indy1 = self.pool(y)
       
        z = F.relu(self.BNEn11(self.conv11(z)))
        z = F.relu(self.BNEn12(self.conv12(z))) 
        z, indz1 = self.pool(z)
    
        x = F.relu(self.BNEn21(self.conv21(x)))
        x = F.relu(self.BNEn22(self.conv22(x)))
        x, indx2 = self.pool(x)
       
        y = F.relu(self.BNEn21(self.conv21(y)))
        y = F.relu(self.BNEn22(self.conv22(y)))
        y, indy2 = self.pool(y)
        
        z = F.relu(self.BNEn21(self.conv21(z)))
        z = F.relu(self.BNEn22(self.conv22(z)))
        z, indz2 = self.pool(z)
       
        x = F.relu(self.BNEn31(self.conv31(x)))
     
        x = F.relu(self.BNEn32(self.conv32(x)))
        x = F.relu(self.BNEn33(self.conv33(x)))
        x, indx3 = self.pool(x)
        
        y = F.relu(self.BNEn31(self.conv31(y)))
        y = F.relu(self.BNEn32(self.conv32(y)))
        y = F.relu(self.BNEn33(self.conv33(y)))
        y, indy3 = self.pool(y)
        
        z = F.relu(self.BNEn31(self.conv31(z)))
        z = F.relu(self.BNEn32(self.conv32(z)))
        z = F.relu(self.BNEn33(self.conv33(z)))
        z, indz3 = self.pool(z)
            
        x = F.relu(self.BNEn41(self.conv41(x)))
        x = F.relu(self.BNEn42(self.conv42(x)))
        x = F.relu(self.BNEn43(self.conv43(x)))
        x, indx4 = self.pool(x)
       
        
        y = F.relu(self.BNEn41(self.conv41(y)))
        y = F.relu(self.BNEn42(self.conv42(y)))
        y = F.relu(self.BNEn43(self.conv43(y)))
        y, indy4 = self.pool(y)
        
        z = F.relu(self.BNEn41(self.conv41(z)))
        z = F.relu(self.BNEn42(self.conv42(z)))
        z = F.relu(self.BNEn43(self.conv43(z)))
        z, indz4 = self.pool(z)
        
   
        x = F.relu(self.BNEn51(self.conv51(x)))
        x = F.relu(self.BNEn52(self.conv52(x)))
        x = F.relu(self.BNEn53(self.conv53(x)))
        x, indx5 = self.pool(x)
        
        
        y = F.relu(self.BNEn51(self.conv51(y)))
        y = F.relu(self.BNEn52(self.conv52(y)))
        y = F.relu(self.BNEn53(self.conv53(y)))
        y, indy5 = self.pool(y)
      
        z = F.relu(self.BNEn51(self.conv51(z)))
        z = F.relu(self.BNEn52(self.conv52(z)))
        z = F.relu(self.BNEn53(self.conv53(z)))
        z, indz5 = self.pool(z)
        
   
        xyz=torch.cat((x,y,z),0)
        
        enc=x
        
        x = xyz.view(-1,98304 )
       
        x = self.dropout(x)
        x = F.relu(self.fc1(x))
        x = self.dropout(x)
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        fc=x
        
      
        x= torch.reshape(x, (1,512, 8, 8))  ##batch size 1
        
        x = self.unpool(x,indx5)
        x = F.relu(self.BNDe53(self.convdec53(x)))
        x = F.relu(self.BNDe52(self.convdec52(x))) 
        x = F.relu(self.BNDe51(self.convdec51(x))) 
        
      
        x = self.unpool(x,indx4)
        x = F.relu(self.BNDe43(self.convdec43(x))) 
        x = F.relu(self.BNDe42(self.convdec42(x))) 
        x = F.relu(self.BNDe41(self.convdec41(x))) 
       
        x = self.unpool(x,indx3)
        x = F.relu(self.BNDe33(self.convdec33(x)))
        x = F.relu(self.BNDe32(self.convdec32(x))) 
        x = F.relu(self.BNDe31(self.convdec31(x))) 
       
        x = self.unpool(x,indx2)
        x = F.relu(self.BNDe22(self.convdec22(x))) 
        x = F.relu(self.BNDe21(self.convdec21(x))) 
       
        x = self.unpool(x,indx1)
        x = F.relu(self.BNDe12(self.convdec12(x))) 
        x = self.convdec11(x) 
        dec=x
        
        return enc, fc, dec
        
      
        
def set_device():

# check if CUDA is available
   if torch.cuda.is_available():
        dev = "cuda:0"
        
   else:
        dev = "cpu"
   return torch.device(dev)  

        

   
def callback(lidar_, radar_, image_):
    global lidar_pub_
    global radar_pub_
    global image_pub_
    global im
    global count
    global model
    global train_on_gpu
  
    model.train()
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_, "bgr8")
    cv_lidar = bridge.imgmsg_to_cv2(lidar_, "bgr8")
    cv_radar = bridge.imgmsg_to_cv2(radar_, "bgr8")
 
    transform = T.ToTensor()

    image_tensor = transform(cv_image)
    radar_tensor = transform(cv_radar)
    lidar_tensor = transform(cv_lidar)
 
    image_resize = T.Resize(size = (256,256))
    
    img_ = image_resize(image_tensor)
    lid_ = image_resize(lidar_tensor)
    rad_ = image_resize(radar_tensor)
  
    img_=torch.unsqueeze(img_, dim=0)
    lid_=torch.unsqueeze(lid_, dim=0)
    rad_=torch.unsqueeze(rad_, dim=0)
   
    dev = set_device()
    
    print("------dev-----:  ",dev)
    
    
       
    img_=img_.cuda()
    lid_=lid_.cuda()   
    rad_=rad_.cuda()
   
    model.cuda()
  
    st = time.time()
    outputdec_test,outputfc_test,output_test = model(img_,lid_,rad_)
 
    et = time.time()
 
    print('Execution time:', elapsed_time, 'seconds')
    
    
    print("----------------output shape type ---------", output_test.shape)
    output_test = eliminate_false_positives(output_test)
  
    img = output_test[0,:,:,:]
    
   
    img = img.swapaxes(0,2)
    img = img.swapaxes(0,1)
 
    img  =img.cpu().detach().numpy()
 
    im.set_data(img)
    
   
    
    bridge = CvBridge()
  
    image_scaled = (img / img.max() * 255.0).astype(np.uint8)
    ros_image = bridge.cv2_to_imgmsg(image_scaled, encoding="bgr8")
    
    
    
    
  
    
    
    cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")

    plt.draw()
    plt.pause(0.01)
  
    print("-----NODE---------", count)
    count+=1	
   
    
def color_false_positives(input_image, output, background_color=(0, 0, 0)):
    # Convert the input image to a numpy array
    input_image = input_image.detach().cpu().numpy().transpose((1, 2, 0))
    # Convert the output to a numpy array
    output = output.detach().cpu().numpy()
    # Create a binary mask from the output
    mask = (output == 0).astype(int)
    # Repeat the background color for each channel
    background = np.array([background_color] * input_image.shape[2]).transpose((1, 2, 0))
    # Apply the mask to the input image
    masked_image = np.where(np.repeat(mask[:, :, np.newaxis], 3, axis=2), background, input_image)
    return masked_image   
    
def eliminate_false_positives(output, threshold=0.3):
    # Convert the output to a numpy array
    output = output.detach().cpu().numpy()
    # Threshold the output probabilities to eliminate false positives
    output = (output > threshold).astype(int)
    # Convert the output back to a torch tensor
    output = torch.from_numpy(output).float()
    return output   

    
    
def array_to_image(arr):
    arr = np.clip(arr, 0, 255).astype(np.uint8)
    if arr.ndim == 2:
        return Image.fromarray(arr, mode='L')
    elif arr.ndim == 3:
        return Image.fromarray(arr, mode='RGB')
    else:
        raise ValueError("Unsupported array shape: {}".format(arr.shape))    

def tensor_to_image_msg(tensor, encoding='bgr8'):
   
    
   # Convert the PyTorch tensor to a NumPy array
   img = tensor.cpu().detach().numpy()
   #print("tensor ",type(img))

   # Convert the NumPy array to a OpenCV image
   img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
   # Normalize the image values to be in the range of 0-255
   img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC3)
   
   
   bridge = CvBridge()
   try:
      ros_img = bridge.cv2_to_imgmsg(img, "bgr8")
      return ros_img
   except CvBridgeError as e:
 
   
   # Create a ROS Image message
   image_msg = Image()
   image_msg.header.stamp = rospy.Time.now()
   image_msg.encoding = encoding
   image_msg.width = img.shape[1]
   image_msg.height = img.shape[0]
   image_msg.step = img.shape[1] * img.shape[2]
   image_msg.data = img.tostring()

   return image_msg
   

def main(): 
   rospy.init_node('segnet_nlab', anonymous=True)
   global lidar_pub_
   global radar_pub_
   global image_pub_
   global im
   global count	
   global model
   global train_on_gpu
   
   model = Net()
   print("torch:----", torch.__version__)
  
   model.load_state_dict(torch.load('path_to_/SegNet/model151_nint_last.pt'))
   
   count=0
  
   image_pub_ = rospy.Publisher('/image_segnet', Image, queue_size=10)
   
   
   lidar_sub = message_filters.Subscriber('/lidar_interpolated16', Image)
   radar_sub = message_filters.Subscriber('/radar_interpolated16', Image)
   image_sub = message_filters.Subscriber('/image_simulation', Image)
   
   ts = message_filters.ApproximateTimeSynchronizer([lidar_sub, radar_sub,image_sub], 10,10,10)
   ts.registerCallback(callback)	
 
   fig = plt.figure()
   im = plt.imshow(np.zeros((256,512, 3)))
  
   plt.show()
   
   
   rate=rospy.Rate(10)
   rospy.spin()


if __name__ == '__main__':
   global done_flag_ 
   global model
   done_flag_ = True
   main()

