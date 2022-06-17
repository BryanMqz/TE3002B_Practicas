#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
font = cv2.FONT_HERSHEY_SIMPLEX

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

    self.imu = Imu()
    self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_cb)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    #Once we read the image we need to change the color space to HSV 
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) 
    #Hsv limits are defined 
    #here is where you define the range of the color you're looking for 
    #each value of the vector corresponds to the H,S & V values respectively
    # https://cvexplained.wordpress.com/2020/04/28/color-detection-hsv/
    # lower boundary RED color range values; Hue (0 - 10)
    min_red1 = np.array([0, 100, 20])
    max_red1 = np.array([10, 255, 255])
 
    # upper boundary RED color range values; Hue (160 - 180)
    min_red2 = np.array([160,100,20])
    max_red2 = np.array([179,255,255])
    #This is the actual color detection  
    #Here we will create a mask that contains only the colors defined in your limits 
    #This mask has only one dimension, so its black and white } 
    red1_mask = cv2.inRange(hsv, min_red1, max_red1)
    red2_mask = cv2.inRange(hsv, min_red2, max_red2)
    #We use the mask with the original image to get the colored post-processed image
    full_mask = red1_mask + red2_mask
    result = cv2.bitwise_and(cv_image, cv_image, mask=full_mask)
    
    # Use putText() method for inserting text on video
    cv2.putText(result, self.orientation_, (50, 50), font, 1, (0, 255, 255), 2, cv2.LINE_4)
    cv2.putText(result, self.acceleration_, (50, 80), font, 1, (0, 255, 0), 2, cv2.LINE_4)
    
    cv2.imshow("Image window", cv_image)
    #cv2.imshow("Mask", full_mask)
    cv2.imshow("Red Filtered", result)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  def imu_cb(self,data):
    imu = data
    
    self.orientation_ = "Orientation: " + \
    str(round(imu.orientation.x,2)) + " " +\
    str(round(imu.orientation.y,2)) + " " +\
    str(round(imu.orientation.z,2))
    self.acceleration_ = "Acceleration: " + \
    str(round(imu.linear_acceleration.x,2)) + " " +\
    str(round(imu.linear_acceleration.y,2)) + " " +\
    str(round(imu.linear_acceleration.z,2))

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)