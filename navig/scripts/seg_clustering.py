#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import csv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

names = {}
key = list()
value = list()

def eq(a, b):
  if a[0]==b[0] and a[1]==b[1] and a[2]==b[2]:
    return True
  else: return False

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    with open("/home/cair/sim_ws/src/navig/config/seg_class.csv", mode='r') as csv_file:
      csv_reader = csv.reader(csv_file)
      next(csv_reader)
      for row in csv_reader:
        names[row[0]] = [int(row[3]), int(row[2]), int(row[1])]
        #names[row[0]] = [int(x) for x in row[1:4]].reverse()]
    print(names)
    self.key = list(names.keys())
    self.value = list(names.values())
    self.index = []
    self.mask_img = []
    self.image_sub = rospy.Subscriber("/segmented",Image,self.callback)
    self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depthCB)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    img = np.array(cv_image)
    print(img.shape)
    bgr_val, index, counts = np.unique(img.reshape(-1, img.shape[2]), axis=0, return_index = True, return_counts = True)
    #print(list(bgr_val[0]), self.value[0])
    class_val = [self.key[self.value.index(list(val))] for val in bgr_val]
    #for i in bgr_val:
      #mask = [np.array([0, 0, 0]) for row in range(img.shape[0]) for col in range(img.shape[1]) if not eq(img[row][col],i)]
      #self.mask_img.append(mask)
    print(bgr_val)
    print(class_val)
    #index = 0
    #self.index = np.array([(np.unique(i[0], axis=0), np.unique(i[1], axis=0)) for i in np.delete(np.array([np.where(img == x) for x in bgr_val]), 2, 1)])
    #print(self.index.shape)

  def depthCB(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
      print(e)

    img = np.array(cv_image)
    cv2.imshow("Image window", img)

    cv2.waitKey(3)




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