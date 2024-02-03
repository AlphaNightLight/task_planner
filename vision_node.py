#!/usr/bin/env python

import rospy as ros

from ur5_lego.srv import GetBoundingBoxes,GetBoundingBoxesRequest,GetBoundingBoxesResponse
from ur5_lego.msg import BoundingBox
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import os
#import matplotlib.pyplot as plt


cwd = os.getcwd()
os.chdir("/home/alex/ros_ws/src/ur5_lego/src/mega_blocks_detector_project/")
from ur5_lego_modules.bosco_code import bb, make_prediction
os.chdir(cwd)
#print(ur5_lego_modules.bosco_code.__file__)



info_name = "  [  vision_node   ]:"
debug_mode = False



def get_bounding_boxes_handler(req):
  ros.loginfo("%s Detecting bounding boxes...", info_name)
  res = GetBoundingBoxesResponse()

  ### Detect the bounding boxes
  img_raw = ros.wait_for_message("/ur5/zed_node/left/image_rect_color", Image)
  img_cv2 = CvBridge().imgmsg_to_cv2(img_raw, 'rgb8')

  #plt.imshow(img_cv2)
  #plt.show()
  #cv2.imwrite('/home/alex/ros_ws/src/ur5_lego/src/task_planner/zed.png', img_cv2)

  predictions = make_prediction(img_cv2)

  for prediction in predictions:
    bounding_box = BoundingBox()
    bounding_box.xc = prediction.xc
    bounding_box.yc = prediction.yc
    bounding_box.width = prediction.w
    bounding_box.height = prediction.h
    bounding_box.label = prediction.ID
    res.boxes.append(bounding_box)

  res.dim = len(res.boxes)
  ###

  ros.loginfo("%s ...detection complete!", info_name)

  if debug_mode:
    ros.loginfo("%s I found %d bounding boxes:", info_name, res.dim)
    for i in range(res.dim):
      box = res.boxes[i]
      ros.loginfo("%s   %d: label=\"%s\", xc=%f, yc=%f, width=%f, height=%f", info_name, i, box.label, box.xc, box.yc, box.width, box.height)

  return res


if __name__ == "__main__":
  ros.init_node("vision_node")
  service = ros.Service("get_bounding_boxes", GetBoundingBoxes, get_bounding_boxes_handler)
  debug_mode = ros.get_param("/debug_mode")
  ros.loginfo("%s vision_node is ready!", info_name)
  ros.spin()
