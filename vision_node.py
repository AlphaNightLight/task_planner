#!/usr/bin/env python
"""
@file vision_node.py
@brief Ros node that exposes a get_bounding_boxes service of type ur5_lego.srv.GetBoundingBoxes
It uses a YOLOv8 model from mega_blocks_detector_project submodule to extract the bounding boxes from the camera image.
@date   04/01/2024
@author Alex Pegoraro
"""

import rospy as ros

from ur5_lego.srv import GetBoundingBoxes,GetBoundingBoxesRequest,GetBoundingBoxesResponse
from ur5_lego.msg import BoundingBox
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import os


cwd = os.getcwd()
os.chdir("/home/alex/ros_ws/src/ur5_lego/src/mega_blocks_detector_project/")
from ur5_lego_modules.bosco_code import bb, make_prediction
os.chdir(cwd)



info_name = "  [  vision_node   ]:"
debug_mode = False
path_to_image = "/home/alex/ros_ws/src/ur5_lego/img/zed.jpg"



def get_bounding_boxes_handler(req):
  """! handler of get_bounding_boxes service, type ur5_lego.srv.GetBoundingBoxes
  It reads the image from the camera and calls the make_prediction() function on it to detect the bounding boxes.
  @param req: empty
  @return res: the list of bounding boxes
  """
  ros.loginfo("%s Detecting bounding boxes...", info_name)
  res = GetBoundingBoxesResponse()

  ### Detect the bounding boxes
  img_raw = ros.wait_for_message("/ur5/zed_node/left/image_rect_color", Image)
  img_cv2 = CvBridge().imgmsg_to_cv2(img_raw, 'rgb8')
  cv2.imwrite(path_to_image, img_cv2)

  predictions = make_prediction(img_cv2)

  for prediction in predictions:
    bounding_box = BoundingBox()
    bounding_box.xc = prediction.xc
    bounding_box.yc = prediction.yc
    bounding_box.width = prediction.w
    bounding_box.height = prediction.h
    bounding_box.label = prediction.convert_num_to_id()
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
