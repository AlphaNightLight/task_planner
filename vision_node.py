#!/usr/bin/env python

import rospy as ros

from ur5_lego.srv import GetBoundingBoxes,GetBoundingBoxesRequest,GetBoundingBoxesResponse
from ur5_lego.msg import BoundingBox


info_name = "  [  vision_node   ]:"
debug_mode = False


def get_bounding_boxes_handler(req):
  ros.loginfo("%s Detecting bounding boxes...", info_name)
  res = GetBoundingBoxesResponse()

  res.dim = 0
  # Detect the bounding boxes

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
