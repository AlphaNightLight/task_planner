#!/usr/bin/env python
"""
@file pointcloud_node.py
@brief Ros node that exposes a bounding_boxes_to_poses service of type ur5_lego.srv.BoundingBoxesToPoses
Thanks to the pointcloud submodule it converts 2D bounding boxes into 3d poses of the blocks.
@date   04/01/2024
@author Alex Pegoraro
"""

import rospy as ros

from ur5_lego.srv import BoundingBoxesToPoses,BoundingBoxesToPosesRequest,BoundingBoxesToPosesResponse
from ur5_lego.msg import BoundingBox,TargetPose
from sensor_msgs.msg import Image,PointCloud2

from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler
import cv2
import os

cwd = os.getcwd()
os.chdir("/home/alex/ros_ws/src/ur5_lego/src/pointcloud/")
from ur5_lego_modules.posky_code import object_detection, Input, Output
os.chdir(cwd)



info_name = "  [pointcloud_node ]:"
debug_mode = False



def bounding_boxes_to_poses_handler(req):
  """! handler of bounding_boxes_to_poses service, type ur5_lego.srv.BoundingBoxesToPoses
  It reads image and pointcloud from the camera and passes it together with the bounding boxes to the object_detection() function.
  @param req: the list of bounding boxes
  @return res: the list of estimated poses
  """
  ros.loginfo("%s Analyzing the pointcloud...", info_name)
  res = BoundingBoxesToPosesResponse()

  ### Estimate the poses
  img_raw = ros.wait_for_message("/ur5/zed_node/left/image_rect_color", Image)
  pointcloud_raw = ros.wait_for_message("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2)

  input_bounding_boxes = []
  for box in req.boxes:
    bb = Input()
    bb.xc = box.xc
    bb.yc = box.yc
    bb.h = box.height
    bb.w = box.width
    bb.ID = box.label
    input_bounding_boxes.append(bb)

  os.chdir("/home/alex/ros_ws/src/ur5_lego/src/pointcloud/")
  estimated_poses = object_detection(img_raw, pointcloud_raw, input_bounding_boxes)
  os.chdir(cwd)

  for p in estimated_poses:
    tp = TargetPose()
    tp.position.x = p.xc
    tp.position.y = p.yc
    tp.position.z = p.zc
    tp.euler.x = p.roll
    tp.euler.y = p.pitch
    tp.euler.z = p.yaw
    q = quaternion_from_euler(p.roll, p.pitch, p.yaw, 'sxyz')
    tp.orientation.x = q[0]
    tp.orientation.y = q[1]
    tp.orientation.z = q[2]
    tp.orientation.w = q[3]
    tp.label = p.ID
    res.poses.append(tp)

  res.dim = len(res.poses)
  ###

  ros.loginfo("%s ...pose estimation complete!", info_name)

  if debug_mode:
    ros.loginfo("%s I estimated %d poses:", info_name, res.dim)
    for i in range(res.dim):
      pose = res.poses[i]
      ros.loginfo("%s   %d: label=\"%s\", position=[%f,%f,%f], quaternion=%f+[%f,%f,%f], euler_angles=[%f,%f,%f]", info_name, i, pose.label, pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.euler.x, pose.euler.y, pose.euler.z)

  return res


if __name__ == "__main__":
  ros.init_node("pointcloud_node")
  service = ros.Service("bounding_boxes_to_poses", BoundingBoxesToPoses, bounding_boxes_to_poses_handler)
  debug_mode = ros.get_param("/debug_mode")
  ros.loginfo("%s pointcloud_node is ready!", info_name)
  ros.spin()
