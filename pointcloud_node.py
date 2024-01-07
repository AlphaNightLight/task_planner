#!/usr/bin/env python

import rospy as ros

from ur5_lego.srv import BoundingBoxesToPoses,BoundingBoxesToPosesRequest,BoundingBoxesToPosesResponse
from ur5_lego.msg import BoundingBox,TargetPose


info_name = "    [pointcloud_node ]:"
debug_mode = False


def bounding_boxes_to_poses_handler(req):
  ros.loginfo("%s Analyzing the pointcloud...", info_name)
  res = BoundingBoxesToPosesResponse()

  res.dim = 0
  # Estimate the poses

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
