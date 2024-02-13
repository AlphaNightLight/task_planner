#!/usr/bin/env python
"""
@file desired_poses_node.py
@brief Ros node that exposes a get_desired_poses service of type ur5_lego.srv.GetDesiredPoses
It simply imports the desired_poses_params module that defines the desired position and orientation of blocks, and sents them to the caller.
@date   04/01/2024
@author Alex Pegoraro
"""

import rospy as ros

from ur5_lego.srv import GetDesiredPoses,GetDesiredPosesRequest,GetDesiredPosesResponse
from ur5_lego.msg import TargetPose

from tf.transformations import quaternion_from_euler
from ur5_lego_modules.desired_poses_params import desired_poses
from ur5_lego_modules.desired_poses_params import x_offset
from ur5_lego_modules.desired_poses_params import y_offset
from ur5_lego_modules.desired_poses_params import z_offset


info_name = "[desired_poses_node]:"
debug_mode = False


def get_desired_poses_handler(req):
  """! handler of get_desired_poses service, type ur5_lego.srv.GetDesiredPoses
  It adjusts the values imported from desired_poses_params.py to convert them from table frame to world frame, and then returns them to the caller.
  @param req: empty
  @return res: the list of desired poses
  """
  ros.loginfo("%s desired_poses_node contacted", info_name)
  res = GetDesiredPosesResponse()

  res.dim = len(desired_poses)
  for i in range(res.dim):
    res.poses.append(TargetPose())

    res.poses[i].label = desired_poses[i].label
    res.poses[i].position.x = desired_poses[i].x + x_offset
    res.poses[i].position.y = desired_poses[i].y + y_offset
    res.poses[i].position.z = z_offset

    res.poses[i].euler.x = 0.0
    res.poses[i].euler.y = 0.0
    res.poses[i].euler.z = desired_poses[i].theta

    q = quaternion_from_euler(res.poses[i].euler.x, res.poses[i].euler.y, res.poses[i].euler.z, 'sxyz')
    res.poses[i].orientation.x = q[0]
    res.poses[i].orientation.y = q[1]
    res.poses[i].orientation.z = q[2]
    res.poses[i].orientation.w = q[3]


  if debug_mode:
    ros.loginfo("%s I have %d desired poses:", info_name, res.dim)
    for i in range(res.dim):
      pose = res.poses[i]
      ros.loginfo("%s   %d: label=\"%s\", position=[%f,%f,%f], quaternion=%f+[%f,%f,%f], euler_angles=[%f,%f,%f]", info_name, i, pose.label, pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.euler.x, pose.euler.y, pose.euler.z)

  return res


if __name__ == "__main__":
  ros.init_node("desired_poses_node")
  service = ros.Service("get_desired_poses", GetDesiredPoses, get_desired_poses_handler)
  debug_mode = ros.get_param("/debug_mode")
  ros.loginfo("%s desired_poses_node is ready!", info_name)
  ros.spin()
