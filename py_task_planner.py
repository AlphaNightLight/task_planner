#!/usr/bin/env python

import rospy as ros
import sys

from ur5_lego.srv import *

if __name__ == "__main__":
  # with ros.ServiceProxy() you don't need ros.init_node()
  # but you can't use ros.loginfo() in that case, so I include it anyway.
  ros.init_node('py_task_planner')

  if len(sys.argv) < 3:
    ros.loginfo("usage: py_task_planner X DIM")
    sys.exit(1)
  x = int(sys.argv[1])
  dim = int(sys.argv[2])

  ros.wait_for_service("vectorize_example")
  try:
    vectorize_example = ros.ServiceProxy("vectorize_example", Vectorize)
    res = vectorize_example(x, dim)
    ros.loginfo("Vector: dim=%ld", res.p.dim)
    for i in range(res.p.dim):
      ros.loginfo("  %d: (%ld,%ld)", i, res.p.a[i].x, res.p.a[i].y)
  except rospy.ServiceException as e:
    ros.logerr("Failed to call service vectorize_example")
    sys.exit(1)
