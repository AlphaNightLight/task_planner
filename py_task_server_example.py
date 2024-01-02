#!/usr/bin/env python

import rospy as ros

from ur5_lego.srv import Vectorize,VectorizeRequest,VectorizeResponse
from ur5_lego.msg import Point2D

def vect(req):
  res = VectorizeResponse();
  res.p.dim = req.dim
  for i in range(req.dim):
    res.p.a.append(Point2D(req.x, req.x))

  ros.loginfo("request: x=%ld, dim=%ld", req.x, req.dim)
  ros.loginfo("sending back response: dim=%ld", res.p.dim)
  for i in range(req.dim):
    ros.loginfo("  %d: (%ld,%ld)", i, res.p.a[i].x, res.p.a[i].y)
  return res

if __name__ == "__main__":
  ros.init_node("task_server_example")
  service = ros.Service("vectorize_example", Vectorize, vect)
  ros.loginfo("py_task_server_example is is ready")
  ros.spin()
