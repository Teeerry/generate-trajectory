#!/usr/bin/env python

from generate_traj.srv import *
import rospy

def handle_generate_traj(req):
    print "My waypoints: (%s , %s , %s)"%(req.x, req.y, req.z)
    req.result = True
    return trajResponse(req.result)

def generate_traj_server():
    rospy.init_node('generate_traj_server')
    s = rospy.Service('generate_traj', traj, handle_generate_traj)
    print "Ready to send the traj."
    rospy.spin()

if __name__ == "__main__":
    generate_traj_server()
