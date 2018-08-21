#!/usr/bin/env python

import sys
import rospy
from generate_traj.srv import *

def generate_traj_client(x, y, z):
    rospy.init_node('generate_traj_client')
    rospy.wait_for_service('generate_traj')
    try:
        generate_traj = rospy.ServiceProxy('generate_traj', traj)
        resp1 = generate_traj(x, y, z)
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
	z = int(sys.argv[3])
	result = False
    #else:
        #print usage()
        #sys.exit(1)
    print "Requesting %s,%s,%s"%(x, y, z)
    print "%s,%s,%s"%(x, y, generate_traj_client(x, y, z))
