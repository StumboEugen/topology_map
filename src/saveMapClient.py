#!/usr/bin/env python

import rospy
from topology_map.srv import *


if __name__ == "__main__":
    rospy.wait_for_service('TopoSrv/SaveMap')
    mapSaving = rospy.ServiceProxy('topoSrv/SaveMap', SaveMap)
    res = mapSaving()
    print(res)
