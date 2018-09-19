#!/usr/bin/env python

import rospy
from topology_map.srv import *

serviceName = 'topoSrv/SaveMap'

if __name__ == "__main__":
    rospy.wait_for_service(serviceName)
    mapSaving = rospy.ServiceProxy(serviceName, SaveMap)
    res = mapSaving()
    print(res)
