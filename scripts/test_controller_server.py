#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from compliant_controllers.cfg import JointTaskSpaceCompliantControllerConfig

def callback(config, level):
    rospy.loginfo("Reconfigure request received:")
    for key, val in config.items():
        rospy.loginfo(f"  {key}: {val}")
    return config

if __name__ == "__main__":
    rospy.init_node("compliant_controller_reconfigure_node")
    srv = Server(JointTaskSpaceCompliantControllerConfig, callback)
    rospy.loginfo("Dynamic reconfigure test server is up.")
    rospy.spin()
