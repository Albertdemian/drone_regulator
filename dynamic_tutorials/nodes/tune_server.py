#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import FieldConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {Kp}, {Kd}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("Tuner", anonymous = False)

    srv = Server(FieldConfig, callback)
    rospy.spin()