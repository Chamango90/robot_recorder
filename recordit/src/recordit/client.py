#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger


class Recorder(object):
    def __init__(self, name="/robot_recorder"):
        rospy.loginfo("Trying to connect to server '" + name + "/start' ...")
        rospy.wait_for_service(name + "/start")
        rospy.loginfo("Connected.")
        self.start_record = rospy.ServiceProxy(name + "/start", Trigger)
        self.save_record = rospy.ServiceProxy(name + "/save", Trigger)

    def __enter__(self):
        rospy.loginfo("Start recorder.")
        self.start_record()
        return

    def __exit__(self, type, value, traceback):
        self.save_record()
        rospy.loginfo("Save recording.")
        return
