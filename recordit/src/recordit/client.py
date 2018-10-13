#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger

def record_func(func, before_save=lambda:None):

  # define services
  recorder = rospy.get_param('~robot_recorder', '/robot_recorder')
  rospy.loginfo("Trying to connect to server '" + recorder + "/start' ...")
  rospy.wait_for_service(recorder + '/start')
  rospy.loginfo("Connected.")
  preconfigure_record = rospy.ServiceProxy(recorder + '/preconfigure', Trigger)
  start_record = rospy.ServiceProxy(recorder + '/start', Trigger)
  save_record = rospy.ServiceProxy(recorder + '/save', Trigger)

  # record
  preconfigure_record()
  rospy.loginfo("Start recorder.")
  start_record()
  func() # main function
  before_save() 
  save_record()
  rospy.loginfo("Save recording.")
  
  return True