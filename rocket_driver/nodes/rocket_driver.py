#!/usr/bin/env python

import roslib; roslib.load_manifest('rocket_driver')

from rocket_msgs.srv import *
import rospy
from rocket_driver import RocketManager

# Commands:
# 0: down
# 1: up
# 2: left
# 3: right
# 4: fire continuously
# 5: stop

# Service to fire N rockets
def fire(req):
   launcher.start_movement(4)
   rospy.sleep(3.5)
   # send some random command to stop movement
   launcher.start_movement(0)
   launcher.start_movement(1)
   launcher.stop_movement()
   launcher.check_limits()
   return RocketResponse() 

if __name__ == "__main__":
   rospy.init_node('rocket_driver', log_level=rospy.DEBUG)
   print "Node initialized"
   # TODO: find USB rockets and set them up here
   manager = RocketManager()
   manager.acquire_devices()
   if len(manager.launchers) == 0:
      rospy.logerror("Failed to find rocket launcher")
      exit(-1)
   launcher = manager.launchers[0]
   launcher.usb_debug = True
   print "RocketManager initialized"

   s = rospy.Service('rocket_fire', Rocket, fire)
   print "Service initialized"
   rospy.logdebug('rocket_driver ready')
   rospy.spin()
