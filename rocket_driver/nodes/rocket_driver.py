#!/usr/bin/env python

import roslib; roslib.load_manifest('rocket_driver')

from rocket_msgs.srv import *
import rospy
from rocket_driver import RocketManager
import math
import yaml

DOWN=0
UP=1
LEFT=2
RIGHT=3

ALPHA_RANGE=320.0
ALPHA_HOME=ALPHA_RANGE/2.0
# TODO: measure and update these
#  TODO TODO: move to calibration file
BETA_RANGE=30.0
BETA_HOME=-5.0

# Commands:
# 0: down
# 1: up
# 2: left
# 3: right
# 4: fire continuously
# 5: stop

class RocketDriver:
   def __init__(self):
      # find USB rockets and set them up
      manager = RocketManager()
      manager.acquire_devices()
      if len(manager.launchers) == 0:
         rospy.logerror("Failed to find rocket launcher")
         exit(-1)
      # TODO: handle more than one launcher
      self.launcher = manager.launchers[0]
      # launcher.usb_debug = True
   
      # Calibration
      if rospy.has_param('avg'):
         print 'Calibration found on prameter server'
         self.avg = rospy.get_param('avg')
         self.var = rospy.get_param('var')
      else:
         print 'No calibration found. calibrating'
         (self.avg, self.var) = self.calibrate()
         rospy.set_param('avg', self.avg)
         rospy.set_param('var', self.var)

      self.pos_home()
   
      rospy.Service('rocket_fire', Rocket, self.fire)
      rospy.Service('rocket_move', RocketMove, self.move)
      rospy.Service('rocket_pos', RocketPos, self.position)
      rospy.logdebug('rocket_driver ready')

   # Rocket service
   #  [rocket_msgs/Rocket]:
   #  ---
   def fire(self, req):
      self.launcher.start_movement(4)
      rospy.sleep(3.5)
      # send some random command to stop movement
      self.launcher.start_movement(0)
      self.launcher.start_movement(1)
      self.launcher.stop_movement()
      self.launcher.check_limits()
      return RocketResponse() 
   
   # RocketMove service
   #  [rocket_msgs/RocketMove]:
   #  int8 DOWN=0
   #  int8 UP=1
   #  int8 RIGHT=2
   #  int8 LEFT=3
   #  int8 dir
   #  duration time
   #  ---
   def move(self, req):
      print "Move %d %f"%(req.dir, req.time.to_sec())
      self.launcher.start_movement(req.dir)
      rospy.sleep(req.time)
      self.launcher.stop_movement()
      return RocketMoveResponse()

   # absolute positioning functions

   # move to a limit
   def move_limit(self, d):
      self.launcher.start_movement(d)
      limits = self.launcher.check_limits()
      while not limits[d]:
         limits = self.launcher.check_limits()

   # move to the home position and reset position variables
   def pos_home(self):
      self.move_limit(DOWN)
      self.move_limit(LEFT)
      self.alpha = ALPHA_HOME
      self.alpha_var = 0
      self.beta = BETA_HOME
      self.beta_var = 0
   
   # absolute position
   # alpha: angle left/right from center
   # beta: angle above horizontal (really, anlge above bottom limit)
   def pos_abs(self, alpha, beta):
      da = alpha - self.alpha
      db = beta - self.beta
      # if da > 0, turn left
      if da > 0:
         d = LEFT
      else:
         d = RIGHT
      if abs(da) > 0:
         time = abs((self.avg[d] / ALPHA_RANGE) * da)
         print "alpha: angle: %f, time %f"%(da, time)
         self.launcher.start_movement(d)
         rospy.sleep(time)
         self.launcher.stop_movement()
         self.alpha = alpha
         var = (self.var[d]/self.avg[d] * time)
         var_deg = var * (ALPHA_RANGE/self.avg[d]) * (ALPHA_RANGE/self.avg[d])
         self.alpha_var += var_deg
         
      # if db > 0, move up
      if db > 0:
         d = UP
      else:
         d = DOWN
      if abs(db) > 0:
         time = abs((self.avg[d] / BETA_RANGE) * db)
         print "beta: angle: %f, time %f"%(db, time)
         self.launcher.start_movement(d)
         rospy.sleep(time)
         self.launcher.stop_movement()
         self.beta = beta
         var = db * db * (self.var[d] / self.avg[d] / BETA_RANGE / BETA_RANGE)
         self.beta_var += var

      limits = self.launcher.check_limits()
      if limits[LEFT]:
         self.alpha = ALPHA_HOME
         self.alpha_var = 0
      if limits[DOWN]:
         self.beta = BETA_HOME
         self.beta_var = 0
      print "Position %f %f"%(self.alpha, self.beta)
      print "Variances %f %f"%(self.alpha_var, self.beta_var)

   def position(self, req):
      self.pos_abs(req.alpha, req.beta)
      return RocketPosResponse(self.alpha_var, self.beta_var)
   
   def time_movement(self, move):
      start = rospy.Time.now()
      self.launcher.start_movement(move)
      limits = self.launcher.check_limits()
      while not limits[move]:
         limits = self.launcher.check_limits()
      end = rospy.Time.now()
      return end - start
   
   
   def calibrate(self):
      self.time_movement(DOWN)
      self.time_movement(LEFT)
   
      data = []
      for i in range(0, 10):
         print i
         # Time how long it takes to move the launcher from bottom to top
         lift_time = self.time_movement(UP).to_sec()
         print "lift ", lift_time
   
         lower_time = self.time_movement(DOWN).to_sec()
         print "lower ", lower_time
   
         right_time = self.time_movement(RIGHT).to_sec()
         print "right ", right_time
         
         left_time = self.time_movement(LEFT).to_sec()
         print "left ", left_time
   
         data.append((lower_time, lift_time, left_time, right_time))
   
      avg = [0.0, 0.0, 0.0, 0.0] # average full-axis travel time
      var = [0.0, 0.0, 0.0, 0.0] # variance of full-axis travel time
   
      # for each movement direction, compute time and variance
      for i in range(0, 4):
         for d in data:
            avg[i] += d[i]
         avg[i] /= len(data)
         for d in data:
            var[i] += (avg[i] - d[i])*(avg[i] - d[i])
         var[i] /= len(data)
         print "avgerage %d %f"%(i, avg[i])
         print "std dev %f"%(math.sqrt(var[i]))
   
      return (avg, var)

if __name__ == "__main__":
   rospy.init_node('rocket_driver', log_level=rospy.DEBUG)
   driver = RocketDriver()
   rospy.spin()

