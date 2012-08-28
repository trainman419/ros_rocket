#!/usr/bin/env python

import roslib; roslib.load_manifest('rocket_driver')

from std_srvs.srv import Empty,EmptyResponse
from rocket_msgs.msg import *
import rospy
from rocket_driver import RocketManager
import math
import yaml

DOWN=0
UP=1
LEFT=2
RIGHT=3

ALPHA_RANGE=320.0*math.pi/180.0
ALPHA_HOME=ALPHA_RANGE/2.0
ALPHA_MAX=ALPHA_HOME
ALPHA_MIN=ALPHA_HOME-ALPHA_RANGE
# TODO: measure and update these
#  TODO TODO: move to calibration file
BETA_RANGE=30.0*math.pi/180.0
BETA_HOME=5.0*math.pi/180.0
BETA_MAX=BETA_HOME
BETA_MIN=BETA_HOME-BETA_RANGE

# Hysteresis of 5 degrees; if movement is less than this, don't bother
HYSTERESIS=5.0*math.pi/180.0

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
         rospy.logerr("Failed to find rocket launcher")
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
      self.alpha_target = ALPHA_HOME
      self.beta_target = BETA_HOME
      self.firing = False
      self.moving = False
   
      rospy.Service('rocket_fire', Empty, self.fire)
      rospy.Subscriber('rocket_command', RocketCommand, self.position)
      self.pub = rospy.Publisher('rocket_position', RocketPosition)
      rospy.logdebug('rocket_driver ready')

   # Rocket service
   #  [rocket_msgs/Rocket]:
   #  ---
   def fire(self, req):
      self.firing = True
      while self.moving:
         rospy.sleep(0.01)
      self.launcher.start_movement(4)
      rospy.sleep(3.5)
      # send some random command to stop movement
      self.launcher.start_movement(0)
      self.launcher.start_movement(1)
      self.launcher.stop_movement()
      rospy.sleep(0.1)
      self.launcher.check_limits()
      rospy.sleep(0.1)
      self.firing = False
      return EmptyResponse() 
   
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
   # OLD absolute positioning; now we do positioning from spin()
   def pos_abs(self, alpha, beta):
      da = alpha - self.alpha
      db = beta - self.beta
      self.moving = True
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

      # wait a little before checking limits
      rospy.sleep(0.1)

      limits = self.launcher.check_limits()
      if limits[LEFT]:
         print "At left limit; re-homing"
         self.alpha = ALPHA_HOME
         self.alpha_var = 0
      if limits[DOWN]:
         print "At lower limit; re-homing"
         self.beta = BETA_HOME
         self.beta_var = 0
      print "Position %f %f"%(self.alpha, self.beta)
      print "Variances %f %f"%(self.alpha_var, self.beta_var)
      self.moving = False

   def spin(self):
      d = None
      cmd_start = rospy.Time.now()
      while not rospy.is_shutdown():
         if not self.firing:
            # update position estimates
            limits = self.launcher.check_limits()
         else:
            limits = (False, False, False, False)
         if limits[LEFT]:
            self.alpha = ALPHA_HOME
            self.alpha_var = 0
         elif d == LEFT or d == RIGHT:
            time = (rospy.Time.now() - cmd_start).to_sec()
            delta = (ALPHA_RANGE / self.avg[d]) * time

            var = (self.var[d]/self.avg[d] * time)
            var_angle = var*(ALPHA_RANGE/self.avg[d])*(ALPHA_RANGE/self.avg[d])
            self.alpha_var += var_angle

            if d == RIGHT:
               delta = -delta
            self.alpha += delta
         if limits[DOWN]:
            self.beta = BETA_HOME
            self.beta_var = 0
         elif d == UP or d == DOWN:
            time = (rospy.Time.now() - cmd_start).to_sec()
            delta = (BETA_RANGE / self.avg[d]) * time
            if d == UP:
               delta = -delta
            self.beta += delta

            var = (self.var[d]/self.avg[d] * time)
            var_angle = var * (BETA_RANGE/self.avg[d])*(BETA_RANGE/self.avg[d])
            self.beta_var += var_angle

         # compute next motion
         d = None
         da = self.alpha_target - self.alpha
         db = self.beta_target - self.beta
         if abs(da) > HYSTERESIS:
            if da > 0:
               d = LEFT
            else:
               d = RIGHT

            if not self.firing:
               self.moving = True
               self.launcher.start_movement(d)
               cmd_start = rospy.Time.now()
         elif abs(db) > HYSTERESIS:
            if db > 0:
               d = DOWN
            else:
               d = UP

            if not self.firing:
               self.moving = True
               self.launcher.start_movement(d)
               cmd_start = rospy.Time.now()
         else:
            if not self.firing:
               self.launcher.stop_movement()
            self.moving = False

         self.pub.publish(RocketPosition(self.alpha, self.beta, self.alpha_var,
                  self.beta_var))

         rospy.sleep(0.05)

   def position(self, req):
      # TODO: take commands and process them asynchronously
      a = min(max(req.alpha, ALPHA_MIN), ALPHA_MAX)
      b = min(max(req.beta, BETA_MIN), BETA_MAX)
      self.alpha_target = a
      self.beta_target = b
   
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
      self.time_movement(UP)
      self.time_movement(LEFT)
   
      data = []
      for i in range(0, 10):
         print i
         # Time how long it takes to move the launcher from top to bottom
         lower_time = self.time_movement(DOWN).to_sec()
         print "lower ", lower_time
   
         lift_time = self.time_movement(UP).to_sec()
         print "lift ", lift_time
   
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
   driver.spin()

