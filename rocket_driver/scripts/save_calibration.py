#!/usr/bin/env python

import roslib; roslib.load_manifest('rocket_driver')

import rospy
import yaml

def write_calibration(fname, arv, var):
   out = yaml.dump({'avg':avg, 'var':var})
   with open(fname, 'w') as f:
      f.write(out)

if __name__ == "__main__":
   rospy.init_node('save_cal', log_level=rospy.DEBUG)

   if rospy.has_param('avg') and rospy.has_param('var'):
      print 'Calibration found on prameter server'
      avg = rospy.get_param('avg')
      var = rospy.get_param('var')

      write_calibration("rocket_cal.yaml", avg, var);
