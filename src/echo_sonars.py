#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range

sonars = [0, 0, 0]
lkg_sonars = [0, 0, 0]

def callback(msg, idx):
  global sonars
  sonars[idx] = msg.range
  if lkg_sonars[idx] == 0 or msg.range != 30.0:
    lkg_sonars[idx] = msg.range
      
rospy.init_node('sonar_subscriber')
for i in range(3):
  rospy.Subscriber("/robotZero/sonar_%d" % i, Range,
                   callback, i)

r = rospy.Rate(5)
while not rospy.is_shutdown():
  s = ["%5.2f" % v for v in sonars]
  s += ["%5.2f" % v for v in lkg_sonars]
  print " ".join(s)
  r.sleep()
                      
