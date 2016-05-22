#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range

sonars = [0, 0, 0]
def callback(msg, idx):
    global sonars
    sonars[idx] = msg.range

rospy.init_node('sonar_subscriber')
for i in range(3):
    rospy.Subscriber("/sonar_%d" % i, Range, callback, i)

r = rospy.Rate(5)
while not rospy.is_shutdown():
    print sonars
    r.sleep()
                      
