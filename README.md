c++ [ROS](http://www.ros.org/) node wrapper for a [hc-sr04](http://www.micropik.com/PDF/HCSR04.pdf) ultrasonic sonar.

![hc-sr04](hc_sr04.jpg)

Publishes [sensor_msgs::Range](http://docs.ros.org/jade/api/sensor_msgs/html/msg/Range.html) messages at 10Hz to `/sonar` topic.
Specifically populates `range`. Publish rate and topic name are configurable.

on rasp pi 2 uses ~??% cpu and <??% mem

````
rostopic echo /sonar
---
????
````		    

TODOS
* need some analysis on `field_of_view`, `min_range` and `max_range`