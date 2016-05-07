#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <wiringPi.h>

using namespace std;

namespace hc_sr04_node {

// Maximum distance reported. Values over this distance
// report MAX_DISTANCE. TODO make this a property.
const static float MAX_DISTANCE = 20;
const static float DIST_SCALE = 58.0;
const static float TRAVEL_TIME_MAX = MAX_DISTANCE * DIST_SCALE;

class Sonar {
 public:
  Sonar(int t, int e) : trigger_(t), echo_(e) {
    pinMode(trigger_, OUTPUT);
    pinMode(echo_, INPUT);
    digitalWrite(trigger_, LOW);
    delay(1000);
  }
  
  float distance(bool* error) {
    // Send trig pulse
    digitalWrite(trigger_, HIGH);
    delayMicroseconds(20);
    digitalWrite(trigger_, LOW);

    // Wait for echo. Very rarely (2 of 12K at 20Hz)
    // see ECHO never go HIGH so we include a way to
    // bail. 
    int bail = 1000;
    while(digitalRead(echo_) == LOW) {
      if (--bail == 0) {
	*error = true;
	return 0;
      }
    }

    // Measure time for echo. Return early if the
    // pulse is appearing to take too long. Note:
    // error case of never going LOW results in
    // MAX reading :/
    long startTime = micros();
    long travelTime = 0;
    while(digitalRead(echo_) == HIGH) {
      travelTime = micros() - startTime;
      if (travelTime > TRAVEL_TIME_MAX) {
	travelTime = TRAVEL_TIME_MAX;
	break;
      }
      delayMicroseconds(100);
    }
    
    // Return distance in cm
    *error = false;    
    return travelTime / 58.0;
  }
 
private:
  int trigger_;
  int echo_;
};

} // namespace hc_sr04_node

int main(int argc, char **argv) {

  // Start ROS node.
  ROS_INFO("Starting node");
  ros::init(argc, argv, "hc_sr04s");
  ros::NodeHandle node;
  ros::Rate rate(10);  // 10 hz

  // Build N sonars.
  wiringPiSetupSys();  // uses gpio pin numbering
  // TODO: config these
  vector<hc_sr04_node::Sonar> sonars;
  sonars.push_back(hc_sr04_node::Sonar(24, 25));
  sonars.push_back(hc_sr04_node::Sonar(22, 23));
  sonars.push_back(hc_sr04_node::Sonar(18, 27));

  // Build a publisher for each sonar.
  vector<ros::Publisher> sonar_pubs;
  for (int i = 0; i < sonars.size(); ++i) {
    stringstream ss;
    ss << "sonar_" << i;
    sonar_pubs.push_back(node.advertise<sensor_msgs::Range>(ss.str(), 10));
  }
  
  // Build base range message that will be used for
  // each time a msg is published.
  sensor_msgs::Range range;
  range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range.min_range = 0.0;
  range.max_range = hc_sr04_node::MAX_DISTANCE;
 
  float distance;
  bool error;
  while(ros::ok()) {    
    for (int i = 0; i < sonars.size(); ++i) {
      range.header.stamp = ros::Time::now();
      float distance = sonars[i].distance(&error);
      if (error) {
	ROS_WARN("Error on sonar %d", i);
      }
      else {
	range.range = distance;
	sonar_pubs[i].publish(range);
      }
    }    
    rate.sleep();    
  }
  return 0;
}
