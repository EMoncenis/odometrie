#include "ros/ros.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include "odometrie/flow_sensor.h"

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>


#include <iostream>
#include <sstream>
#include <string>

double R, P, Y, tof, Dx, Dy;
double CaptorResolution = 30.0;
double  fieldOfView= 0.383972; //22° in radians, the field of view
double  a = 3.333; //scalar use


//////////// Function for get the yaw pitch roll /////////////////////
void get_yaw_pitch_roll ( const diagnostic_msgs::DiagnosticArray::ConstPtr&  msg) { //type of message
  std::string roll = msg->status[0].values[0].value;
  std::string pitch = msg->status[0].values[1].value;
  std::string yaw = msg->status[0].values[2].value;
  // R = 0.0;
  std::stringstream(roll)>>R ;
  // P = 0.0;
  std::stringstream(pitch)>>P ;
  // Y = 0.0;
  std::stringstream(yaw)>>Y ;
  ROS_INFO("IMU MESSAGE :");
  ROS_INFO("Y: %f", Y);
  ROS_INFO("P: %f", P);
  ROS_INFO("R: %f", R);
  ROS_INFO("<------------------>");
}

///////////// Function for get the time of flight //////////////////
//Message with data type float :
void get_tof ( const std_msgs::Float64::ConstPtr&  msg ) {
  tof = msg->data;
  ROS_INFO("tof : ", tof);
}

////////////// Function for get the Delta X and Delta Y values during the deplacement/////////////////////
//Data capture by te PMW3901
//Message with data type flow_sensor : this is custom message with the different values in float64
void get_flow_sensor_data ( const odometrie::flow_sensor::ConstPtr&  msg ) {
  Dx = msg->deltaX;
  Dy = msg->deltaY;
  // Dx = 0.0;
  // std::stringstream(deltaXY)>>Dx ;
  ROS_INFO("deltaX: ", Dx);
  ROS_INFO("deltaY: ", Dy);
}

//////////////// Different functions for calculate the distance moved in X and Y ////////////////////
double  calculation_distance_X (double  Dx, double  altitude) {
  double  distance_moved_X = (Dx * altitude)/(CaptorResolution * a) * 2 * tan(fieldOfView/2);
  return distance_moved_X;
}

double  calculation_distance_Y (double  Dx, double  altitude) {
  double  distance_moved_Y = (Dy * altitude)/(CaptorResolution * a) * 2 * tan(fieldOfView/2);
  return distance_moved_Y;
}

double  calculation_error_X (double  R) {
  double  compensation_X = (R * CaptorResolution * a) / fieldOfView;
  return compensation_X;
}

double  calculation_error_Y (double P) {
  double compensation_X = (P * CaptorResolution * a) / fieldOfView;
  return compensation_X;
}

double real_distance_X(double distance_moved_X, double compensation_X) {
  double real_distance_X = distance_moved_X - compensation_X;
  return real_distance_X;
}

double real_distance_Y(double distance_moved_Y, double compensation_Y) {
  double real_distance_Y = distance_moved_Y - compensation_Y;
  return real_distance_Y;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometrie_node");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("diagnostics", 1, get_yaw_pitch_roll);
  ros::Subscriber sub1 = n.subscribe("tof", 1, get_tof);
  ros::Subscriber sub2 = n.subscribe("flow_sensor", 1, get_flow_sensor_data);

  while (ros::ok()) {
      ros::spinOnce();

      double distX = calculation_distance_X(Dx, tof);
      double errX = calculation_error_X(R);
      std::cout << errX <<std::endl;
      double distance_moved_X = real_distance_X(distX, errX);
      std::cout << distance_moved_X <<std::endl;

      double distY = calculation_distance_Y(Dy, tof);
      double errY = calculation_error_Y(R);
      std::cout << errY <<std::endl;
      double distance_moved_Y = real_distance_Y(distY, errY);
      std::cout << distance_moved_Y<<std::endl;
    }
  return 0;
}
