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

double R, P, Y, T, Dx, Dy;
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
//Message evoyé en string
// void get_tof ( const std_msgs::String::ConstPtr&  msg ) {
//   std::string tof = msg->data;
//   T = 0.0;
//   std::stringstream(tof)>>T ;
//   ROS_INFO("tof : ", T);
// }

//Message en float :
void get_tof ( const std_msgs::Float64::ConstPtr&  msg ) {
  double tof = msg->data;
  ROS_INFO("tof : ", tof);
}

void get_flow_sensor_data ( const odometrie::flow_sensor::ConstPtr&  msg ) {
  double deltaX = msg->deltaX;
  double deltaY = msg->deltaY;
  // Dx = 0.0;
  // std::stringstream(deltaXY)>>Dx ;
  ROS_INFO("deltaX: ", deltaXY);
  ROS_INFO("deltaY: ", deltaY);
}

// double  calcul_distance_X (double  Dx, double  altitude) {
//   double  distance_parcourue_X = (Dx * altitude)/(CaptorResolution * a) * 2 * tan(fieldOfView/2);
//   return distance_parcourue_X;
// }
//
// double  calcul_distance_Y (double  Dx, double  altitude) {
//   double  distance_parcourue_Y = (Dy * altitude)/(CaptorResolution * a) * 2 * tan(fieldOfView/2);
//   return distance_parcourue_Y;
// }
//
// double  calcul_erreur_X (double  R) {
//   double  compensation_X = (R * CaptorResolution * a) / fieldOfView;
//   return compensation_X;
// }
//
// double  calcul_erreur_Y (double P) {
//   double compensation_X = (P * CaptorResolution * a) / fieldOfView;
//   return compensation_X;
// }
//
// double reel_distance_X(double distance_parcourue_X, double compensation_X) {
//   double reel_distance_X = distance_parcourue_X - compensation_X;
//   return reel_distance_X;
// }
//
// double reel_distance_Y(double distance_parcourue_Y, double compensation_Y) {
//   double reel_distance_Y = distance_parcourue_Y - compensation_Y;
//   return reel_distance_Y;
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometrie_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("diagnostics", 1, get_yaw_pitch_roll);
  ros::Subscriber sub1 = n.subscribe("tof", 1, get_tof);
  ros::Subscriber sub2 = n.subscribe("flow_sensor", 1, get_flow_sensor_data);
  while (ros::ok()) {
      ros::spinOnce();
      double distX = calcul_distance_X(Dx, T);
      double errX = calcul_erreur_X(R);
      std::cout << errX <<std::endl;
      double distance_parcourue_X = reel_distance_X(distX, errX);
      std::cout << distance_parcourue_X <<std::endl;
    }
  return 0;
}
