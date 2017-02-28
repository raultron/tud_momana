#include "tud_momana/StartStopMomana.h"
#include "sound_play/SoundRequest.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
#include <unordered_map>

double height = 2;
double velx = 0;
double vely = 0;
double angz = 0;

bool buttonControlc3po = false;
bool buttonControlr2d2 = false;
bool buttonSwitchRobot = false;
bool buttonMomanaOdom = false;
bool momanaOdom_started = false;
bool c3po_active = true;
bool r2d2_active = false;




std_msgs::Empty empty;
ros::Subscriber cmd_vel_sub;
ros::Publisher c3po_cmd_vel_pub, r2d2_cmd_vel_pub;

ros::ServiceClient momana_set_c3po_static_client;
ros::ServiceClient momana_set_r2d2_static_client;
ros::ServiceClient momana_switch_static_client;
ros::ServiceClient momana_start_odom_client;
tud_momana::StartStopMomana srv_StartStopMomana;

ros::Time buttonSwitch_pressed_instant;
ros::Time buttonStartStopMomanaOdom_pressed_instant;
ros::Duration delay_button(1.0);

void joyCallback(const sensor_msgs::Joy& in) {
  geometry_msgs::Twist out_twist;
  vely = double(in.axes[0]);
  velx = double(in.axes[1]);
  angz = double(in.axes[3]);
  height = double(in.axes[4]);


  buttonSwitchRobot = bool(in.buttons[0]);
  buttonControlc3po = bool(in.buttons[6]);
  buttonControlr2d2 = bool(in.buttons[7]);
  buttonMomanaOdom = bool(in.buttons[8]);


  if (buttonSwitchRobot) {
    if ((ros::Time::now() - buttonSwitch_pressed_instant) > delay_button) {
      buttonSwitch_pressed_instant = ros::Time::now();
      std_srvs::Empty::Request req,res;
      c3po_active = !c3po_active;
      r2d2_active = !r2d2_active;
      momana_switch_static_client.call(req, res);

      ROS_INFO("Switched Robots");
    } else {
      ROS_INFO("Waiting delay)");
    }
  } else if (buttonControlc3po){
    c3po_active = true;
    r2d2_active = false;
    std_srvs::Empty::Request req,res;
    momana_set_c3po_static_client.call(req, res);
    ROS_INFO("c3po active");
  } else if (buttonControlr2d2){
    r2d2_active = true;
    c3po_active = false;
    std_srvs::Empty::Request req,res;
    momana_set_r2d2_static_client.call(req, res);
    ROS_INFO("r2d2 active");
  } else if (buttonMomanaOdom){
    // Start Odom messages publishing
    // This sets c3po as static
    r2d2_active = true;
    c3po_active = false;
    std_srvs::Empty::Request req,res;
    ROS_INFO("Sending command to start momana odometry");
    momana_start_odom_client.call(req, res);
  }


  out_twist.linear.x = velx;
  out_twist.linear.y = vely;
  out_twist.angular.z = angz;

  if(c3po_active){
    c3po_cmd_vel_pub.publish(out_twist);
  } else if (r2d2_active){
    r2d2_cmd_vel_pub.publish(out_twist);
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "joy_control");  // Name of the node
  ros::NodeHandle nh;

  buttonSwitch_pressed_instant = ros::Time::now();
  buttonStartStopMomanaOdom_pressed_instant = ros::Time::now();


  momana_set_c3po_static_client = nh.serviceClient<std_srvs::Empty>("tud_momana/set_c3po_static");
  momana_set_r2d2_static_client = nh.serviceClient<std_srvs::Empty>("tud_momana/set_r2d2_static");
  momana_switch_static_client = nh.serviceClient<std_srvs::Empty>("tud_momana/switch_static_ref");
  momana_start_odom_client = nh.serviceClient<std_srvs::Empty>("tud_momana/start_odom");



  cmd_vel_sub = nh.subscribe("/joy", 1, joyCallback);
  c3po_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/c3po/cmd_vel", 1);
  r2d2_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/r2d2/cmd_vel", 1);

  ros::Rate rate(10);  // 10 hz

  while (nh.ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}




