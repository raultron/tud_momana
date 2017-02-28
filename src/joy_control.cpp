#include "tud_coop_uv/SetController.h"
#include "tud_momana/StartStopMomana.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include <unordered_map>

double height = 2;
double velx = 0;
double vely = 0;
double angz = 0;
std::unordered_map<std::string, uint> controller({{"joy", 0},
                                                  {"tracking", 1},
                                                  {"joint", 2}});
bool buttonTakeoff = false;
bool buttonLand = false;
bool buttonEnable = false;
bool buttonControlJoy = false;
bool buttonControlTracking = false;
bool buttonControlJoint = false;
bool buttonMomanaOdom = false;
bool momanaOdom_started = false;


std_msgs::Empty empty;

ros::Subscriber cmd_vel_sub;
ros::Publisher twist_pub, takeoff_pub, land_pub;
ros::ServiceClient set_controller_client;
ros::ServiceClient start_stop_momana_client;
tud_coop_uv::SetController srv_SetController;
tud_momana::StartStopMomana srv_StartStopMomana;

ros::Time buttonTakeoff_pressed_instant;
ros::Time buttonStart_pressed_instant;
ros::Duration delay_button(1.0);

void set_hover(void) {
  geometry_msgs::Twist cmd_vel_out;
  cmd_vel_out.linear.x = 0;
  cmd_vel_out.linear.y = 0;
  cmd_vel_out.linear.z = 0;
  cmd_vel_out.angular.x = 0;
  cmd_vel_out.angular.y = 0;
  cmd_vel_out.angular.z = 0;
  twist_pub.publish(cmd_vel_out);
}

void joyCallback(const sensor_msgs::Joy& in) {
  geometry_msgs::Twist out_twist;
  vely = double(in.axes[0]);
  velx = double(in.axes[1]);
  angz = double(in.axes[3]);
  height = double(in.axes[4]);


  buttonEnable = bool(in.buttons[0]);
  buttonLand = bool(in.buttons[1]);
  buttonTakeoff = bool(in.buttons[2]);
  buttonControlJoy = bool(in.buttons[3]);
  buttonControlTracking = bool(in.buttons[4]);
  buttonControlJoint = bool(in.buttons[5]);
  buttonMomanaOdom = double(in.buttons[8]);


  if (buttonControlJoy || buttonControlTracking || buttonControlJoint) {
    if (buttonControlJoy) {
      srv_SetController.request.controller = controller["joy"];
    } else if (buttonControlTracking) {
      srv_SetController.request.controller = controller["tracking"];
    } else if (buttonControlJoint) {
      srv_SetController.request.controller = controller["joint"];
    }

    // call service in node Merge
    if (set_controller_client.call(srv_SetController)) {
      if (srv_SetController.response.result) {
        ROS_INFO("Succesfully changed controller");
      } else {
        ROS_ERROR("Problem changing controller");
      }
    } else {
      ROS_ERROR("Failed to call service /tud_coop_uv/set_controller");
    }
  }

  if (buttonTakeoff) {
    if ((ros::Time::now() - buttonTakeoff_pressed_instant) > delay_button) {
      buttonTakeoff_pressed_instant = ros::Time::now();
      takeoff_pub.publish(empty);
      ROS_INFO("Button Pressed");
    } else {
      ROS_INFO("Waiting delay)");
    }
  }
  if (buttonLand) {
    land_pub.publish(empty);
  }

  out_twist.linear.x = velx;
  out_twist.linear.y = vely;
  out_twist.linear.z = height;
  out_twist.angular.z = angz;

  if (buttonEnable) {
    // Enable flying without hover
    out_twist.angular.x = 1.0;
    out_twist.angular.y = 1.0;
  } else {
    // Default. Flying using auto hover
    out_twist.angular.x = 0.0;
    out_twist.angular.y = 0.0;
  }

  if (buttonMomanaOdom) {
    if ((ros::Time::now() - buttonStart_pressed_instant) > delay_button) {
      buttonStart_pressed_instant = ros::Time::now();
      if(momanaOdom_started){
        //We want to stop
        srv_StartStopMomana.request.start = false;
        srv_StartStopMomana.request.stop = true;
      } else {
        //We want to start
        srv_StartStopMomana.request.start = true;
        srv_StartStopMomana.request.stop = false;
      }

      // call service in node momana
      if (start_stop_momana_client.call(srv_StartStopMomana)) {
        if (srv_StartStopMomana.response.state) {
          ROS_INFO("Momana Started");
          momanaOdom_started = true;
        } else {
          ROS_INFO("Momana Stopped");
          momanaOdom_started = false;
        }
      } else {
        ROS_ERROR("Failed to call service /tud_momana/start_stop");
      }


    }
    else{
      ROS_INFO("waiting delay button");
    }
  }

  twist_pub.publish(out_twist);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "joy_control");  // Name of the node
  ros::NodeHandle nh;

  buttonTakeoff_pressed_instant = ros::Time::now();
  buttonStart_pressed_instant = ros::Time::now();
  set_controller_client = nh.serviceClient<tud_coop_uv::SetController>(
      "tud_coop_uv/set_controller");
  start_stop_momana_client = nh.serviceClient<tud_momana::StartStopMomana>("tud_momana/start_stop");
  cmd_vel_sub = nh.subscribe("/joy", 1, joyCallback);
  twist_pub = nh.advertise<geometry_msgs::Twist>("/joy/cmd_vel", 1);
  takeoff_pub = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  land_pub = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
  ros::Rate rate(10);  // 10 hz

  while (nh.ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
