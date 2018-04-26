#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

bool buttonStartOdom = false;
bool buttonSwitchMoma = false;
bool momanaOdom_started = false;


std_msgs::Empty empty;
ros::ServiceClient start_odom_client_, switch_odom_client_;
ros::Subscriber joy_sub;

ros::Time buttonStart_pressed_instant;
ros::Time buttonSwitch_pressed_instant;
ros::Duration delay_button(1.0);


void joyCallback(const sensor_msgs::Joy& in) {
  buttonStartOdom = double(in.buttons[9]);
  buttonSwitchMoma = double(in.buttons[1]);

  if (buttonStartOdom) {
    if ((ros::Time::now() - buttonStart_pressed_instant) > delay_button) {
      std_srvs::Empty::Request req;
      std_srvs::Empty::Response res;
      buttonStart_pressed_instant = ros::Time::now();
      start_odom_client_.call(req, res);
    }
    else{
      ROS_DEBUG("waiting delay button");
    }
  }

  if (buttonSwitchMoma) {
    if ((ros::Time::now() - buttonSwitch_pressed_instant) > delay_button) {
      std_srvs::Empty::Request req;
      std_srvs::Empty::Response res;
      buttonSwitch_pressed_instant = ros::Time::now();
      switch_odom_client_.call(req, res);
    }
    else{
      ROS_DEBUG("waiting delay button");
    }
  }

}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "joy_control");  // Name of the node
  ros::NodeHandle nh_;

  buttonStart_pressed_instant = ros::Time::now();
  buttonSwitch_pressed_instant = ros::Time::now();
  start_odom_client_ = nh_.serviceClient<std_srvs::Empty>("tud_momana/start_odom");

  switch_odom_client_ = nh_.serviceClient<std_srvs::Empty>("tud_momana/switch_static_ref");

  joy_sub = nh_.subscribe("/joy", 1, joyCallback);
  ros::Rate rate(10);  // 10 hz

  while (nh_.ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
