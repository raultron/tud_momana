#include "tud_momana/momanaodomnode.hpp"

MomanaOdomNode::MomanaOdomNode(): sequence_(0) {
  start_odom_srv_ =
      nh_.advertiseService("tud_momana/start_odom",
                           &MomanaOdomNode::start_odom_service_callback, this);

  switch_static_ref_srv_ = nh_.advertiseService("tud_momana/switch_static_ref", &MomanaOdomNode::switch_static_ref_service_callback, this);


  odometry_pub_c3po_ = nh_.advertise<nav_msgs::Odometry>("/c3po/odom", 1, true);
  odometry_pub_r2d2_ = nh_.advertise<nav_msgs::Odometry>("/r2d2/odom", 1, true);

  nh_.param<std::string>("uav_base_link_", uav_base_link_,
                         "/ardrone_base_link");

  // Transformation frames_id
  odom_to_c3po_rel_.frame_id_ = "rel_odom";
  odom_to_c3po_rel_.child_frame_id_ = "rel_c3po_base_link";

  odom_to_r2d2_rel_.frame_id_ = "rel_odom";
  odom_to_r2d2_rel_.child_frame_id_ = "rel_r2d2_base_link";

  // Odometry frames_id
  c3po_odometry_msg_.header.frame_id = "rel_odom";
  c3po_odometry_msg_.child_frame_id = "rel_c3po_base_link";

  r2d2_odometry_msg_.header.frame_id = "rel_odom";
  r2d2_odometry_msg_.child_frame_id = "rel_r2d2_base_link";
}

bool MomanaOdomNode::start_odom_service_callback(
    tud_momana::StartOdometry::Request& request,
    tud_momana::StartOdometry::Response& response) {

  if (request.start) {
    ROS_INFO("Starting relative odometry on ground robots");
    // Start odometry system ()
    init_odom();
    response.result = true;
  } else {
    response.result = false;
  }
}

tf::StampedTransform MomanaOdomNode::get_c3po_to_r2d2(void){
  tf::StampedTransform c3po_to_r2d2;
  ros::Time now = ros::Time::now();
  // We look for a transformation betwwen c3po and r2d2
  try {
    tf_listener_.waitForTransform("c3po_base_link", "r2d2_base_link", now,
                                  ros::Duration(0.25));
    tf_listener_.lookupTransform("c3po_base_link", "r2d2_base_link", ros::Time(0),
                                 c3po_to_r2d2);
    // ROS_DEBUG("tracking_node | Parent Frame: %s, Child frame: %s",
    // base_to_target.frame_id_.c_str(),
    // base_to_target.child_frame_id_.c_str());
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    // if we dont have a valid transformation return false
  }
  return c3po_to_r2d2;
}

void MomanaOdomNode::init_odom(void){
  ///wait until all required transformations are available
  wait_for_transforms();

  sequence_ = 0;
  // set odom frame as current C3PO position
  odom_to_c3po_rel_.setIdentity();

  //set_c3po_static
  set_c3po_static();

  odom_initialized_ = true;

  momana_odom_spin();
}

void MomanaOdomNode::wait_for_transforms(void){
  ros::Time now = ros::Time::now();
  bool transform_available = false;
  while(!transform_available){
    ROS_INFO("waiting for a transformation between r2d2 and c3po");
    try {
      transform_available = tf_listener_.waitForTransform("c3po_base_link", "r2d2_base_link", now,
                                    ros::Duration(5));
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      // if we dont have a valid transformation return false
    }
  }
}

void MomanaOdomNode::set_r2d2_static(void){
  c3po_static_ = false;
  r2d2_static_ = true;
}

void MomanaOdomNode::set_c3po_static(void){
  c3po_static_ = true;
  r2d2_static_ = false;
}

void MomanaOdomNode::c3po_static_calc(void){
  ros::Time now = ros::Time::now();

  // c3po static no new transform calculation
  odom_to_c3po_rel_ = odom_to_c3po_rel_;
  odom_to_r2d2_rel_.stamp_ = now;

  // we calculate r2d2 relative transform
  c3po_to_r2d2_ = get_c3po_to_r2d2(); //from quad_camera
  odom_to_r2d2_rel_.setData(odom_to_c3po_rel_*c3po_to_r2d2_);
  odom_to_r2d2_rel_.stamp_ = now;
}

void MomanaOdomNode::r2d2_static_calc(void){
  ros::Time now = ros::Time::now();

  // r2d2 static no new transform calculation
  odom_to_r2d2_rel_ = odom_to_r2d2_rel_;
  odom_to_r2d2_rel_.stamp_ = now;

  // we calculate c3po relative transform
  c3po_to_r2d2_ = get_c3po_to_r2d2(); //from quad_camera
  tf::Transform r2d2_to_c3po= c3po_to_r2d2_.inverse();
  odom_to_c3po_rel_.setData(odom_to_r2d2_rel_*r2d2_to_c3po);
  odom_to_c3po_rel_.stamp_ = now;
}

void MomanaOdomNode::publish_transforms(void){
  tf_broadcaster_.sendTransform(odom_to_c3po_rel_);
  tf_broadcaster_.sendTransform(odom_to_r2d2_rel_);
  //tf_broadcaster_.sendTransform(odom_to_quad_rel_);
}

void MomanaOdomNode::publish_odometry(void){
  geometry_msgs::Pose c3po_pose, r2d2_pose;

  tf::poseTFToMsg(odom_to_c3po_rel_, c3po_pose);
  tf::poseTFToMsg(odom_to_r2d2_rel_, r2d2_pose);

  // Construct c3po odometry message
  c3po_odometry_msg_.header.seq = sequence_;
  c3po_odometry_msg_.header.stamp = odom_to_c3po_rel_.stamp_;
  c3po_odometry_msg_.pose.pose = c3po_pose;
  // dont need to set up twist in the navigation message (yet..)

  // Construct r2d2 odometry message
  r2d2_odometry_msg_.header.seq = sequence_;
  r2d2_odometry_msg_.header.stamp = odom_to_r2d2_rel_.stamp_;
  r2d2_odometry_msg_.pose.pose = r2d2_pose;
  // dont need to set up twist in the navigation message (yet..)

  //Publisht the odometry messages
  odometry_pub_c3po_.publish(c3po_odometry_msg_);
  odometry_pub_r2d2_.publish(r2d2_odometry_msg_);
  sequence_++;
}


void MomanaOdomNode::momana_odom_spin(void){
  if(odom_initialized_){
    if (c3po_static_){
      c3po_static_calc();
    } else{
      r2d2_static_calc();
    }
    publish_transforms();
    publish_odometry();
  }
  ros::spinOnce();
}


bool MomanaOdomNode::switch_static_ref_service_callback(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response){
  /// NOT GOOD PROGRAMMING, ONLY VALID FOR TWO ROBOTS
  if (c3po_static_){
    set_r2d2_static();
  }else{
    set_c3po_static();
  }
  return true;
}
