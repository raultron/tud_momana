#include "tud_momana/momanaodomnode.hpp"

MomanaOdomNode::MomanaOdomNode():
  sequence_(0),
  distance_exp_average(0),
  filter_enabled_(false)
{
  //start_odom_srv_ =
  //    nh_.advertiseService("tud_momana/start_odom",
  //                         &MomanaOdomNode::start_odom_service_callback, this);
  filter_enable_srv_ = nh_.advertiseService("tud_momana/filter_enable", &MomanaOdomNode::filter_enable_service_callback, this);
  filter_disable_srv_ = nh_.advertiseService("tud_momana/filter_disable", &MomanaOdomNode::filter_disable_service_callback, this);
  set_c3po_static_srv_ = nh_.advertiseService("tud_momana/set_c3po_static", &MomanaOdomNode::set_c3po_static_service_callback, this);
  set_r2d2_static_srv_ = nh_.advertiseService("tud_momana/set_r2d2_static", &MomanaOdomNode::set_r2d2_static_service_callback, this);
  switch_static_ref_srv_ = nh_.advertiseService("tud_momana/switch_static_ref", &MomanaOdomNode::switch_static_ref_service_callback, this);
  start_odom_srv_ = nh_.advertiseService("tud_momana/start_odom", &MomanaOdomNode::start_odom_service_callback, this);


  odometry_pub_c3po_ = nh_.advertise<nav_msgs::Odometry>("/c3po/odom", 1, true);
  odometry_pub_r2d2_ = nh_.advertise<nav_msgs::Odometry>("/r2d2/odom", 1, true);

  path_pub_c3po_ = nh_.advertise<nav_msgs::Path>("/c3po/path", 1, true);
  path_pub_r2d2_ = nh_.advertise<nav_msgs::Path>("/r2d2/path", 1, true);

  nh_.param<std::string>("uav_base_link_", uav_base_link_,
                         "/ardrone_base_link");
  nh_.param<int>("camera_framerate", camera_framerate_, 25); // in hz
  distance_hist_.set_capacity(camera_framerate_);
  buffer_c3po_to_r2d2_.set_capacity(camera_framerate_),

  // Transformation frames_id
  odom_to_c3po_rel_.frame_id_ = "rel_c3po/odom";
  odom_to_c3po_rel_.child_frame_id_ = "rel_c3po/base_link";

  odom_to_r2d2_rel_.frame_id_ = "rel_r2d2/odom";
  odom_to_r2d2_rel_.child_frame_id_ = "rel_r2d2/base_link";

  // Odometry frames_id
  c3po_odometry_msg_.header.frame_id = "rel_c3po/odom";
  c3po_odometry_msg_.child_frame_id = "rel_c3po/base_link";

  r2d2_odometry_msg_.header.frame_id = "rel_r2d2/odom";
  r2d2_odometry_msg_.child_frame_id = "rel_r2d2/base_link";

  // Nav path frames_id
  c3po_path_msg_.header.frame_id = "rel_c3po/odom";

  r2d2_path_msg_.header.frame_id = "rel_r2d2/odom";
}


tf::StampedTransform MomanaOdomNode::get_c3po_to_r2d2(void){
  ros::Time spin_begin = ros::Time::now();
  ros::Time spin_end;
  tf::StampedTransform c3po_to_r2d2;
  ros::Time now = ros::Time::now();
  // We look for a transformation betwwen c3po and r2d2
  try {
    tf_listener_.waitForTransform("c3po_base_link", "r2d2_base_link", ros::Time(0),
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

  //! Optimization, this transformation is noisy (comes from the camera)
  //! It should be possible to filter this noise.
  //! Our first aproximation is to use a one second moving average
  //! based on the framerate of the camera (25hz).
  //! In the control part, before doing the static marker switch
  //! both robots need to be static for 1 second

  //add relative transformation to the circular buffer
  buffer_c3po_to_r2d2_.push_back(c3po_to_r2d2);

  if (filter_enabled_){
    tf::StampedTransform c3po_to_r2d2_filtered;
    c3po_to_r2d2_filtered.setData(do_interpolation_tf_buffer());
    spin_end = ros::Time::now();
    ros::Duration spin_duration(spin_end-spin_begin);
    ROS_DEBUG("Odom cycle duration: %f", spin_duration.toSec());
    return c3po_to_r2d2_filtered;
  }else{
    return c3po_to_r2d2;
  }
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
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("-------------Relative Odometry Initialized (C3PO Static)--------------");
  ROS_INFO("----------------------------------------------------------------------");

  momana_odom_spin();
}

void MomanaOdomNode::wait_for_transforms(void){
  ros::Time now;
  bool transform_available = false;
  while(!transform_available){
    now = ros::Time::now();
    ROS_INFO("MomanaOdom: waiting for a transformation between r2d2 and c3po");
    try {
      transform_available = tf_listener_.waitForTransform("c3po_base_link", "r2d2_base_link", now,
                                    ros::Duration(5));
      if (transform_available){
        ROS_INFO("MomanaOdom: found a valid transform between r2d2 and c3po");
      }

    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      // if we dont have a valid transformation return false
    }
  }
}

void MomanaOdomNode::set_r2d2_static(void){
  //! TODO(racuna)
  //!Average one second of measurements to improve pose
  //ros::Duration(2.0).sleep();
  ROS_INFO("MomanaOdom: r2d2 configured as static");
  c3po_static_ = false;
  r2d2_static_ = true;
}

void MomanaOdomNode::set_c3po_static(void){
  //! TODO(racuna)
  //!Average one second of measurements to improve pose
  //! Implement this in the control!!!!
  //ros::Duration(2.0).sleep();
  ROS_INFO("MomanaOdom: c3po configured as static");
  c3po_static_ = true;
  r2d2_static_ = false;
}

void MomanaOdomNode::c3po_static_calc(void){
  ros::Time now = ros::Time::now();

  // c3po static no new transform calculation
  //odom_to_c3po_rel_ = odom_to_c3po_rel_;
  odom_to_c3po_rel_.stamp_ = now;

  // we calculate r2d2 relative transform
  c3po_to_r2d2_ = get_c3po_to_r2d2(); //from quad_camera
  odom_to_r2d2_rel_.setData(odom_to_c3po_rel_*c3po_to_r2d2_);
  odom_to_r2d2_rel_.stamp_ = now;
}

void MomanaOdomNode::r2d2_static_calc(void){
  ros::Time now = ros::Time::now();

  // r2d2 static no new transform calculation
  //odom_to_r2d2_rel_ = odom_to_r2d2_rel_;
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

  ROS_DEBUG("C3PO odom x: %f", c3po_pose.position.x);

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

  // Construct and publish nav path messages
  geometry_msgs::PoseStamped r2d2_pose_stamped, c3po_pose_stamped;
  r2d2_pose_stamped.header.frame_id = r2d2_odometry_msg_.header.frame_id;
  r2d2_pose_stamped.header.stamp = r2d2_odometry_msg_.header.stamp;
  r2d2_pose_stamped.header.seq = r2d2_odometry_msg_.header.seq;
  r2d2_pose_stamped.pose = r2d2_pose;

  c3po_pose_stamped.header.frame_id = c3po_odometry_msg_.header.frame_id;
  c3po_pose_stamped.header.stamp = c3po_odometry_msg_.header.stamp;
  c3po_pose_stamped.header.seq = c3po_odometry_msg_.header.seq;
  c3po_pose_stamped.pose = c3po_pose;

  c3po_path_msg_.poses.push_back(c3po_pose_stamped);
  r2d2_path_msg_.poses.push_back(r2d2_pose_stamped);

  path_pub_c3po_.publish(c3po_path_msg_);
  path_pub_r2d2_.publish(r2d2_path_msg_);
}


void MomanaOdomNode::momana_odom_spin(void){
//  //Calculate distance and rotation between markers
//  //for debugging
//  tf::StampedTransform c3po_to_r2d2;
//  c3po_to_r2d2 = get_c3po_to_r2d2();
//  double x,y,z,distance, yaw;
//  tf::Vector3 origin = c3po_to_r2d2.getOrigin();
//  x = origin.x();
//  y = origin.y();
//  z = origin.z();


//  distance = std::sqrt(x*x + y*y + z*z);
//  distance_hist_.push_back(distance);
//  double distance_average = 0.0;
//  if (distance_hist_.full()){
//    for (auto it = distance_hist_.begin(); it != distance_hist_.end(); it++){
//      distance_average += *it;
//    }
//    distance_average = distance_average/distance_hist_.size();
//  }
//  //accumulator = (alpha * new_value) + (1.0 - alpha) * accumulator
//  double alpha = 1.0/25; //because we have 25 hz of sampling time and we want 1 second average
//  distance_exp_average = (alpha * distance) + (1.0 - alpha) * distance_exp_average;

//  ROS_DEBUG("Distance, moving average: %f, exponential average: %f", distance_average, distance_exp_average);
//  yaw = tf::getYaw(c3po_to_r2d2.getRotation());
//  //ROS_DEBUG("Yaw Between Markers: %f", angles::to_degrees(yaw));


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

bool MomanaOdomNode::set_c3po_static_service_callback(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response){
  set_c3po_static();
  return true;
}

bool MomanaOdomNode::set_r2d2_static_service_callback(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response){
  set_r2d2_static();
  return true;
}

bool MomanaOdomNode::start_odom_service_callback(std_srvs::Empty::Request& request,
                                                 std_srvs::Empty::Response& response){
  ROS_INFO("Starting relative odometry on ground robots");
  init_odom();
  return true;

//    tud_momana::StartOdometry::Request& request,
//    tud_momana::StartOdometry::Response& response) {

//  if (request.start) {
//    ROS_INFO("Starting relative odometry on ground robots");
//    // Start odometry system ()
//    init_odom();
//    response.result = true;
//  } else {
//    response.result = false;
//  }
}

bool MomanaOdomNode::filter_enable_service_callback(std_srvs::Empty::Request& request,
                                                    std_srvs::Empty::Response& response){
  filter_enabled_ = true;
  return true;
}

bool MomanaOdomNode::filter_disable_service_callback(std_srvs::Empty::Request& request,
                                                     std_srvs::Empty::Response& response){
  filter_enabled_ = false;
  return true;
}


tf::StampedTransform& MomanaOdomNode::tf_interpolation(tf::StampedTransform& new_tf,
                                           tf::StampedTransform& old_tf, double interpolation_weight) {

  tf::Vector3 pos = vector_interpolation(new_tf.getOrigin(), old_tf.getOrigin(), interpolation_weight);
  new_tf.setOrigin(pos);

  tf::Quaternion orn = quaternion_interpolation(new_tf.getRotation(), old_tf.getRotation(), interpolation_weight);
  new_tf.setRotation(orn);
  return new_tf;
}

tf::Vector3 MomanaOdomNode::vector_interpolation(
    tf::Vector3 new_vector, tf::Vector3 old_vector,
    double interpolation_weight) {
  tf::Vector3 interpolated_vector((1 - interpolation_weight) * old_vector.x() +
                                       interpolation_weight * new_vector.x(),
                                  (1 - interpolation_weight) * old_vector.y() +
                                       interpolation_weight * new_vector.y(),
                                  (1 - interpolation_weight) * old_vector.z() +
                                       interpolation_weight * new_vector.z());
  return interpolated_vector;
}

tf::Quaternion MomanaOdomNode::quaternion_interpolation(
    tf::Quaternion new_q, tf::Quaternion old_q,
    double interpolation_weight){
    tf::Quaternion interpolated_q;
    interpolated_q = old_q.slerp(new_q, interpolation_weight);
    return interpolated_q;
  }

tf::StampedTransform MomanaOdomNode::do_interpolation_tf_buffer(void){

  tf::StampedTransform tf_filtered;
  int count = 1.0;
  tf_filtered.setIdentity();

  for(tf::StampedTransform it : buffer_c3po_to_r2d2_){

    tf_filtered = tf_interpolation( it, tf_filtered, (1.0/count));
    count++;
    //ROS_INFO("X in each tf: %f, filtered: %f, count %d", it.getOrigin().x(), tf_filtered.getOrigin().x(), count);
  }

  return tf_filtered;
}

///TODO(racuna) read the odometry from the robotinos and use this information when the transformation is too old. (safeguard)
/// Also find a way to stop the Action if by any change we are not detecting the robots.

