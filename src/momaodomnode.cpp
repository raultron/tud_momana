#include "tud_momana/momaodomnode.hpp"

MomaOdomNode::MomaOdomNode():
  seq_odometry_(0),
  seq_odometry_at_switch_(0),
  distance_exp_average(0),
  filter_enabled_(false)
{
  odom_initialized_ = false;
  filter_enable_srv_ = nh_.advertiseService("tud_momana/filter_enable", &MomaOdomNode::filter_enable_service_callback, this);
  filter_disable_srv_ = nh_.advertiseService("tud_momana/filter_disable", &MomaOdomNode::filter_disable_service_callback, this);
  set_cam_static_srv_ = nh_.advertiseService("tud_momana/set_cam_static", &MomaOdomNode::set_cam_static_service_callback, this);
  set_marker_static_srv_ = nh_.advertiseService("tud_momana/set_marker_static", &MomaOdomNode::set_marker_static_service_callback, this);
  switch_static_ref_srv_ = nh_.advertiseService("tud_momana/switch_static_ref", &MomaOdomNode::switch_static_ref_service_callback, this);
  start_odom_srv_ = nh_.advertiseService("tud_momana/start_odom", &MomaOdomNode::start_odom_service_callback, this);

  odometry_pub_cam_ = nh_.advertise<nav_msgs::Odometry>("/cam/odom", 1, true);
  odometry_pub_marker_ = nh_.advertise<nav_msgs::Odometry>("/marker/odom", 1, true);

  odometry_at_switch_cam_pub_ = nh_.advertise<nav_msgs::Odometry>("/cam/odom_at_switch", 1, true);
  odometry_at_switch_marker_pub_ = nh_.advertise<nav_msgs::Odometry>("/marker/odom_at_switch", 1, true);

  path_pub_cam_ = nh_.advertise<nav_msgs::Path>("/cam/path", 1, true);
  path_pub_marker_ = nh_.advertise<nav_msgs::Path>("/marker/path", 1, true);


  nh_.param<int>("camera_framerate", camera_framerate_, 25); // in hz
  distance_hist_.set_capacity(camera_framerate_);
  buffer_cam_to_marker_.set_capacity(camera_framerate_),

  nh_.param<std::string>("camera_frame", cam_frame_, "/camera_link");
  nh_.param<std::string>("marker_frame", marker_frame_, "/marker_link");

  // Transformation frames_id
  odom_to_cam_rel_.frame_id_ = "rel_cam/odom";
  odom_to_cam_rel_.child_frame_id_ = "rel_cam/base_link";

  odom_to_marker_rel_.frame_id_ = "rel_marker/odom";
  odom_to_marker_rel_.child_frame_id_ = "rel_marker/base_link";

  // Odometry frames_id
  cam_odometry_msg_.header.frame_id = "rel_cam/odom";
  cam_odometry_msg_.child_frame_id = "rel_cam/base_link";

  marker_odometry_msg_.header.frame_id = "rel_marker/odom";
  marker_odometry_msg_.child_frame_id = "rel_marker/base_link";

  // Nav path frames_id
  cam_path_msg_.header.frame_id = "rel_cam/odom";
  marker_path_msg_.header.frame_id = "rel_marker/odom";
}




void MomaOdomNode::init_odom(void){
  ///wait until all required transformations are available
  wait_for_transforms();

  seq_odometry_ = 0;
  // set odom frame as current Camera position
  odom_to_cam_rel_.setIdentity();

  //set_cam_static
  set_cam_static();

  odom_initialized_ = true;
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("-------------Relative Odometry Initialized (Camera is Static)--------------");
  ROS_INFO("----------------------------------------------------------------------");

  moma_odom_spin();
}

void MomaOdomNode::moma_odom_spin(void){
  if(odom_initialized_){
    if (cam_static_){
      cam_static_calc();
    } else{
      marker_static_calc();
    }

    publish_transforms();
    publish_odometry();
  }
  ros::spinOnce();
}

void MomaOdomNode::wait_for_transforms(void){
  ros::Time now;
  bool transform_available = false;
  while(!transform_available){
    now = ros::Time::now();
    ROS_INFO("MomaOdom: waiting for a transformation between marker and cam");
    try {
      transform_available = tf_listener_.waitForTransform(cam_frame_, marker_frame_, now,
                                    ros::Duration(5));
      if (transform_available){
        ROS_INFO("MomaOdom: found a valid transform between marker and cam");
      }

    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      // if we dont have a valid transformation return false
    }
  }
}

tf::StampedTransform MomaOdomNode::get_cam_to_marker(void){
  ros::Time spin_begin = ros::Time::now();
  ros::Time spin_end;
  tf::StampedTransform cam_to_marker;
  ros::Time now = ros::Time::now();
  // We look for a transformation betwwen cam and marker
  try {
    tf_listener_.waitForTransform(cam_frame_, marker_frame_, ros::Time(0),
                                  ros::Duration(0.25));
    tf_listener_.lookupTransform(cam_frame_, marker_frame_, ros::Time(0),
                                 cam_to_marker);
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
  buffer_cam_to_marker_.push_back(cam_to_marker);

  if (filter_enabled_){
    tf::StampedTransform cam_to_marker_filtered;
    cam_to_marker_filtered.setData(do_interpolation_tf_buffer());
    spin_end = ros::Time::now();
    ros::Duration spin_duration(spin_end-spin_begin);
    ROS_DEBUG("Odom cycle duration: %f", spin_duration.toSec());
    return cam_to_marker_filtered;
  }else{
    return cam_to_marker;
  }
}

void MomaOdomNode::set_cam_static(void){
  //! TODO(racuna)
  //!Average one second of measurements to improve pose
  //! Implement this in the control!!!!
  //ros::Duration(2.0).sleep();
  ROS_INFO("MomaOdom: Cam Static - MOVE MARKER");
  cam_static_ = true;
  marker_static_ = false;
  if(odom_initialized_){
    cam_static_calc();
    publish_odometry_at_switch();
  }
}

void MomaOdomNode::set_marker_static(void){
  //! TODO(racuna)
  //!Average one second of measurements to improve pose
  //ros::Duration(2.0).sleep();
  ROS_INFO("MomaOdom: Marker Static - MOVE CAMERA");
  cam_static_ = false;
  marker_static_ = true;
  if(odom_initialized_){
    marker_static_calc();
    publish_odometry_at_switch();
  }
}

void MomaOdomNode::cam_static_calc(void){
  ros::Time now = ros::Time::now();

  // cam static no new transform calculation
  //odom_to_cam_rel_ = odom_to_cam_rel_;
  odom_to_cam_rel_.stamp_ = now;

  // we calculate cam to marker relative transform
  cam_to_marker_ = get_cam_to_marker(); //from marker PnP pose estimation using camera image
  odom_to_marker_rel_.setData(odom_to_cam_rel_*cam_to_marker_);
  odom_to_marker_rel_.stamp_ = now;
}

void MomaOdomNode::marker_static_calc(void){
  ros::Time now = ros::Time::now();

  // marker static no new transform calculation
  //odom_to_marker_rel_ = odom_to_marker_rel_;
  odom_to_marker_rel_.stamp_ = now;

  // we calculate cam relative transform
  cam_to_marker_ = get_cam_to_marker(); //from marker PnP pose estimation using camera image
  tf::Transform marker_to_cam = cam_to_marker_.inverse();
  odom_to_cam_rel_.setData(odom_to_marker_rel_*marker_to_cam);
  odom_to_cam_rel_.stamp_ = now;
}

void MomaOdomNode::publish_transforms(void){
  tf_broadcaster_.sendTransform(odom_to_cam_rel_);
  tf_broadcaster_.sendTransform(odom_to_marker_rel_);
  //tf_broadcaster_.sendTransform(odom_to_quad_rel_);
}

void MomaOdomNode::publish_odometry_at_switch(void){
  nav_msgs::Odometry cam_odom_at_switch_msg, marker_odom_at_switch_msg;
  geometry_msgs::Pose cam_pose, marker_pose;

  tf::poseTFToMsg(odom_to_cam_rel_, cam_pose);
  tf::poseTFToMsg(odom_to_marker_rel_, marker_pose);

  cam_odom_at_switch_msg = cam_odometry_msg_;
  marker_odom_at_switch_msg = marker_odometry_msg_;

  // Construct cam odometry at switch message
  cam_odom_at_switch_msg.header.seq = seq_odometry_at_switch_;
  cam_odom_at_switch_msg.header.stamp = odom_to_cam_rel_.stamp_;
  cam_odom_at_switch_msg.pose.pose = cam_pose;

  // Construct marker odometry message
  marker_odom_at_switch_msg.header.seq = seq_odometry_at_switch_;
  marker_odom_at_switch_msg.header.stamp = odom_to_marker_rel_.stamp_;
  marker_odom_at_switch_msg.pose.pose = marker_pose;
  // dont need to set up twist in the navigation message (yet..)

  //Publisht the odometry messages
  odometry_at_switch_cam_pub_.publish(cam_odom_at_switch_msg);
  odometry_at_switch_marker_pub_.publish(marker_odom_at_switch_msg);
  seq_odometry_at_switch_++;

}

void MomaOdomNode::publish_odometry(void){
  geometry_msgs::Pose cam_pose, marker_pose;

  tf::poseTFToMsg(odom_to_cam_rel_, cam_pose);
  tf::poseTFToMsg(odom_to_marker_rel_, marker_pose);

  ROS_DEBUG("Cam odom x: %f", cam_pose.position.x);

  // Construct cam odometry message
  cam_odometry_msg_.header.seq = seq_odometry_;
  cam_odometry_msg_.header.stamp = odom_to_cam_rel_.stamp_;
  cam_odometry_msg_.pose.pose = cam_pose;
  // dont need to set up twist in the navigation message (yet..)

  // Construct marker odometry message
  marker_odometry_msg_.header.seq = seq_odometry_;
  marker_odometry_msg_.header.stamp = odom_to_marker_rel_.stamp_;
  marker_odometry_msg_.pose.pose = marker_pose;
  // dont need to set up twist in the navigation message (yet..)

  //Publisht the odometry messages
  odometry_pub_cam_.publish(cam_odometry_msg_);
  odometry_pub_marker_.publish(marker_odometry_msg_);
  seq_odometry_++;

  // Construct and publish nav path messages
  geometry_msgs::PoseStamped marker_pose_stamped, cam_pose_stamped;
  marker_pose_stamped.header.frame_id = marker_odometry_msg_.header.frame_id;
  marker_pose_stamped.header.stamp = marker_odometry_msg_.header.stamp;
  marker_pose_stamped.header.seq = marker_odometry_msg_.header.seq;
  marker_pose_stamped.pose = marker_pose;

  cam_pose_stamped.header.frame_id = cam_odometry_msg_.header.frame_id;
  cam_pose_stamped.header.stamp = cam_odometry_msg_.header.stamp;
  cam_pose_stamped.header.seq = cam_odometry_msg_.header.seq;
  cam_pose_stamped.pose = cam_pose;

  cam_path_msg_.poses.push_back(cam_pose_stamped);
  marker_path_msg_.poses.push_back(marker_pose_stamped);

  path_pub_cam_.publish(cam_path_msg_);
  path_pub_marker_.publish(marker_path_msg_);
}


bool MomaOdomNode::switch_static_ref_service_callback(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response){
  /// NOT GOOD PROGRAMMING, ONLY VALID FOR TWO ROBOTS
  if (cam_static_){
    set_marker_static();
    sound_handle_.play(1,1);
  }else{
    set_cam_static();
    sound_handle_.play(2,1);
  }
  return true;
}

bool MomaOdomNode::set_cam_static_service_callback(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response){
  set_cam_static();
  return true;
}

bool MomaOdomNode::set_marker_static_service_callback(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response){
  set_marker_static();
  return true;
}

bool MomaOdomNode::start_odom_service_callback(std_srvs::Empty::Request& request,
                                               std_srvs::Empty::Response& response){
  ROS_INFO("Starting relative odometry on ground robots");
  init_odom();
  return true;
}

bool MomaOdomNode::filter_enable_service_callback(std_srvs::Empty::Request& request,
                                                    std_srvs::Empty::Response& response){
  filter_enabled_ = true;
  return true;
}

bool MomaOdomNode::filter_disable_service_callback(std_srvs::Empty::Request& request,
                                                     std_srvs::Empty::Response& response){
  filter_enabled_ = false;
  return true;
}


tf::StampedTransform& MomaOdomNode::tf_interpolation(tf::StampedTransform& new_tf,
                                           tf::StampedTransform& old_tf, double interpolation_weight) {

  tf::Vector3 pos = vector_interpolation(new_tf.getOrigin(), old_tf.getOrigin(), interpolation_weight);
  new_tf.setOrigin(pos);

  tf::Quaternion orn = quaternion_interpolation(new_tf.getRotation(), old_tf.getRotation(), interpolation_weight);
  new_tf.setRotation(orn);
  return new_tf;
}

tf::Vector3 MomaOdomNode::vector_interpolation(
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

tf::Quaternion MomaOdomNode::quaternion_interpolation(
    tf::Quaternion new_q, tf::Quaternion old_q,
    double interpolation_weight){
    tf::Quaternion interpolated_q;
    interpolated_q = old_q.slerp(new_q, interpolation_weight);
    return interpolated_q;
  }

tf::StampedTransform MomaOdomNode::do_interpolation_tf_buffer(void){

  tf::StampedTransform tf_filtered;
  int count = 1.0;
  tf_filtered.setIdentity();

  for(tf::StampedTransform it : buffer_cam_to_marker_){

    tf_filtered = tf_interpolation( it, tf_filtered, (1.0/count));
    count++;
    //ROS_INFO("X in each tf: %f, filtered: %f, count %d", it.getOrigin().x(), tf_filtered.getOrigin().x(), count);
  }

  return tf_filtered;
}

