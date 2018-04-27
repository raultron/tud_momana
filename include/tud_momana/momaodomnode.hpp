#ifndef MOMAODOMNODE_HPP
#define MOMAODOMNODE_HPP

#include "ros/ros.h"
#include "tud_momana/StartOdometry.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <std_srvs/Empty.h>
#include <angles/angles.h>
#include <boost/circular_buffer.hpp>

#include <sound_play/SoundRequest.h>
#include <sound_play/sound_play.h>

class MomaOdomNode
{
public:
  MomaOdomNode();
    ros::NodeHandle nh_;
    sound_play::SoundClient sound_handle_;




    // ROS services callbacks

    bool start_odom_service_callback(std_srvs::Empty::Request& request,
                                     std_srvs::Empty::Response& response);

    bool switch_static_ref_service_callback(std_srvs::Empty::Request& request,
                                            std_srvs::Empty::Response& response);

    bool set_cam_static_service_callback(std_srvs::Empty::Request& request,
                                         std_srvs::Empty::Response& response);

    bool set_marker_static_service_callback(std_srvs::Empty::Request& request,
                                            std_srvs::Empty::Response& response);

    bool filter_enable_service_callback(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response);

    bool filter_disable_service_callback(std_srvs::Empty::Request& request,
                                         std_srvs::Empty::Response& response);

    tf::StampedTransform &tf_interpolation(tf::StampedTransform& new_tf,
                                           tf::StampedTransform& old_tf,
                                           double interpolation_weight);

    tf::Vector3 vector_interpolation(tf::Vector3 new_vector,
                                     tf::Vector3 old_vector,
                                     double interpolation_weight);

    tf::Quaternion quaternion_interpolation(tf::Quaternion new_q,
                                            tf::Quaternion old_q,
                                            double interpolation_weight);

    void init_odom();
    void publish_transforms(void);
    void publish_odometry_at_switch(void);
    void publish_odometry(void);
    void cam_static_calc(void);
    void marker_static_calc(void);
    void set_marker_static(void);
    void set_cam_static(void);
    void moma_odom_spin(void);
    void wait_for_transforms(void);
    tf::StampedTransform do_interpolation_tf_buffer(void);
    tf::StampedTransform get_cam_to_marker(void);

    tf::StampedTransform get_cam_gt(void);
    tf::StampedTransform get_marker_gt(void);

private:
    ros::ServiceServer start_odom_srv_;
    ros::ServiceServer set_cam_static_srv_;
    ros::ServiceServer set_marker_static_srv_;
    ros::ServiceServer switch_static_ref_srv_;
    ros::ServiceServer filter_enable_srv_;
    ros::ServiceServer filter_disable_srv_;

    //Odometry publishers
    ros::Publisher odometry_pub_cam_;
    ros::Publisher odometry_pub_marker_;

    ros::Publisher odometry_pub_cam_gt_;
    ros::Publisher odometry_pub_marker_gt_;

    ros::Publisher odometry_at_switch_cam_pub_;
    ros::Publisher odometry_at_switch_marker_pub_;

    //Navigation path publishers
    ros::Publisher path_pub_cam_;
    ros::Publisher path_pub_cam_gt_;
    ros::Publisher path_pub_marker_;
    ros::Publisher path_pub_marker_gt_;

    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;

    std::string cam_frame_;
    std::string marker_frame_;

    tf::StampedTransform odom_to_cam_rel_;
    tf::StampedTransform odom_to_marker_rel_;
    tf::StampedTransform cam_to_marker_;
    tf::StampedTransform world_to_cam_gt_;
    tf::StampedTransform world_to_marker_gt_;



    nav_msgs::Odometry cam_odometry_msg_, marker_odometry_msg_;

    nav_msgs::Path cam_path_msg_, marker_path_msg_;
    nav_msgs::Path cam_gt_path_msg_, marker_gt_path_msg_;

    int camera_framerate_;
    int seq_odometry_;
    int seq_odometry_at_switch_;
    bool cam_static_;
    bool marker_static_;
    bool odom_initialized_;
    bool gt_available_; // If we have ground truth publish this as well


    double distance_exp_average;

    // Create a circular buffer for distance with a capacity for 25 double.
    boost::circular_buffer<double> distance_hist_;

    // Circular buffer for raw relative transformation between robots history
    boost::circular_buffer<tf::StampedTransform> buffer_cam_to_marker_;
    bool filter_enabled_;

};

#endif // MOMAODOMNODE_HPP
