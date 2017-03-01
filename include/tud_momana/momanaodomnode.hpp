#ifndef MOMANAODOMNODE_HPP
#define MOMANAODOMNODE_HPP

#include "ros/ros.h"
#include "tud_momana/StartOdometry.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <std_srvs/Empty.h>
#include <angles/angles.h>
#include <boost/circular_buffer.hpp>

class MomanaOdomNode
{
public:
  MomanaOdomNode();
    ros::NodeHandle nh_;

    // ROS services callbacks
    //bool start_odom_service_callback(tud_momana::StartOdometry::Request& request,
    //                             tud_momana::StartOdometry::Response& response);
    bool start_odom_service_callback(std_srvs::Empty::Request& request,
                                 std_srvs::Empty::Response& response);
    bool switch_static_ref_service_callback(std_srvs::Empty::Request& request,
                                            std_srvs::Empty::Response& response);
    bool set_c3po_static_service_callback(std_srvs::Empty::Request& request,
                                            std_srvs::Empty::Response& response);
    bool set_r2d2_static_service_callback(std_srvs::Empty::Request& request,
                                            std_srvs::Empty::Response& response);

    bool filter_enable_service_callback(std_srvs::Empty::Request& request,
                                            std_srvs::Empty::Response& response);
    bool filter_disable_service_callback(std_srvs::Empty::Request& request,
                                            std_srvs::Empty::Response& response);

    tf::StampedTransform &tf_interpolation(tf::StampedTransform& new_tf,
                                    tf::StampedTransform& old_tf, double interpolation_weight);
    tf::Vector3 vector_interpolation(tf::Vector3 new_vector, tf::Vector3 old_vector,
                                     double interpolation_weight);
    tf::Quaternion quaternion_interpolation(tf::Quaternion new_q, tf::Quaternion old_q,
                                            double interpolation_weight);

    void init_odom();
    void publish_transforms(void);
    void publish_odometry_at_switch(void);
    void publish_odometry(void);
    void c3po_static_calc(void);
    void r2d2_static_calc(void);
    void set_r2d2_static(void);
    void set_c3po_static(void);
    void momana_odom_spin(void);
    void wait_for_transforms(void);
    tf::StampedTransform do_interpolation_tf_buffer(void);

    tf::StampedTransform get_c3po_to_r2d2(void);

private:
    ros::ServiceServer start_odom_srv_;
    ros::ServiceServer set_c3po_static_srv_;
    ros::ServiceServer set_r2d2_static_srv_;
    ros::ServiceServer switch_static_ref_srv_;
    ros::ServiceServer filter_enable_srv_;
    ros::ServiceServer filter_disable_srv_;

    //Odometry publishers
    ros::Publisher odometry_pub_c3po_;
    ros::Publisher odometry_pub_r2d2_;

    ros::Publisher odometry_at_switch_c3po_pub_;
    ros::Publisher odometry_at_switch_r2d2_pub_;

    //Navigation path publishers
    ros::Publisher path_pub_c3po_;
    ros::Publisher path_pub_r2d2_;

    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;

    std::string uav_base_link_;

    tf::StampedTransform odom_to_c3po_rel_;
    tf::StampedTransform odom_to_r2d2_rel_;
    tf::StampedTransform odom_to_quad_rel_;
    tf::StampedTransform c3po_to_r2d2_;
    tf::StampedTransform c3po_to_quad_;

    nav_msgs::Odometry c3po_odometry_msg_, r2d2_odometry_msg_;

    nav_msgs::Path c3po_path_msg_, r2d2_path_msg_;

    int seq_odometry_;
    int seq_odometry_at_switch_;
    bool c3po_static_;
    bool r2d2_static_;
    bool odom_initialized_;
    int camera_framerate_;

    double distance_exp_average;

    // Create a circular buffer for distance with a capacity for 25 double.
    boost::circular_buffer<double> distance_hist_;

    // Circular buffer for raw relative transformation between robots history
    boost::circular_buffer<tf::StampedTransform> buffer_c3po_to_r2d2_;
    bool filter_enabled_;

};

#endif // MOMANAODOMNODE_HPP

