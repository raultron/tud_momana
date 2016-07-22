#ifndef MOMANAODOMNODE_HPP
#define MOMANAODOMNODE_HPP

#include "ros/ros.h"
#include "tud_momana/StartOdometry.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"
#include <std_srvs/Empty.h>

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

    void init_odom();
    void publish_transforms(void);
    void publish_odometry(void);
    void c3po_static_calc(void);
    void r2d2_static_calc(void);
    void set_r2d2_static(void);
    void set_c3po_static(void);
    void momana_odom_spin(void);
    void wait_for_transforms(void);

    tf::StampedTransform get_c3po_to_r2d2(void);

private:
    ros::ServiceServer start_odom_srv_;
    ros::ServiceServer switch_static_ref_srv_;

    ros::Publisher odometry_pub_c3po_;
    ros::Publisher odometry_pub_r2d2_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;

    std::string uav_base_link_;

    tf::StampedTransform odom_to_c3po_rel_;
    tf::StampedTransform odom_to_r2d2_rel_;
    tf::StampedTransform odom_to_quad_rel_;
    tf::StampedTransform c3po_to_r2d2_;
    tf::StampedTransform c3po_to_quad_;

    nav_msgs::Odometry c3po_odometry_msg_, r2d2_odometry_msg_;

    int sequence_;
    bool c3po_static_;
    bool r2d2_static_;
    bool odom_initialized_;

};

#endif // MOMANAODOMNODE_HPP

