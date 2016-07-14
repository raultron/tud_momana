#ifndef MOMANANODE_HPP
#define MOMANANODE_HPP

#include "ros/ros.h"
#include "tud_momana/StartStopMomana.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "robotino_local_move/LocalMoveAction.h"

enum State { Idle, Running, Paused, Finished };

class MomanaNode
{
public:
  MomanaNode();
    ros::NodeHandle nh_;

    // ROS message callbacks
    bool start_stop_service_callback(tud_momana::StartStopMomana::Request& request,
                                 tud_momana::StartStopMomana::Response& response);
    void run(void);

private:
  ros::ServiceServer start_stop_momana_srv_;
  State state_;

  actionlib::SimpleActionClient<robotino_local_move::LocalMoveAction> ac_c3po_;
  actionlib::SimpleActionClient<robotino_local_move::LocalMoveAction> ac_r2d2_;
};

#endif // MOMANANODE_HPP
