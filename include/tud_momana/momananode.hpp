#ifndef MOMANANODE_HPP
#define MOMANANODE_HPP

#include "ros/ros.h"
#include <boost/thread.hpp>

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
    bool check_c3po_move_server(void);
    bool check_r2d2_move_server(void);
    void sendGoal_and_wait_c3po(const robotino_local_move::LocalMoveGoal& goal);
    void sendGoal_and_wait_r2d2(const robotino_local_move::LocalMoveGoal& goal);

private:
  ros::ServiceServer start_stop_momana_srv_;
  State state_;
  boost::thread run_thread_;

  actionlib::SimpleActionClient<robotino_local_move::LocalMoveAction> ac_c3po_;
  actionlib::SimpleActionClient<robotino_local_move::LocalMoveAction> ac_r2d2_;
};

#endif // MOMANANODE_HPP
