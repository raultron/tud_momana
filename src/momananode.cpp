#include "tud_momana/momananode.hpp"

MomanaNode::MomanaNode(): ac_c3po_("/c3po/local_move", true), ac_r2d2_("/r2d2/local_move", true)
{
  state_ = Idle;
  start_stop_momana_srv_ = nh_.advertiseService("tud_momana/start_stop", &MomanaNode::start_stop_service_callback,this);
}

bool MomanaNode::start_stop_service_callback(tud_momana::StartStopMomana::Request& request,
                             tud_momana::StartStopMomana::Response& response){
    if (request.start){
      if(state_ == Idle){
        ROS_INFO("Mobile marker Navigation Started");
        state_ = Running;
        //START()
        run();
        // return state to the service call
        response.state = true;
      } else if (state_ == Running){
        ROS_INFO("Mobile marker Navigation already Started");
        // return state to the service call
        response.state = true;
      }
    }

    if (request.stop){
      if(state_ == Idle){
        ROS_INFO("Mobile marker Navigation already Stopped");
        // return state to the service call
        response.state = false;
      } else if (state_ == Running){
        ROS_INFO("Mobile marker Navigation Stopped");
        //STOP_Momana
        // return state to the service call
        state_ = Idle;
        response.state = false;
      }
    }


    return true;
}


void MomanaNode::run(void){
  // Start Odom messages publishing

  // check for c3po service
  ROS_INFO("Waiting for c3po local move server");
  ac_c3po_.waitForServer();
  ROS_INFO("c3po Local move server found");

  // check for r2d2 service
  ROS_INFO("Waiting for r2d2 local move server");
  ac_r2d2_.waitForServer();
  ROS_INFO("r2d2 Local move server found");

  state_ = Idle;

  // main loop

    // Action, move c3po 50cm to the front
      //check until finishes or stop signal

    // Action, move r2d2 50cm to the front
      //check until finishes or stop signal
}
