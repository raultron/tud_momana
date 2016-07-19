#include "tud_momana/momananode.hpp"


MomanaNode::MomanaNode()
    : ac_c3po_("/c3po/local_move", true), ac_r2d2_("/r2d2/local_move", true) {
  state_ = Idle;
  start_stop_momana_srv_ = nh_.advertiseService(
      "tud_momana/start_stop", &MomanaNode::start_stop_service_callback, this);
}

bool MomanaNode::start_stop_service_callback(
    tud_momana::StartStopMomana::Request& request,
    tud_momana::StartStopMomana::Response& response) {
  if (request.start) {
    if (state_ == Idle) {
      ROS_INFO("Mobile marker Navigation Started");
      state_ = Running;
      // START()
      run_thread_ = boost::thread(&MomanaNode::run,this);
      //run();all
      // return state to the service c
      response.state = true;
    } else if (state_ == Running) {
      ROS_INFO("Mobile marker Navigation already Started");
      // return state to the service call
      response.state = true;
    }
  }

  if (request.stop) {
    if (state_ == Idle) {
      ROS_INFO("Mobile marker Navigation already Stopped");
      // return state to the service call
      response.state = false;
    } else if (state_ == Running) {
      ROS_INFO("Receive order to Stop Momana");
      // STOP_Momana
      state_ = Idle;
      ROS_INFO("Canceling running thread..");
      run_thread_.join();
      ROS_INFO("Momana succesfully stopped");
      // return state to the service call      
      response.state = false;
    }
  }

  return true;
}

void MomanaNode::run(void) {
  robotino_local_move::LocalMoveGoal goal_r2d2, goal_c3po;
  bool result = false;

  // Start Odom messages publishing

  result = check_c3po_move_server();
  if(!result)
    return;

  result = check_r2d2_move_server();
  if(!result)
    return;


  // main loop
  for (int i = 0; i < 3; i++) {
    ROS_INFO("Move R2D2 half a meter to the front");
    goal_r2d2.move_x = 0.5;
    sendGoal_and_wait_r2d2(goal_r2d2);

    ROS_INFO("SIMULATED Move C3P0 half a meter to the front");
    goal_c3po.move_x = 0.5;
    sendGoal_and_wait_c3po(goal_c3po);
    ros::Duration(15).sleep();
  }

  state_ = Idle;
}

bool MomanaNode::check_c3po_move_server(void){
  // check for c3po service
  bool result = false;
  while(!result){
    ROS_INFO("Waiting 5s for c3po local move server");
    result = ac_c3po_.waitForServer(ros::Duration(5));
    if(!result){
      ROS_INFO("Cannot find c3po action server");
    }
    if(state_==Idle){
      ROS_INFO("Exiting running thread..");
      state_==Idle;
      return false;
    }
  }
  ROS_INFO("c3po Local move server found");
  return true;
}

bool MomanaNode::check_r2d2_move_server(void){
  // check for r2d2 service
  bool result = false;
  while(!result){
    ROS_INFO("Waiting 5s for r2d2 local move server");
    result = ac_r2d2_.waitForServer(ros::Duration(5));
    if(!result){
      ROS_INFO("Cannot find r2d2 action server");
    }
    if(state_==Idle){
      ROS_INFO("Exiting running thread..");
      state_==Idle;
      return false;
    }
  }
  ROS_INFO("r2d2 Local move server found");
  return true;
}

void MomanaNode::sendGoal_and_wait_c3po(const robotino_local_move::LocalMoveGoal& goal){
  ac_r2d2_.sendGoal(goal);
  // wait for the action to return
  bool finished_before_timeout = ac_r2d2_.waitForResult(ros::Duration(30.0));
  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac_r2d2_.getState();
    ROS_INFO("C3PO Action finished: %s", state.toString().c_str());
  } else {
    ROS_INFO("C3PO Action did not finish before the time out.");
  }
}

void MomanaNode::sendGoal_and_wait_r2d2(const robotino_local_move::LocalMoveGoal& goal){
  ac_c3po_.sendGoal(goal);
  // wait for the action to return
  bool finished_before_timeout = ac_c3po_.waitForResult(ros::Duration(30.0));
  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = ac_c3po_.getState();
    ROS_INFO("C3PO Action finished: %s", state.toString().c_str());
  } else {
    ROS_INFO("C3PO Action did not finish before the time out.");
  }
}

