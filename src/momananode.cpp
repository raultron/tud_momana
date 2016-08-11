#include "tud_momana/momananode.hpp"


MomanaNode::MomanaNode()
    : ac_c3po_("/c3po/local_move", true),
      ac_r2d2_("/r2d2/local_move", true),
      ac_c3po_move_base_("/c3po/move_base", true),
      ac_r2d2_move_base_("/r2d2/move_base", true)
{
  state_ = Idle;
  start_stop_momana_srv_ = nh_.advertiseService(
      "tud_momana/start_stop", &MomanaNode::start_stop_service_callback, this);

  filter_enable_client_ = nh_.serviceClient<std_srvs::Empty>("tud_momana/filter_enable");
  filter_disable_client_ = nh_.serviceClient<std_srvs::Empty>("tud_momana/filter_disable");

  set_c3po_static_client_ = nh_.serviceClient<std_srvs::Empty>("tud_momana/set_c3po_static");
  set_r2d2_static_client_ = nh_.serviceClient<std_srvs::Empty>("tud_momana/set_r2d2_static");
  switch_static_client_ = nh_.serviceClient<std_srvs::Empty>("tud_momana/switch_static_ref");
  start_odom_client_ = nh_.serviceClient<std_srvs::Empty>("tud_momana/start_odom");
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response res;
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
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response res;

  bool result = false;

  // Start Odom messages publishing
  // This sets c3po as static
  start_odom_client_.call(req, res);

  result = check_c3po_move_base_server();
  if(!result)
    return;


  result = check_r2d2_move_base_server();
  if(!result)
    return;
  square_navigation_test(2,0.6);



//  // Move both robots 2m to the front
//  for (int i = 0; i < 4; i++) {
//    ROS_INFO("Move R2D2 half a meter to the front");
//    goal_r2d2.move_x = 0.5;
//    sendGoal_and_wait_r2d2(goal_r2d2, ros::Duration(10));
//    //ros::Duration(10).sleep();
//    set_r2d2_static_client_.call(req, res);

//    ROS_INFO("SIMULATED Move C3P0 half a meter to the front");
//    goal_c3po.move_x = 0.5;
//    sendGoal_and_wait_c3po(goal_c3po, ros::Duration(10));
//    //ros::Duration(10).sleep();

//    set_c3po_static_client_.call(req, res);
//  }

//  // Rotate both robots to the right 90 degrees
//  ROS_INFO("Rotate r2d2 90 degrees to the right");
//  goal_r2d2.move_x = 0;
//  goal_r2d2.rotation = -1.5708; //-90°
//  sendGoal_and_wait_r2d2(goal_r2d2, ros::Duration(10));

//  set_r2d2_static_client_.call(req, res);

//  ROS_INFO("Rotate c3po 90 degrees to the right");
//  goal_c3po.move_x = 0;
//  goal_c3po.rotation = -1.5708; //-90°
//  sendGoal_and_wait_c3po(goal_c3po, ros::Duration(10));

//  set_c3po_static_client_.call(req, res);


  state_ = Idle;
}

bool MomanaNode::check_c3po_local_move_server(void){
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

bool MomanaNode::check_r2d2_local_move_server(void){
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

bool MomanaNode::check_c3po_move_base_server(void){
  // check for c3po navigation stack move base server
  bool result = false;
  while(!result){
    ROS_INFO("Waiting 5s for c3po move_base server");
    result = ac_c3po_move_base_.waitForServer(ros::Duration(5));
    if(!result){
      ROS_INFO("Cannot find c3po move_base server");
    }
    if(state_==Idle){
      ROS_INFO("Exiting running thread..");
      state_==Idle;
      return false;
    }
  }
  ROS_INFO("c3po move_base server found");
  return true;
}

bool MomanaNode::check_r2d2_move_base_server(void){
  // check for r2d2 navigation stack move base server
  bool result = false;
  while(!result){
    ROS_INFO("Waiting 5s for r2d2 move_base server");
    result = ac_r2d2_move_base_.waitForServer(ros::Duration(5));
    if(!result){
      ROS_INFO("Cannot find r2d2 move_base server");
    }
    if(state_==Idle){
      ROS_INFO("Exiting running thread..");
      state_==Idle;
      return false;
    }
  }
  ROS_INFO("r2d2 move_base server found");
  return true;
}

void MomanaNode::sendGoal_and_wait_c3po(const robotino_local_move::LocalMoveGoal& goal, ros::Duration duration){
  ros::Time start_time = ros::Time::now();
  bool OdometryAvailable = true;

  ac_c3po_.sendGoal(goal);

  while(nh_.ok()){
    // wait for the action to return
    if (ac_c3po_.waitForResult(ros::Duration(1.0) ) ){
      actionlib::SimpleClientGoalState state = ac_c3po_.getState();
      ROS_INFO("C3PO Action finished: %s", state.toString().c_str());
      break;
    } else {
      ROS_INFO("C3PO Action did not finish before the time out.");
    }
    if( ( ros::Time::now() - start_time ) > duration )
    {
      ROS_INFO( "Timeout: Aborting Local move" );
      ac_c3po_.cancelAllGoals();
      break;
    }
    //OdometryAvailable = check_odometry();
    if(!OdometryAvailable){
      ROS_INFO( "Odometry Down: Aborting Local move" );
      ac_c3po_.cancelAllGoals();
      break;
    }
    ros::spinOnce();
  }
}

void MomanaNode::sendGoal_and_wait_r2d2(const robotino_local_move::LocalMoveGoal& goal, ros::Duration duration){
  std_srvs::Empty::Request req,res;
  ros::Time start_time = ros::Time::now();
  bool OdometryAvailable = true;

  ac_r2d2_.sendGoal(goal);

  while(nh_.ok()){
    // wait for the action to return
    if ( ac_r2d2_.waitForResult( ros::Duration(1.0) ) ) {
      actionlib::SimpleClientGoalState state = ac_r2d2_.getState();
      ROS_INFO("R2D2 Action finished: %s", state.toString().c_str());
      break;
    } else {
      ROS_INFO("R2D2 Action did not finish before the time out.");
    }
    if( ( ros::Time::now() - start_time ) > duration )
    {
      ROS_INFO( "Timeout: Aborting Local move" );
      ac_r2d2_.cancelAllGoals();
      break;
    }
    //OdometryAvailable = check_odometry();
    if(!OdometryAvailable){
      ROS_INFO( "Odometry Down: Aborting Local move" );
      ac_r2d2_.cancelAllGoals();
      break;
    }
    ros::spinOnce();
  }
}

void MomanaNode::sendGoal_and_wait_c3po(const move_base_msgs::MoveBaseGoal& goal, ros::Duration duration){
  std_srvs::Empty::Request req,res;
  ros::Time start_time = ros::Time::now();
  bool OdometryAvailable = true;

  //Before setting r2d2 static we enable filtering of the transform
  //and sleep for 1.5 seconds so the filtering is based on static positions of both robots
  //!TODO(racuna) ugly... find a better way right now it is based on my knowledge of the framerate
  //! and the time that it takes to do the switching in momanaodom
  filter_enable_client_.call(req, res);
  ros::Duration(1.5).sleep();

  set_r2d2_static_client_.call(req, res);

  ros::Duration(0.1).sleep();
  filter_disable_client_.call(req, res);

  ac_c3po_move_base_.sendGoal(goal);

  while(nh_.ok()){
    // wait for the action to return
    if (ac_c3po_move_base_.waitForResult(ros::Duration(1.0) ) ){
      actionlib::SimpleClientGoalState state = ac_c3po_move_base_.getState();
      ROS_INFO("C3PO Action finished: %s", state.toString().c_str());
      break;
    } else {
      ROS_INFO("C3PO Action did not finish before the time out.");
    }
    if( ( ros::Time::now() - start_time ) > duration )
    {
      ROS_INFO( "Timeout: Aborting move" );
      ac_c3po_move_base_.cancelAllGoals();
      break;
    }
    //OdometryAvailable = check_odometry();
    if(!OdometryAvailable){
      ROS_INFO( "Odometry Down: Aborting Local move" );
      ac_c3po_move_base_.cancelAllGoals();
      break;
    }
    ros::spinOnce();
  }
}

void MomanaNode::sendGoal_and_wait_r2d2(const move_base_msgs::MoveBaseGoal& goal, ros::Duration duration){
  std_srvs::Empty::Request req,res;
  ros::Time start_time = ros::Time::now();
  bool OdometryAvailable = true;
  //Before setting r2d2 static we enable filtering of the transform
  //and sleep for 1.5 seconds so the filtering is based on static positions of both robots
  //!TODO(racuna) ugly... find a better way right now it is based on my knowledge of the framerate
  //! and the time that it takes to do the switching in momanaodom
  filter_enable_client_.call(req, res);
  ros::Duration(1.5).sleep();

  set_c3po_static_client_.call(req, res);

  ros::Duration(0.1).sleep();
  filter_disable_client_.call(req, res);

  ac_r2d2_move_base_.sendGoal(goal);

  while(nh_.ok()){
    // wait for the action to return
    if (ac_r2d2_move_base_.waitForResult(ros::Duration(1.0) ) ){
      actionlib::SimpleClientGoalState state = ac_r2d2_move_base_.getState();
      ROS_INFO("R2D2 Action finished: %s", state.toString().c_str());
      break;
    } else {
      ROS_INFO("R2D2 Action did not finish before the time out.");
    }
    if( ( ros::Time::now() - start_time ) > duration )
    {
      ROS_INFO( "Timeout: Aborting move" );
      ac_r2d2_move_base_.cancelAllGoals();
      break;
    }
    //OdometryAvailable = check_odometry();
    if(!OdometryAvailable){
      ROS_INFO( "Odometry Down: Aborting Local move" );
      ac_r2d2_move_base_.cancelAllGoals();
      break;
    }
    ros::spinOnce();
  }
}

//! Implement Check ODOMETRY


void MomanaNode::square_navigation_test(double size_nav_square, double separation){
  // This function sends the commands to the robots so they navigate
  // a square of a given dimension meanwhile mantaining a given separation between them.
  // Parameters:
  // size_nav_square (m): Size of the square to navigate
  // separation (m): Distance to keep between robots.

  // Break distances in segments <= 1m (restriction of relative odometry)


  // Build waypoints
  geometry_msgs::PoseArray waypoints_robotA;
  geometry_msgs::PoseArray waypoints_robotB;
  geometry_msgs::Pose pose_robotA, pose_robotB;

  // initial pose of robots (IN MAP FRAME)
  // Robots aligned with MAP FRAME
  pose_robotA.orientation.w = pose_robotB.orientation.w = 1;

  //RobotA in this contex is c3po
  //RobotB in this contex is r2d2
  pose_robotA.position.x = 0;
  pose_robotA.position.y = 0;
  pose_robotA.position.z = 0;

  pose_robotB.position.x = 0;
  pose_robotB.position.y = pose_robotA.position.y - separation;
  pose_robotB.position.z = 0;

  //We define a set of displacements on x and y
  double dx[8]   = {0.5, 0.5,  0.0, 0.0, -0.5, -0.5, 0.0, 0.0};
  double dy[8]   = {0.0, 0.0, -0.5,-0.5,  0.0,  0.0, 0.5, 0.5};
  double dyaw[8] = {0, angles::from_degrees(-90), 0, angles::from_degrees(180),  0,  angles::from_degrees(90), 0, angles::from_degrees(0.1)};

  for (int i=0; i<8; i++){
    //Two movements and one rotation
    //First A and then B
    pose_robotA.position.x += dx[i];
    pose_robotA.position.y += dy[i];

    pose_robotB.position.x += dx[i];
    pose_robotB.position.y += dy[i];

    waypoints_robotA.poses.push_back(pose_robotA);
    waypoints_robotB.poses.push_back(pose_robotB);

    if(dyaw[i]){
      // Rotate 90 degrees clockwise
      pose_robotA.orientation = tf::createQuaternionMsgFromYaw(dyaw[i]); //-90°
      pose_robotB.orientation = tf::createQuaternionMsgFromYaw(dyaw[i]); //-90°
      waypoints_robotA.poses.push_back(pose_robotA);
      waypoints_robotB.poses.push_back(pose_robotB);
    }

  }




  // WE START MOVING
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "/map";
  std::vector<geometry_msgs::Pose>::iterator itA = waypoints_robotA.poses.begin();
  std::vector<geometry_msgs::Pose>::iterator itB = waypoints_robotB.poses.begin();

  int n_waypoints = waypoints_robotA.poses.size();
  for (int i = 0; i<n_waypoints; i++){
    if(i<=8){
      //Goal RobotB
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = waypoints_robotB.poses[i];
      sendGoal_and_wait_r2d2(goal, ros::Duration(20));

      //Goal RobotA
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = waypoints_robotA.poses[i];
      sendGoal_and_wait_c3po(goal, ros::Duration(20));
    }else{
      //When robots are returning they have to change order
      // so they dont collide
      ///!TODO (racuna)
      /// replace this ugly hack with proper inter-robot obstacle avoidance
      ///
      //Goal RobotA
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = waypoints_robotA.poses[i];
      sendGoal_and_wait_c3po(goal, ros::Duration(20));

      //Goal RobotB
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = waypoints_robotB.poses[i];
      sendGoal_and_wait_r2d2(goal, ros::Duration(20));
    }
  }






//  move_base_msgs::MoveBaseGoal goal;
//  goal.target_pose.header.frame_id = "/rel_r2d2/base_link";
//  goal.target_pose.header.stamp = ros::Time::now();
//  goal.target_pose.pose.position.x = 0.5;
//  goal.target_pose.pose.orientation.w = 1;
//  sendGoal_and_wait_r2d2(goal, ros::Duration(10));

//  goal.target_pose.header.frame_id = "/rel_c3po/base_link";
//  goal.target_pose.header.stamp = ros::Time::now();
//  sendGoal_and_wait_c3po(goal, ros::Duration(10));


//  //Move forward the distance given by size_nav_square
//  const double max_distance_segment = 1.0;
//  double partial_distance;
//  double remaining_distance = size_nav_square;
//  if (remaining_distance > 1.0){
//    remaining_distance =  size_nav_square - max_distance_segment;
//    partial_distance = max_distance_segment;
//  } else{
//    partial_distance = remaining_distance;
//  }

  //send command to robot 1 (based on previous position)
  // new_pos = old_pos + partial distance;

  //send command to robot 2 (based on previous position)
  // new_pos = old_pos + partial distance;







}
