/*!*******************************************************************************************
 *  \file       behavior_rotate.cpp
 *  \brief      Behavior Rotate implementation file.
 *  \details    This file implements the behaviorRotate class.
 *  \authors    Rafael Artiñano Muñoz
 *  \copyright  Copyright 2016 Universidad Politecnica de Madrid (UPM)
 *
 *     This program is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program. If not, see http://www.gnu.org/licenses/.
 ********************************************************************************/
#include "../include/behavior_look_at_uav.h"
BehaviorLookAtUAV::BehaviorLookAtUAV():BehaviorProcess()
{

}

BehaviorLookAtUAV::~BehaviorLookAtUAV()
{

}

void BehaviorLookAtUAV::ownSetUp()
{
  safetyR0=0.75;
  safetyR1=1.5;
  speed = 5;
 std::cout << "ownSetup" << std::endl;

 ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("drone_id", drone_id, "1");
  private_nh.param<std::string>("drone_id_namespace", drone_id_namespace, "drone"+drone_id);
  private_nh.param<std::string>("my_stack_directory", my_stack_directory,
                  "~/workspace/ros/quadrotor_stack_catkin/src/quadrotor_stack");
  private_nh.param<std::string>("estimated_pose_topic", estimated_pose_str, "estimated_pose");
  private_nh.param<std::string>("controllers_topic", controllers_str, "command/high_level");
  private_nh.param<std::string>("rotation_angles_topic", rotation_angles_str, "rotation_angles");
  private_nh.param<std::string>("estimated_speed_topic",estimated_speed_str,"estimated_speed");
  private_nh.param<std::string>("yaw_controller_str",yaw_controller_str , "droneControllerYawRefCommand");
  private_nh.param<std::string>("service_topic_str",service_topic_str , "droneTrajectoryController/setControlMode");
  private_nh.param<std::string>("drone_position_str",drone_position_str , "dronePositionRefs");
  private_nh.param<std::string>("speed_topic",speed_topic_str , "droneSpeedsRefs");

  private_nh.param<std::string>("drone_yaw_to_look_str",yaw_to_look_str, "droneYawToLook");
  private_nh.param<std::string>("drone_yaw_ref",drone_yaw_ref_str,"droneControllerYawRefCommand");
  private_nh.param<std::string>("drone_control_mode",drone_control_mode_str,"droneTrajectoryController/controlMode");
  private_nh.param<std::string>("d_yaw",d_yaw_str,"command/dYaw");
  private_nh.param<std::string>("consult_belief",execute_query_srv,"consult_belief");
}

void BehaviorLookAtUAV::ownStart()
{
  std::cout << "ownStart" << std::endl;
  is_finished = false;
  state = 0;

  /*Initialize topics*/
  estimated_pose_sub = node_handle.subscribe(estimated_pose_str, 1000, &BehaviorLookAtUAV::estimatedPoseCallBack, this);
  estimated_speed_sub = node_handle.subscribe(estimated_speed_str, 1000, &BehaviorLookAtUAV::estimatedSpeedCallback, this);
  rotation_angles_sub = node_handle.subscribe(rotation_angles_str, 1000, &BehaviorLookAtUAV::rotationAnglesCallback, this);
  controllers_pub = node_handle.advertise<droneMsgsROS::droneCommand>(controllers_str, 1, true);
  yaw_controller_pub=node_handle.advertise<droneMsgsROS::droneYawRefCommand>(yaw_controller_str,1000);
  mode_service=node_handle.serviceClient<droneMsgsROS::setControlMode>(service_topic_str);
  drone_position_pub=node_handle.advertise< droneMsgsROS::dronePositionRefCommandStamped>(drone_position_str,1000);
  yaw_command_pub=node_handle.advertise<droneMsgsROS::droneYawRefCommand>(drone_yaw_ref_str,1000);
  d_yaw_pub = node_handle.advertise<droneMsgsROS::droneDYawCmd>(d_yaw_str,1);
  query_client = node_handle.serviceClient <droneMsgsROS::ConsultBelief> (execute_query_srv);
  speed_topic_pub=node_handle.advertise<droneMsgsROS::droneSpeeds>(speed_topic_str,1000);

  estimated_pose_msg = *ros::topic::waitForMessage<droneMsgsROS::dronePose>(estimated_pose_str, node_handle, ros::Duration(2));
  target_position.x = estimated_pose_msg.x;
  target_position.y = estimated_pose_msg.y;
  target_position.z = estimated_pose_msg.z;
  //get arguments
  std::string arguments=getArguments();
  YAML::Node config_file = YAML::Load(arguments);

  //get leader ID
  if(config_file["droneID"].IsDefined()){
    leaderID=config_file["droneID"].as<int>();

    estimated_leader_pose_str   = std::string("/drone") + std::to_string(leaderID) + std::string("/estimated_pose");
    estimated_leader_speed_str  = std::string("/drone") + std::to_string(leaderID) + std::string("/estimated_speed");
    estimated_leader_pose_sub = node_handle.subscribe(estimated_leader_pose_str, 1000, &BehaviorLookAtUAV::estimatedLeaderPoseCallBack, this);
    estimated_leader_speed_sub = node_handle.subscribe(estimated_leader_speed_str, 1000, &BehaviorLookAtUAV::estimatedLeaderSpeedCallback, this);

  }
  else{
    setStarted(false);
    return;
  }
  //subsctibe leaders pose:

  droneMsgsROS::setControlMode mode;
  mode.request.controlMode.command=mode.request.controlMode.SPEED_CONTROL;
  mode_service.call(mode);

  ros::topic::waitForMessage<droneMsgsROS::droneTrajectoryControllerControlMode>(
    drone_control_mode_str, node_handle
  );

  droneMsgsROS::droneCommand msg;
  msg.command = droneMsgsROS::droneCommand::MOVE;
  controllers_pub.publish(msg);
}

void BehaviorLookAtUAV::ownRun()
{
  switch(state){
  case 0:{
    std::cout << "state 0" << std::endl;
    double intruderDistanceXY = sqrt(pow(estimated_leader_pose_msg.x-estimated_pose_msg.x,2)
                           + pow(estimated_leader_pose_msg.y-estimated_pose_msg.y,2));
    double distance_base = sqrt(pow(static_pose.x-estimated_pose_msg.x,2)
                           + pow(static_pose.y-estimated_pose_msg.y,2));
    if(intruderDistanceXY < safetyR0){
      state = 1;
      break;
    }
    else if (intruderDistanceXY < safetyR1){
      state = 2;
      break;
    }
    droneMsgsROS::droneSpeeds droneSpeed;
    droneSpeed.dx   = 0.0;
    droneSpeed.dy   = 0.0;
    droneSpeed.dz   = 0.0;
    if (distance_base > 0.1){
      droneSpeed.dx = speed * (target_position.x - estimated_pose_msg.x) / distance_base;
      droneSpeed.dy = speed * (target_position.y - estimated_pose_msg.y) / distance_base;
      droneSpeed.dz = 1.0 * (target_position.z - estimated_pose_msg.z);
    }
    droneSpeed.dyaw = calculateDYaw();

    speed_topic_pub.publish(droneSpeed);

    break;
  }
  case 1:{
    std::cout << "state 1" << std::endl;
    double intruderDistanceXY = sqrt(pow(estimated_leader_pose_msg.x-estimated_pose_msg.x,2)
                           + pow(estimated_leader_pose_msg.y-estimated_pose_msg.y,2));
    if(intruderDistanceXY > safetyR1){ // start working in normal way
      state = 0;
      break;
    }
    else if(intruderDistanceXY > safetyR0){ // stop movement
      state = 2;
      break;
    }

    float yIntruderEstimate = estimated_leader_pose_msg.y + (estimated_leader_speed_msg.dy/estimated_leader_speed_msg.dx) *
        (estimated_pose_msg.x - estimated_leader_pose_msg.x);
    float droneDirection;
    float temp_dx, temp_dy;
    droneMsgsROS::droneSpeeds droneSpeed;

    if (estimated_leader_speed_msg.dy > 0.1 && estimated_leader_speed_msg.dx > 0.1){
      droneDirection = (-1) * (estimated_leader_speed_msg.dx/estimated_leader_speed_msg.dy);
      if (yIntruderEstimate > estimated_pose_msg.y){
        temp_dy = (-1) * std::abs(droneDirection);
      }
      else{
        temp_dy = std::abs(droneDirection);
      }
      temp_dx = temp_dy / droneDirection;
      float temp_length = sqrt(pow(temp_dx,2) + pow(temp_dy,2));
      droneSpeed.dx = speed * temp_dx/temp_length;
      droneSpeed.dy = speed * temp_dy/temp_length;
    }
    else if (estimated_leader_speed_msg.dy < 0.1 && estimated_leader_speed_msg.dx < 0.1){
      float targetYaw = atan2(target_position.y-estimated_pose_msg.y,target_position.x-estimated_pose_msg.x);
      float intruderYaw = atan2(estimated_leader_pose_msg.y-estimated_pose_msg.y,estimated_leader_pose_msg.x-estimated_pose_msg.x);
      float movementYaw;
      if (std::abs(targetYaw - intruderYaw) < M_PI/2){
        if (targetYaw - intruderYaw > 0 ) movementYaw = targetYaw + (M_PI/2 - std::abs(targetYaw - intruderYaw));
        else movementYaw = targetYaw - (M_PI/2 - std::abs(targetYaw - intruderYaw));
      }
      else movementYaw = targetYaw;
      droneSpeed.dx = speed * cos(movementYaw);
      droneSpeed.dy = speed * sin(movementYaw);
      }
    else if (estimated_leader_speed_msg.dy < 0.1){
      if (estimated_leader_pose_msg.x < estimated_pose_msg.x){
        droneSpeed.dx = speed;
      }
      else{
        droneSpeed.dx = (-1.0) * speed;
      }
      droneSpeed.dy = 0.0;
    }
    else{
      if (estimated_leader_pose_msg.y < estimated_pose_msg.y){
        droneSpeed.dy = speed;
      }
      else{
        droneSpeed.dy = (-1.0) * speed;
      }
      droneSpeed.dx = 0.0;
    }

    droneSpeed.dz = 0.0;
    droneSpeed.dyaw = calculateDYaw();
    speed_topic_pub.publish(droneSpeed);
    break;

  }
  case 2:{
    std::cout << "state 2" << std::endl;
    double intruderDistanceXY = sqrt(pow(estimated_leader_pose_msg.x-estimated_pose_msg.x,2)
                           + pow(estimated_leader_pose_msg.y-estimated_pose_msg.y,2));
    if(intruderDistanceXY < safetyR0){ // start escape
      state = 1;
      break;
    }
    else if(intruderDistanceXY > safetyR1){ // start movement in normal way
      state = 0;
      break;
    }
    float yIntruderEstimate = estimated_leader_pose_msg.y + (estimated_leader_speed_msg.dy/estimated_leader_speed_msg.dx) *
        (estimated_pose_msg.x - estimated_leader_pose_msg.x);
    float droneDirection;
    float escapeSpeed = speed * (1 - 0.5 * (intruderDistanceXY - safetyR0)/(safetyR1 - safetyR0));
    float temp_dx, temp_dy;
    droneMsgsROS::droneSpeeds droneSpeed;

    if (estimated_leader_speed_msg.dy > 0.1 && estimated_leader_speed_msg.dx > 0.1){
      droneDirection = (-1) * (estimated_leader_speed_msg.dx/estimated_leader_speed_msg.dy);
      if (yIntruderEstimate > estimated_pose_msg.y){
        temp_dy = (-1) * std::abs(droneDirection);
      }
      else{
        temp_dy = std::abs(droneDirection);
      }
      temp_dx = temp_dy / droneDirection;
      float temp_length = sqrt(pow(temp_dx,2) + pow(temp_dy,2));
      droneSpeed.dx = escapeSpeed * temp_dx/temp_length;
      droneSpeed.dy = escapeSpeed * temp_dy/temp_length;
    }
    else if (estimated_leader_speed_msg.dy < 0.1 && estimated_leader_speed_msg.dx < 0.1){
      float targetYaw = atan2(target_position.y-estimated_pose_msg.y,target_position.x-estimated_pose_msg.x);
      float intruderYaw = atan2(estimated_leader_pose_msg.y-estimated_pose_msg.y,estimated_leader_pose_msg.x-estimated_pose_msg.x);
      float movementYaw;
      if (std::abs(targetYaw - intruderYaw) < M_PI/2){
        if (targetYaw - intruderYaw > 0 ) movementYaw = targetYaw + (M_PI/2 - std::abs(targetYaw - intruderYaw));
        else movementYaw = targetYaw - (M_PI/2 - std::abs(targetYaw - intruderYaw));
      }
      else movementYaw = targetYaw;
      droneSpeed.dx = escapeSpeed * cos(movementYaw);
      droneSpeed.dy = escapeSpeed * sin(movementYaw);
      }
    else if (estimated_leader_speed_msg.dy < 0.1){
      if (estimated_leader_pose_msg.x < estimated_pose_msg.x){
        droneSpeed.dx = escapeSpeed;
      }
      else{
        droneSpeed.dx = (-1.0) * escapeSpeed;
      }
      droneSpeed.dy = 0.0;
    }
    else{
      if (estimated_leader_pose_msg.y < estimated_pose_msg.y){
        droneSpeed.dy = speed;
      }
      else{
        droneSpeed.dy = (-1.0) * escapeSpeed;
      }
      droneSpeed.dx = 0.0;
    }

    droneSpeed.dz = 0.0;
    droneSpeed.dyaw = calculateDYaw();
    speed_topic_pub.publish(droneSpeed);
    break;
  }
  }
}
std::tuple<bool,std::string> BehaviorLookAtUAV::ownCheckSituation()
{
  droneMsgsROS::ConsultBelief query_service;
  std::ostringstream capturador;
  capturador << "battery_level(self,LOW)";
  std::string query(capturador.str());
  query_service.request.query = query;
  query_client.call(query_service);
  if(query_service.response.success)
  {
    return std::make_tuple(false,"Error: Battery low, unable to perform action");
    //return false;
  }
  std::ostringstream capturador2;
  capturador2<<"flight_state(self,LANDED)";
  std::string query2(capturador2.str());
  query_service.request.query = query2;
  query_client.call(query_service);
  if(query_service.response.success)
  {
    return std::make_tuple(false,"Error: Drone landed");
    //return false;
  }

  return std::make_tuple(true,"");
  //return true;
}
void BehaviorLookAtUAV::ownStop()
{
  droneMsgsROS::droneCommand msg;
  msg.command = droneMsgsROS::droneCommand::HOVER;
  controllers_pub.publish(msg);
  estimated_pose_sub.shutdown();
  rotation_angles_sub.shutdown();
  estimated_speed_sub.shutdown();
  // staticity_timer.stop();
}


float BehaviorLookAtUAV::calculateDYaw(){

  float dYaw;
  float setpoint_yaw=atan2(estimated_leader_pose_msg.y-estimated_pose_msg.y,estimated_leader_pose_msg.x-estimated_pose_msg.x);
  float current_yaw = fmod(estimated_pose_msg.yaw + 2*M_PI, 2*M_PI);
  float yaw_diff = fmod((setpoint_yaw - current_yaw)+2*M_PI,2*M_PI);

  if(std::abs(yaw_diff) > 0.1 && std::abs(yaw_diff) < (2*M_PI - 0.1)){
    dYaw = (-2.0) * (fmod((yaw_diff/M_PI + 1),2)-1);
    //calculate dYaw speed
  }
  else{
    dYaw = 0;
  }

  std::cout << "setpoint_yaw = " << setpoint_yaw * 180/M_PI << std::endl;
  std::cout << "current_yaw = " << current_yaw * 180/M_PI<< std::endl;
  std::cout << "yaw_diff = " << yaw_diff * 180/M_PI<< std::endl;
  std::cout << "dYaw = " << dYaw << std::endl;
  return dYaw;
}

void BehaviorLookAtUAV::estimatedSpeedCallback(const droneMsgsROS::droneSpeeds& msg)
{
estimated_speed_msg=msg;
}
void BehaviorLookAtUAV::estimatedPoseCallBack(const droneMsgsROS::dronePose& msg)
{
estimated_pose_msg=msg;
}
void BehaviorLookAtUAV::estimatedLeaderPoseCallBack(const droneMsgsROS::dronePose& msg){
  estimated_leader_pose_msg=msg;
}
void BehaviorLookAtUAV::rotationAnglesCallback(const geometry_msgs::Vector3Stamped& msg)
{
rotation_angles_msg=msg;
}
void BehaviorLookAtUAV::estimatedLeaderSpeedCallback(const droneMsgsROS::droneSpeeds& msg){
  estimated_leader_speed_msg=msg;
}
