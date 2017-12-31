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
  private_nh.param<std::string>("drone_yaw_to_look_str",yaw_to_look_str, "droneYawToLook");
  private_nh.param<std::string>("drone_yaw_ref",drone_yaw_ref_str,"droneControllerYawRefCommand");
  private_nh.param<std::string>("drone_control_mode",drone_control_mode_str,"droneTrajectoryController/controlMode");
  private_nh.param<std::string>("consult_belief",execute_query_srv,"consult_belief");
}

void BehaviorLookAtUAV::ownStart()
{
  std::cout << "ownStart" << std::endl;
  is_finished = false;

  /*Initialize topics*/
  estimated_pose_sub = node_handle.subscribe(estimated_pose_str, 1000, &BehaviorLookAtUAV::estimatedPoseCallBack, this);
  estimated_speed_sub = node_handle.subscribe(estimated_speed_str, 1000, &BehaviorLookAtUAV::estimatedSpeedCallback, this);
  rotation_angles_sub = node_handle.subscribe(rotation_angles_str, 1000, &BehaviorLookAtUAV::rotationAnglesCallback, this);
  controllers_pub = node_handle.advertise<droneMsgsROS::droneCommand>(controllers_str, 1, true);
  yaw_controller_pub=node_handle.advertise<droneMsgsROS::droneYawRefCommand>(yaw_controller_str,1000);
  mode_service=node_handle.serviceClient<droneMsgsROS::setControlMode>(service_topic_str);
  drone_position_pub=node_handle.advertise< droneMsgsROS::dronePositionRefCommandStamped>(drone_position_str,1000);
  yaw_command_pub=node_handle.advertise<droneMsgsROS::droneYawRefCommand>(drone_yaw_ref_str,1000);
  query_client = node_handle.serviceClient <droneMsgsROS::ConsultBelief> (execute_query_srv);

  estimated_pose_msg = *ros::topic::waitForMessage<droneMsgsROS::dronePose>(estimated_pose_str, node_handle, ros::Duration(2));
  /*
   * get arguments
   *
   * enter speed_control mode, enter move
   */
}

void BehaviorLookAtUAV::ownRun()
{
  /*
   * calculate yaw diff
   *
   * calculate speeds
   *
   * send speed in dYawCmd
   */
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

void BehaviorLookAtUAV::estimatedSpeedCallback(const droneMsgsROS::droneSpeeds& msg)
{
estimated_speed_msg=msg;
}
void BehaviorLookAtUAV::estimatedPoseCallBack(const droneMsgsROS::dronePose& msg)
{
estimated_pose_msg=msg;
}
void BehaviorLookAtUAV::rotationAnglesCallback(const geometry_msgs::Vector3Stamped& msg)
{
rotation_angles_msg=msg;
}