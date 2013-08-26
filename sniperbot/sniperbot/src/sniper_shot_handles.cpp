/*
 * sniper_shot_handles.cpp
 *
 *  Created on: May, 2013
 *      Author: Jihoon
 */

#include "sniperbot/sniper_node.hpp"

namespace sniperbot
{

bool SniperNode::processMission(nonstop_msgs::Mission& mission)
{
  // wait for semantic pose initialization
  waitForPoses();

  // 0. wakeup or leave nest SniperNode::wakeUp,leaveNest
  sendFeedback(nonstop_msgs::Status::GO_TO_BASE_CAMP);
  ros::Duration(1).sleep();
  if (getReadyToWork() == false)
  {
    return setFailure("Sniper failed to get ready to work");
  }

  // 1. goto reload place Navigator::reloadMission
  if (navigator_.reloadMission(reload_pose_) == false)
  {
    return setFailure("Sniper failed to go to reload place");
  }
  sendFeedback(nonstop_msgs::Status::ARRIVE_BASE_CAMP);
  ros::Duration(1).sleep();
  sendFeedback(nonstop_msgs::Status::WAITING_FOR_BASE_CAMP);
  ros::Duration(1).sleep();
  
  // 2. Wait for button
  if (waitForButton() == false)
  {
    return setFailure("Sniper didn't receive the button from basecamp");
  }
  sendFeedback(nonstop_msgs::Status::IN_OPERATION);

  // 3. goto site     Navigator::deliverMission
  if (gotoTable(mission.site_id) == false)
  {
    return setFailure("Sniper failed to go to site");
  }
  sendFeedback(nonstop_msgs::Status::ARRIVE_SITE);
  ros::Duration(1).sleep();
  sendFeedback(nonstop_msgs::Status::WAITING_FOR_COMMANDER_CONFIRMATION);
  ros::Duration(1).sleep();

  // 4. wait for button
  if (waitForButton() == false)
  {
    return setFailure("Sniper didn't receive the confirm from commander");
  }
  sendFeedback(nonstop_msgs::Status::COMPLETE_OPERATION);
  ros::Duration(1).sleep();
  sendFeedback(nonstop_msgs::Status::RETURNING_TO_DOCK);
  ros::Duration(1).sleep();

  // 5. return to dock Navigator::dockInBase
  if (navigator_.dockInBase(ar_markers_.getDockingBasePose()) == false)
  {
    return setFailure("Sniper failed to go back to nest");
  }

  sendFeedback(nonstop_msgs::Status::END_OPERATION_ORDER);
  ros::Duration(1).sleep();

  return setSucceeded("Operation successfully completed (hopefully...)");
}

bool SniperNode::setSucceeded(std::string message)
{
  // Return the result to Task Coordinator
  ROS_INFO_STREAM(message);
  nonstop_msgs::OperationalMissionResult result;
  result.result = message;
  as_.setSucceeded(result);
  order_.status = nonstop_msgs::Status::IDLE;

  return true;
}

bool SniperNode::setFailure(std::string message)
{
  // Return the result to Task Coordinator
  ROS_ERROR_STREAM(message);
  nonstop_msgs::OperationalMissionResult result;
  result.result = message;
//  as_.setAborted(result);

  // NEW POLITICS:  we don't close the action until we are in the docking base, ready to take a new mission, or someone press the red button (manual recovery)

  // Try to go back to nest   TODO  a better feedback would be RECOVERING
  sendFeedback(nonstop_msgs::Status::ERROR);

  bool at_base;
  ROS_ERROR("Something went wrong while processing mission; try to go back to nest...");
  if (ar_markers_.dockingBaseSpotted() == true)
    at_base = navigator_.dockInBase(ar_markers_.getDockingBasePose());
  else
    at_base = navigator_.dockInBase();

  if (at_base == false)
  {
    ROS_ERROR("Go back to nest failed; we don't have a recovery mechanism, so... please put me on my nest and press the red button to notify TC that I'm ready again");
    order_.status = nonstop_msgs::Status::ERROR;
  }
  else
  {
    order_.status = nonstop_msgs::Status::IDLE;
    as_.setAborted(result);
  }

  return at_base;
}

void SniperNode::sendFeedback(int feedback_status)
{
  nonstop_msgs::OperationalMissionFeedback feedback;

//  ROS_DEBUG("Sending Feedback %d", feedback_status);
  feedback.status = feedback_status;
  as_.publishFeedback(feedback);

  order_.status = feedback_status;
}

} // namespace sniperbot
