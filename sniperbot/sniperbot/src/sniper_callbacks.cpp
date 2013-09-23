/*
 * waiter_callbacks.cpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#include <visualization_msgs/MarkerArray.h>

#include "sniperbot/sniper_node.hpp"

namespace sniperbot
{

void SniperNode::sitePosesCB(const semantic_region_handler_nonstop::SitePoseList::ConstPtr& msg)
{
  // Just take first message; ignore the rest, as global markers list is not dynamic
  if ((site_poses_.sites.size() == 0) && (msg->sites.size() > 0))
  {
    // We will also publish site markers to help visualizing single robot navigation in a rocon environment
    visualization_msgs::MarkerArray markers_array;

    site_poses_ = *msg;
    ROS_INFO("%lu site pose(s) received", site_poses_.sites.size());
    for (unsigned int i = 0; i < site_poses_.sites.size(); i++)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = ros::Time::now();
      marker.ns = site_poses_.sites[i].name;
      marker.id = i;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = site_poses_.sites[i].radius * 2.0;
      marker.scale.y = site_poses_.sites[i].radius * 2.0;
      marker.scale.z = 0.1;
      marker.pose = site_poses_.sites[i].pose_cov_stamped.pose.pose;
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 0.5f;
      markers_array.markers.push_back(marker);

      // Look for the reload point
      if (site_poses_.sites[i].name.find("reload") != std::string::npos)
      {
        ROS_DEBUG("reload point: %s", tk::pose2str(site_poses_.sites[i].pose_cov_stamped.pose.pose));
        pickup_pose_.header = site_poses_.sites[i].pose_cov_stamped.header;
        pickup_pose_.pose = site_poses_.sites[i].pose_cov_stamped.pose.pose;
      }
      else
      {
        ROS_DEBUG("%s. rad: %f, pose: %s", site_poses_.sites[i].name.c_str(), site_poses_.sites[i].radius,
                  tk::pose2str(site_poses_.sites[i].pose_cov_stamped.pose.pose));
      }
    }

    initialized_site_ = true;

    // Is a latched topic, so we just need to publish once
    if (markers_array.markers.size() > 0)
      site_marker_pub_.publish(markers_array);
  }
}

void SniperNode::digitalInputCB(const kobuki_msgs::DigitalInputEvent::ConstPtr& msg)
{
  // TODO msg->values[1] red button -> use to stop action in course;
  // by now just close the current action with failure, so we can accept new missions; but we cannot cancel current task
  if ((msg->values[1] == false) && (mission_.status == nonstop_msgs::Status::ERROR))
  {
    // Return the result to Task Coordinator
    ROS_INFO("This is embarrassing... looks like I have being rescued by a human...   %d", as_.isActive());
    nonstop_msgs::OperationalMissionResult result;
    result.result = "Robot manually recovered";
    as_.setAborted(result);
    mission_.status = nonstop_msgs::Status::IDLE;

    // assume robot is not localized
    initialized_ = false;
  }

  if (msg->values[0] == false)
    wait_for_button_ = false;
}

void SniperNode::coreSensorsCB(const kobuki_msgs::SensorState::ConstPtr& msg)
{
  core_sensors_ = *msg;
}

void SniperNode::operationalMissionCB()
{
  if (mission_.status != nonstop_msgs::Status::IDLE)
  {
    ROS_WARN("sniperbot not in idle status; cannot attend request (current status is %s)", toCStr(mission_.status));
//    return;
    // TODO how the hell I inform of this to the task coordinator???
  }

  // Sccept the new goal
  mission_ = as_.acceptNewGoal()->mission;
  ROS_INFO("Operational mission action requested [mission: %d, site: %d]", mission_.mission_id, mission_.site_id);

  //< DEBUG  Fake missions for evaluating individual tasks
  if ((debug_mode_ == true) && (mission_.mission_id < 0))
  {
    fakeMissionForEasyDebugging(mission_.mission_id * -1, mission_.site_id);
    // Return the result to Task Coordinator
    nonstop_msgs::OperationalMissionResult result;
    result.result = "VAMONOS!!!!";
    as_.setSucceeded(result);
    return;
  }
  //> DEBUG

  // starts a thread to process mission
  mission_process_thread_ = boost::thread(&SniperNode::processMission, this, mission_);
}

void SniperNode::preemptMissionCB()
{
  ROS_WARN("Current mission preempted [mission: %d, site: %d] (current status is %s)",
           mission_.mission_id, mission_.site_id, toCStr(mission_.status));
  // set the action state to preempted
  //  TODO WE REALLY WANT???  as_.setPreempted();
  as_.setPreempted();
}


//< DEBUG
void SniperNode::fakeMissionForEasyDebugging(int mission_id, int site_id)
{
  ROS_INFO("FAKE operational mission action requested [mission: %d, site: %d]", mission_id, site_id);

  if (mission_id == 11)       ar_markers_.setTrackerFreq(5);
  if (mission_id == 22)       ar_markers_.setTrackerFreq(10);

  if (mission_id == 33)       ar_markers_.setTrackerFreq(0);
//  return;
//  if (mission_id == 1)       mission_.status = nonstop_msgs::Status::ERROR;
//  if (mission_id == 2)       mission_.status = nonstop_msgs::Status::WAITING_FOR_KITCHEN;
//  if (mission_id == 3)       mission_.status = nonstop_msgs::Status::WAITING_FOR_USER_CONFIRMATION;
//  if (mission_id == 4)       mission_.status = nonstop_msgs::Status::IDLE;

  if (mission_id == 1)       boost::thread wakeUpThread(&SniperNode::wakeUp, this);
  if (mission_id == 2)       boost::thread wakeUpThread(&SniperNode::leaveNest, this);
  if (mission_id == 3)       boost::thread dockingThread(boost::bind(&Navigator::dockInBase, &navigator_, ar_markers_.getDockingBasePose()));
  if (mission_id == 4)       boost::thread dockingThread(boost::bind(&Navigator::dockInBase, &navigator_));
  if (mission_id == 5)       boost::thread pickUpThread(&Navigator::pickUpMission, &navigator_, pickup_pose_);
  if (mission_id == 6)
  {
    bool site_found = false;
    for (unsigned int i = 0; i < site_poses_.sites.size(); i++)
    {
      // Look for the requested site's pose (and get rid of the useless covariance)
      if (site_poses_.sites[i].name.find(tk::nb2str(site_id), strlen("site")) != std::string::npos)
      {
        ROS_DEBUG("Target site %d: rad = %f, pose = %s", site_id, site_poses_.sites[i].radius,
                  tk::pose2str(site_poses_.sites[i].pose_cov_stamped.pose.pose));
        geometry_msgs::PoseStamped site_pose;
        site_pose.header = site_poses_.sites[i].pose_cov_stamped.header;
        site_pose.pose = site_poses_.sites[i].pose_cov_stamped.pose.pose;

        boost::thread pickUpThread(&Navigator::operationMission, &navigator_,
                                   site_pose, site_poses_.sites[i].radius);
        site_found = true;
        break;
      }
    }

    if (site_found == false)
    {
      ROS_WARN("Site %d not found! ignoring mission", site_id);
    }
  }
  if (mission_id == 666)     boost::thread kk(&Navigator::moveBaseReset, &navigator_);
  if (mission_id == 8)       boost::thread kk(&Navigator::turn, &navigator_, M_PI*1.5);
  if (mission_id == 9)       boost::thread kk(&Navigator::turn, &navigator_, -M_PI*0.5);
  if (mission_id == 10)      ar_markers_.enableTracker();
  if (mission_id == 11)      ar_markers_.disableTracker();
  if (mission_id == 13)      ar_markers_.setTrackerFreq(11);
  if (mission_id == 14)      ar_markers_.setTrackerFreq(3);
}
//> DEBUG


} // namespace sniperbot
