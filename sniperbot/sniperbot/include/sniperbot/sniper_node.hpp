 /*
 * waiter_node.hpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#ifndef SNIPER_NODE_HPP_
#define SNIPER_NODE_HPP_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <kobuki_msgs/SensorState.h>
#include <kobuki_msgs/DigitalInputEvent.h>

#include <nonstop_msgs/Status.h>
#include <nonstop_msgs/Mission.h>
#include <nonstop_msgs/OperationalMissionAction.h>
#include <semantic_region_handler_nonstop/SitePoseList.h>

#include "sniperbot/ar_markers.hpp"
#include "sniperbot/nav_watchdog.hpp"
#include "sniperbot/navigator.hpp"

namespace sniperbot
{

class SniperNode
{
public:

  SniperNode(std::string name) :
    as_(nh_, "operation_mission", false),
    node_name_(name),
    SPOT_BASE_CAMP_MARKER_TIMEOUT(10.0),
    SPOT_POSE_MARKER_TIMEOUT(15.0),
    blink_frequency_(2.0),
    last_blink_time_(0.0),
    last_blink_led_(1)
  {
  }

  ~SniperNode(void)
  {
  }

  bool init();
  void spin();
  bool wakeUp();
  bool leaveNest();

  void odometryCB(const nav_msgs::Odometry::ConstPtr& msg);
  void digitalInputCB(const kobuki_msgs::DigitalInputEvent::ConstPtr& msg);
  void coreSensorsCB(const kobuki_msgs::SensorState::ConstPtr& msg);
  void sitePosesCB(const semantic_region_handler_nonstop::SitePoseList::ConstPtr& msg);

  void operationalMissionCB();
  void preemptMissionCB();

protected:
  const double SPOT_BASE_CAMP_MARKER_TIMEOUT;
  const double SPOT_POSE_MARKER_TIMEOUT;

  ros::NodeHandle nh_;
  std::string node_name_;

  // NodeHandle instance must be created before this line. Otherwise strange error may occur
  actionlib::SimpleActionServer<nonstop_msgs::OperationalMissionAction> as_;

  // create messages that are used to published feedback/result
  nonstop_msgs::OperationalMissionFeedback feedback_;
  nonstop_msgs::OperationalMissionResult   result_;

  /*********************
  ** Publishers
  **********************/
  ros::Publisher led_1_pub_;
  ros::Publisher led_2_pub_;
  ros::Publisher sound_pub_;
  ros::Publisher site_marker_pub_;

  /*********************
  ** Subscribers
  **********************/
  ros::Subscriber digital_input_sub_;
  ros::Subscriber core_sensors_sub_;
  ros::Subscriber site_poses_sub_;

  ARMarkers   ar_markers_;
  NavWatchdog nav_watchd_;
  Navigator   navigator_;

  geometry_msgs::PoseStamped             pickup_pose_;
  semantic_region_handler_nonstop::SitePoseList site_poses_;
  kobuki_msgs::SensorState core_sensors_;
  nonstop_msgs::Mission mission_;
  std::string global_frame_;

  // LED blinking attributes; TODO make a separate HRI class
  double blink_frequency_;
  double last_blink_time_;
  uint8_t last_blink_led_;

  boost::thread mission_process_thread_;

  bool debug_mode_;
  bool initialized_;
  bool initialized_site_;

  bool wait_for_button_;

  bool processMission(nonstop_msgs::Mission& mission);
  bool getReadyToWork();
  bool waitForPoses();
  bool waitForButton();
  bool gotoSite(int site_id);
  void sendFeedback(int feedback_status);
  bool setSucceeded(std::string message);
  bool setFailure(std::string message);

  bool cleanupAndSuccess();
  bool cleanupAndError();

  const std::string toStr(int16_t status)
  {
    return std::string(toCStr(status));
  }

  const char* toCStr(int16_t status)
  {
    switch (status)
    {
      case nonstop_msgs::Status::IDLE                          : return "idle";
      case nonstop_msgs::Status::GO_TO_BASE_CAMP                 : return "going to kitchen";
      case nonstop_msgs::Status::ARRIVE_BASE_CAMP                : return "arrived to kitchen";
      case nonstop_msgs::Status::WAITING_FOR_BASE_CAMP           : return "waiting for kitchen";
      case nonstop_msgs::Status::IN_OPERATION                   : return "going to site";
      case nonstop_msgs::Status::ARRIVE_SITE                  : return "arrived to site";
      case nonstop_msgs::Status::WAITING_FOR_COMMANDER_CONFIRMATION : return "waiting for customer";
      case nonstop_msgs::Status::COMPLETE_OPERATION             : return "operation completed";
      case nonstop_msgs::Status::RETURNING_TO_DOCK             : return "going to base";
      case nonstop_msgs::Status::END_OPERATION_MISSION            : return "mission completed";
      case nonstop_msgs::Status::ERROR                         : return "error";
      default                                               : return "UNRECOGNIZED STATUS";
    }
  }

  //< DEBUG
  void fakeMissionForEasyDebugging(int mission_id, int site_id);
  //> DEBUG
};

} /* namespace sniperbot */

#endif /* SNIPER_NODE_HPP_ */
