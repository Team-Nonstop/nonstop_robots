#include <sstream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"
#include "tf/tf.h"

#include <yaml-cpp/yaml.h>
#include "nonstop_drone_marker_tracking/tracking_marker_data.hpp"
#include <fstream>

ros::Publisher cmd_vel_pub;
ros::Publisher target_pose_pub;
ros::Subscriber sub;

tf::TransformListener *m_tfListener;
//tf::TransformBroadcaster *m_tfBroadcaster;
std::string yaml_file;

geometry_msgs::PoseStamped target_pose;
geometry_msgs::Twist cmd_vel;

std::string itoa(int n){
	std::stringstream ss;
	ss << n;
	return ss.str();
}


int getParamCnt(){
	std::ifstream ifs(yaml_file.c_str(), std::ifstream::in);

	if (ifs.good() == false)
	{
		ROS_ERROR("configuration file not found [%s]", yaml_file.c_str());
		return 0;
	}

	YAML::Parser parser(ifs);
	YAML::Node doc;
	parser.GetNextDocument(doc);

	return (int)doc.size();
}

void matchParam(int marker_id, int &marker_exist){
  std::ifstream ifs(yaml_file.c_str(), std::ifstream::in);

  if (ifs.good() == false)
    {
      ROS_ERROR("configuration file not found [%s]", yaml_file.c_str());
      return;
    }
  else
    ROS_WARN("YAML File found");

  YAML::Parser parser(ifs);
  YAML::Node doc;
  parser.GetNextDocument(doc);

  ROS_WARN("Param config counter : %d", (int)doc.size());


      MarkerMarkData mmd;

      mmd.configure(doc["mark_data"]);
      for(int j = 0; j < mmd.size(); j++){
          if(std::atoi(mmd[j].marker_id.c_str()) == marker_id){
            marker_exist = 1;
          }else{
            marker_exist = 0;
          }
  }
}

void ReceiveCallback(const visualization_msgs::Marker mrk)
{
  std::ifstream ifs(yaml_file.c_str(), std::ifstream::in);

  if (ifs.good() == false)
    {
      ROS_ERROR("configuration file not found [%s]", yaml_file.c_str());
      return;
    }
  else
    ROS_WARN("YAML File found");

  YAML::Parser parser(ifs);
  YAML::Node doc;
  parser.GetNextDocument(doc);

  const YAML::Node *drone_frameid_node = doc.FindValue("drone_frameid");
  const YAML::Node *camera_frameid_node = doc.FindValue("camera_frame_id");
  const YAML::Node *marker_frameid_node = doc.FindValue("marker_frame_id");
  const YAML::Node *tracking_sensivity_node = doc.FindValue("tracking_sensivity");
  const YAML::Node *tracking_height_node = doc.FindValue("tracking_height");

  std::string drone_frameid = "";
  std::string camera_frameid = "";
  std::string marker_frameid = "";
  int tracking_sensivity = 0;
  int tracking_height = 0;

  *drone_frameid_node >> drone_frameid;
  *camera_frameid_node >> camera_frameid;
  *marker_frameid_node >> marker_frameid;
  *tracking_sensivity_node >> tracking_sensivity;
  *tracking_height_node >> tracking_height;

  int exist;

  matchParam(mrk.id, exist);

  tf::StampedTransform drone_to_camera_transform;
  tf::StampedTransform camera_to_marker_transform;
  tf::Transform marker_to_drone_transform ;

  if(exist){

  marker_frameid = marker_frameid + itoa(mrk.id);


    try
    {
        ROS_WARN("GET MARKER %d",mrk.id);
        m_tfListener->lookupTransform(drone_frameid, camera_frameid, ros::Time(), drone_to_camera_transform);
        m_tfListener->lookupTransform(camera_frameid, marker_frameid, ros::Time(), camera_to_marker_transform);
        //drone_to_camera_transform = tf::StampedTransform(marker_transform, ros::Time::now(), camera_frameid, marker_frameid);


        marker_to_drone_transform = drone_to_camera_transform*camera_to_marker_transform;
    }
    catch(tf::TransformException &e)
    {
          ROS_WARN("drone_frameid :%s",drone_frameid.c_str());
          ROS_WARN("camera_frameid :%s",camera_frameid.c_str());
          ROS_WARN("marker_frameid :%s",marker_frameid.c_str());

        ROS_ERROR("Failed to transform");
        return;
    }

    target_pose.header.stamp = ros::Time();
    target_pose.header.frame_id = drone_frameid;
    target_pose.pose.position.x = marker_to_drone_transform.getOrigin().x();
    target_pose.pose.position.y = marker_to_drone_transform.getOrigin().y();
    target_pose.pose.position.z = marker_to_drone_transform.getOrigin().z() + tracking_height;
    target_pose.pose.orientation.x = marker_to_drone_transform.getRotation().x();
    target_pose.pose.orientation.y = marker_to_drone_transform.getRotation().y();
    target_pose.pose.orientation.z = marker_to_drone_transform.getRotation().z();
    target_pose.pose.orientation.w = marker_to_drone_transform.getRotation().w();



    //TODO : vel pub
  double roll, pitch, yaw;
  tf::Quaternion q(target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    cmd_vel.linear.y = target_pose.pose.position.x * tracking_sensivity ;
    cmd_vel.linear.x = target_pose.pose.position.y * tracking_sensivity ;
    cmd_vel.linear.z = 0;//target_pose.pose.position.z * tracking_sensivity * -1;
    cmd_vel.angular.y = 0;//yaw * tracking_sensivity * -1;

    target_pose.pose.position.x = cmd_vel.linear.x;
    target_pose.pose.position.y = cmd_vel.linear.y;
    target_pose.pose.position.z = cmd_vel.linear.z;




    target_pose_pub.publish(target_pose);

    cmd_vel_pub.publish(cmd_vel);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_tracking");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  m_tfListener = new tf::TransformListener();

  n.getParam("/tracking_marker/tracking_config_file", yaml_file);

  int param_n = getParamCnt();

  target_pose_pub = n.advertise<geometry_msgs::PoseStamped>("tracking_target_pose", 1);
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);


  sub = n.subscribe("/visualization_marker", 1000, ReceiveCallback);


  while(ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
  return 0;
}
