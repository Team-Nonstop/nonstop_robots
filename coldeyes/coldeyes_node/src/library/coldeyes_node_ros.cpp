#include <coldeyes_node/coldeyes_node_ros.h>
#include <tf/transform_datatypes.h>

namespace coldeyes_node
{
  ColdeyesNodeRos::ColdeyesNodeRos(std::string& node_name)
  {
    this->name = node_name;
  }

  ColdeyesNodeRos::~ColdeyesNodeRos()
  {
  }

  bool ColdeyesNodeRos::init(ros::NodeHandle& nh)
  {
    coldeyes_node.init(nh);

    // initialize publishers
    advertiseTopics(nh);

    // initialize subscribers
    return true;
  }


  void ColdeyesNodeRos::advertiseTopics(ros::NodeHandle& nh) 
  {

    // odometry
    this->publisher["odom"] = nh.advertise<nav_msgs::Odometry>("odom",100);

  }


  void ColdeyesNodeRos::updateTF(geometry_msgs::TransformStamped& odom_tf)
  {
    odom_tf.header = this->coldeyes_node.odom.header;
    odom_tf.child_frame_id = this->coldeyes_node.odom.child_frame_id;
    odom_tf.transform.translation.x = this->coldeyes_node.odom.pose.pose.position.x;
    odom_tf.transform.translation.y = this->coldeyes_node.odom.pose.pose.position.y;
    odom_tf.transform.translation.z = this->coldeyes_node.odom.pose.pose.position.z;
    odom_tf.transform.rotation = this->coldeyes_node.odom.pose.pose.orientation;
  }


  bool ColdeyesNodeRos::update()
  {
    ros::Time time_now = ros::Time::now();
  
    // odom
    this->coldeyes_node.odom.header.stamp = time_now;
    this->publisher["odom"].publish(this->coldeyes_node.odom);
        this->coldeyes_node.odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->coldeyes_node.odom_pose[2]);
    // tf
    geometry_msgs::TransformStamped odom_tf;
    updateTF(odom_tf);
    this->tf_broadcaster.sendTransform(odom_tf);

    return true;
  }
}
