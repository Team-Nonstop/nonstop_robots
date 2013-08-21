#include <coldeyes_node/coldeyes_node.h>

namespace coldeyes_node
{
  void ColdeyesNode::init(ros::NodeHandle& nh)
  {
    // odometry
    nh.param("odom_frame",this->odom.header.frame_id,std::string("odom"));
    nh.param("base_frame",this->odom.child_frame_id,std::string("base_footprint"));

    this->odom_pose[0] = 0;
    this->odom_pose[1] = 0;
    this->odom_pose[2] = 0;

  }
}
