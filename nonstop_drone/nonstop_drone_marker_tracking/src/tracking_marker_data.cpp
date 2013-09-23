#include "nonstop_drone_marker_tracking/tracking_marker_data.hpp"
#include <fstream>


void MarkerMarkData::MarkData::operator << (const YAML::Node& node)
{
  node["marker_id"]     >> marker_id;
}

void MarkerMarkData::configure(const YAML::Node& node) {

  list.clear();

  if ( node.size() == 0 ) {
      ROS_ERROR("yaml size is 0");
  }
  else{
      for (unsigned int i = 0; i < node.size(); i++)
        {
          // Parse every entries on YAML
          MarkData subscriber(i);
          subscriber << node[i];
          list.push_back(subscriber);
        }
  }
}
