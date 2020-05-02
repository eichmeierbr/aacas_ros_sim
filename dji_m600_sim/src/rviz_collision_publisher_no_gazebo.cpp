#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "aacas_detection/ObstacleDetectionArray.h"
#include "aacas_detection/ObstacleDetection.h"
#include <vector>
#include <string>

namespace {
constexpr double radius = 0.127;
} // namespace
rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

// Update Visual Object Location callback
void obstacle_visualization_callback(const aacas_detection::ObstacleDetectionArray msg)
{
  
  visual_tools_.reset(
      new rviz_visual_tools::RvizVisualTools("/world", "/obstacles"));

  
  std::vector<geometry_msgs::Point> sphere_centers;
  auto detections = msg.detections;

  geometry_msgs::Point sphere;

  for(int i=0; i<detections.size(); ++i)
  {
    sphere.x = detections[i].position.x;
    sphere.y = detections[i].position.y;
    sphere.z = detections[i].position.z;
    sphere_centers.push_back(sphere);
  }

  visual_tools_->loadMarkerPub();
  visual_tools_->waitForMarkerPub();
  visual_tools_->setLifetime(0);
  visual_tools_->publishSpheres(sphere_centers, rviz_visual_tools::RED, radius*2.0,
                                "obstacles");
  visual_tools_->trigger();
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_collision_publisher");
  ros::NodeHandle nh;

  std::string obstacle_location_topic;
  obstacle_location_topic = nh.getParam("true_obstacle_topic", obstacle_location_topic);

  ros::Subscriber april_detection_sub = nh.subscribe("obstacle_information", 1000, obstacle_visualization_callback);

  ros::spin();
  return 0;
}