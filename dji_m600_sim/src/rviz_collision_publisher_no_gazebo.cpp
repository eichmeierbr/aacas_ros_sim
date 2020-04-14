#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace {
constexpr double radius = 0.127
;
} // namespace
int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_collision_publisher");
  ros::NodeHandle nh;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  visual_tools_.reset(
      new rviz_visual_tools::RvizVisualTools("/world", "/obstacles"));

  std::vector<geometry_msgs::Point> sphere_centers;
  
  geometry_msgs::Point sphere;
  float obs_x;
  float obs_y;
  float obs_z;

  nh.getParam("ob_start_x", obs_x);
  nh.getParam("ob_start_y", obs_y);
  nh.getParam("ob_start_z", obs_z);

  sphere.x = obs_x;
  sphere.y = obs_y;
  sphere.z = obs_z;

//   geometry_msgs::Point sphere = geometry_msgs::Point(obs_x,obs_y,obs_z);


  sphere_centers.push_back(sphere);

  visual_tools_->loadMarkerPub();
  visual_tools_->waitForMarkerPub();
  visual_tools_->setLifetime(0);
  visual_tools_->publishSpheres(sphere_centers, rviz_visual_tools::RED, radius*2.0,
                                "obstacles");
  visual_tools_->trigger();
  ros::Duration(1.0).sleep();
  return 0;
}