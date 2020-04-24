#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <vector>

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
  int num_obstacles;
  std::vector<float> obs_x;
  std::vector<float> obs_y;
  std::vector<float> obs_z;

  nh.getParam("ob_start_x", obs_x);
  nh.getParam("ob_start_y", obs_y);
  nh.getParam("ob_start_z", obs_z);

  for(int i=0; i<obs_x.size(); ++i)
  {
    sphere.x = obs_x[i];
    sphere.y = obs_y[i];
    sphere.z = obs_z[i];
    ROS_INFO("Adding Sphere: x:%.1f, y:%.1f, z:%.1f", obs_x[i], obs_y[i],obs_z[i]);
    sphere_centers.push_back(sphere);
  }

  visual_tools_->loadMarkerPub();
  visual_tools_->waitForMarkerPub();
  visual_tools_->setLifetime(0);
  visual_tools_->publishSpheres(sphere_centers, rviz_visual_tools::RED, radius*2.0,
                                "obstacles");
  visual_tools_->trigger();
  ros::Duration(1.0).sleep();
  return 0;
}