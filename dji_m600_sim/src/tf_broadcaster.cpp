#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


void poseCallback(const geometry_msgs::PointStamped& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  auto x = msg.point.x;
  auto y = msg.point.y;
  auto z = msg.point.z;
  // ROS_ERROR("need turtle name as argument")
  // std::cout<< x<<y<<z;
  transform.setOrigin( tf::Vector3(x,y,z) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/local_position", 10, &poseCallback);

  ros::spin();
  return 0;
};