#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <string>

geometry_msgs::Quaternion geom_quat;

void attitudeCallback(const geometry_msgs::QuaternionStamped& msg){
  geom_quat = msg.quaternion;

}


void poseCallback(const geometry_msgs::PointStamped& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  auto x = msg.point.x;
  auto y = msg.point.y;
  auto z = msg.point.z;
  transform.setOrigin( tf::Vector3(x,y,z) );

  tf::Quaternion q(geom_quat.x, geom_quat.y, geom_quat.z, geom_quat.w);
  q.normalize();
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");

  ros::NodeHandle node;

  std::string position_topic, attitude_topic;

  node.getParam("position_pub_name", position_topic);
  node.getParam("attitude_pub_name", attitude_topic);

  ros::Subscriber pos_sub = node.subscribe(position_topic, 10, &poseCallback);
  ros::Subscriber att_sub = node.subscribe(attitude_topic, 10, &attitudeCallback);

  ros::spin();
  return 0;
};