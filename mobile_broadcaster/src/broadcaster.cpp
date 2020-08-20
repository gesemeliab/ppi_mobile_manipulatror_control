#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

std::string rover_name;



void poseCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->vector.x, msg->vector.y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->vector.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", rover_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "mobile_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need rover name as argument"); return -1;};
  rover_name = argv[1];
  //rover_name = "rover";

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};
