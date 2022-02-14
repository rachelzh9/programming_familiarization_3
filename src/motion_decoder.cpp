#include <ros/ros.h>
#include <motion_decoder/image_converter.hpp>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>


ImageConverter* ic;

void apriltag_detection_callback(const apriltags_ros::AprilTagDetectionArray msg)
{
  ROS_INFO("In subscribe\n");
  //TODO: Parse message and publish transforms as apriltag_tf and camera
  static tf::TransformBroadcaster br;
  geometry_msgs::PoseStamped tag_pose = msg.detections[0].pose;
  ic->setTagLocations(tag_pose.pose.position.x, tag_pose.pose.position.y, tag_pose.pose.position.z);
  tf::Stamped<tf::Transform> tag_transform;
  tf::poseStampedMsgToTF(tag_pose, tag_transform);
  br.sendTransform(tf::StampedTransform(tag_transform, ros::Time::now(), "camera", "april_tf"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  
  ros::NodeHandle n;
  //TODO: Add a subscriber to get the AprilTag detections The callback function skelton is given.
  ros::Subscriber sub = n.subscribe("tag_detections", 1000, apriltag_detection_callback);

  ImageConverter converter;
  ic = &converter;
  ros::Rate loop_rate(50);
  ROS_INFO("In main\n");
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
