// hero_radar.cpp
//#include "/home/yuyu/workspace/rm/rm_ws/src/radar_vision/include/hero_radar.h"
#include "hero_radar.h"

SubscriberPublisher::SubscriberPublisher(ros::NodeHandle& nh) {
    subscribe_topic_ = nh.param<std::string>("subscribe_topic", "/Odometry");
    publish_topic_ = nh.param<std::string>("publish_topic", "/output");

    sub_ = nh.subscribe(subscribe_topic_, 1000, &SubscriberPublisher::callback, this);
    pub_ = nh.advertise<std_msgs::Float64>(publish_topic_, 1000);
}

void SubscriberPublisher::callback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("Received odometry message");
    doLaser(*msg);
}

void SubscriberPublisher::doLaser(const nav_msgs::Odometry& msg) {
//    laserposition.laser_X = msg.pose.pose.position.x;
//    laserposition.laser_Y = msg.pose.pose.position.y;
//    laserposition.laser_Z = msg.pose.pose.position.z;
//
//    laserorientation.laser_x = msg.pose.pose.orientation.x;
//    laserorientation.laser_y = msg.pose.pose.orientation.y;
//    laserorientation.laser_z = msg.pose.pose.orientation.z;
//    laserorientation.laser_w = msg.pose.pose.orientation.w;
    tf::Quaternion laserorientation(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf::Vector3 laserposition(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    tf::Vector3 position(-0.0281, -0.1426, 0.27);
    tf::Quaternion orientation(0.382683, 0.0, 0.0, 0.92388);
    tf::Transform transform(orientation, position);

    tf::Vector3 baselinkposition = transform * laserposition;
    tf::Quaternion baselinkorientation = transform.getRotation() * laserorientation;

    Distance_target = sqrt((baselinkposition.x() - targetPos.x) * (baselinkposition.x() - targetPos.x) +
                           (baselinkposition.y() - targetPos.y) * (baselinkposition.y() - targetPos.y) +
                           (baselinkposition.z() - targetPos.z) * (baselinkposition.z() - targetPos.z));
//    Distance_target /= 1000;

    std_msgs::Float64 distance_msg;
    distance_msg.data = Distance_target;
    pub_.publish(distance_msg);
    baselinkposition1.baselink_X=baselinkposition.x();
    baselinkposition1.baselink_Y=baselinkposition.y();
    baselinkposition1.baselink_Z=baselinkposition.z();

    publishTF(msg);
}

void SubscriberPublisher::publishTF(const nav_msgs::Odometry& msg) {
    geometry_msgs::TransformStamped laser_transform;
    laser_transform.header.stamp = ros::Time::now();
    laser_transform.header.frame_id = "map";
    laser_transform.child_frame_id = "baselink";
    laser_transform.transform.translation.x = baselinkposition1.baselink_X ;
    laser_transform.transform.translation.y = baselinkposition1.baselink_Y ;
    laser_transform.transform.translation.z = baselinkposition1.baselink_Z ;
    laser_transform.transform.rotation.w = 1.0;

    geometry_msgs::TransformStamped target_transform;
    target_transform.header.stamp = ros::Time::now();
    target_transform.header.frame_id = "map";
    target_transform.child_frame_id = "target";
    target_transform.transform.translation.x = targetPos.x;
    target_transform.transform.translation.y = targetPos.y;
    target_transform.transform.translation.z = targetPos.z;
    target_transform.transform.rotation.w = 1.0;

    br_.sendTransform(laser_transform);
    br_.sendTransform(target_transform);
}