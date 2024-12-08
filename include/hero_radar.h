// hero_radar.h
#ifndef HERO_RADAR_H
#define HERO_RADAR_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>


class SubscriberPublisher
{
public:
    SubscriberPublisher(ros::NodeHandle& nh);

private:
    void callback(const nav_msgs::Odometry::ConstPtr& msg);
    void doLaser(const nav_msgs::Odometry& msg);
    void publishTF(const nav_msgs::Odometry& msg);

    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::string subscribe_topic_;
    std::string publish_topic_;
    tf2_ros::TransformBroadcaster br_;


    struct Laserposition {
        float laser_X;
        float laser_Y;
        float laser_Z;
    }laserposition;

    struct Laserorientation{
        float laser_x;
        float laser_y;
        float laser_z;
        float laser_w;
    }laserorientation;

    struct Baselinkposition{
        float baselink_X;
        float baselink_Y;
        float baselink_Z;
    }baselinkposition1;

    struct Baselinkorientation {
        float baselink_x;
        float baselink_y;
        float baselink_z;
        float baselink_w;
    }baselinkorientation;

//    float laser_X, laser_Y, laser_Z;
//    float laser_x, laser_y, laser_z,laser_w;
//    float baselink_X, baselink_Y, baselink_Z;
//    float baselink_x, baselink_y, baselink_z,baselink_w;

    float Distance_target;
    struct TargetPos {
        float x = 10.0;
        float y = 10.0;
        float z = 10.0;
    } targetPos;
};

#endif  // HERO_RADAR_H