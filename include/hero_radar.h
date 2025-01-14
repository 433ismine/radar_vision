// hero_radar.h
#ifndef HERO_RADAR_H
#define HERO_RADAR_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"
#include <rm_msgs/TrackData.h>

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
    ros::Publisher pub2_;
    ros::Publisher pub3_;
    std::string subscribe_topic_;
    std::string publish_topic_;
    std::string position_topic_;
    std::string odom_topic_;
    std::uint8_t targetid=8;
    std::uint8_t heroid=1;

    tf2_ros::TransformBroadcaster br_;


    struct Hero2targetposition {
        float hero_X;
        float hero_Y;
        float hero_Z;
    }hero2targetposition;

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
        float x ;
        float y ;
        float z ;
    } targetPos;
};

#endif  // HERO_RADAR_H