#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <GeographicLib/Geocentric.hpp>

using namespace GeographicLib;

constexpr double GPS_ORIENTATION_STEP_DIST = 0.2;

class Fix2PointmapPose{
public:
    Fix2PointmapPose() : earth(Constants::WGS84_a(), Constants::WGS84_f()), nh("~")
    {
        if (!nh.getParam("a1", tf_mat(0, 0)))           throw std::runtime_error("set tf_matrix parameter!");
        if (!nh.getParam("b1", tf_mat(0, 1)))           throw std::runtime_error("set tf_matrix parameter!");
        if (!nh.getParam("c1", tf_mat(0, 2)))           throw std::runtime_error("set tf_matrix parameter!");
        if (!nh.getParam("d1", tf_mat(0, 3)))           throw std::runtime_error("set tf_matrix parameter!");
        if (!nh.getParam("a2", tf_mat(1, 0)))           throw std::runtime_error("set tf_matrix parameter!");
        if (!nh.getParam("b2", tf_mat(1, 1)))           throw std::runtime_error("set tf_matrix parameter!");
        if (!nh.getParam("c2", tf_mat(1, 2)))           throw std::runtime_error("set tf_matrix parameter!");
        if (!nh.getParam("d2", tf_mat(1, 3)))           throw std::runtime_error("set tf_matrix parameter!");
        if (!nh.getParam("a3", tf_mat(2, 0)))           throw std::runtime_error("set tf_matrix parameter!");
        if (!nh.getParam("b3", tf_mat(2, 1)))           throw std::runtime_error("set tf_matrix parameter!");
        if (!nh.getParam("c3", tf_mat(2, 2)))           throw std::runtime_error("set tf_matrix parameter!");
        if (!nh.getParam("d3", tf_mat(2, 3)))           throw std::runtime_error("set tf_matrix parameter!");
        
        std::cout << "tf_matrix \n";
        std::cout << tf_mat << std::endl;

        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose_raw", 1000);
        stat_pub = nh.advertise<std_msgs::Bool>("gnss_state", 1000);

        fix_sub = nh.subscribe("/ublox_gps/fix", 1000, &Fix2PointmapPose::fixCB, this);
    }

    void fixCB(const sensor_msgs::NavSatFixConstPtr& ptr){
    /*
        1. (lat, lon, alt) -> (x, y, z) based on earth
        2. (x, y, z) based on Earth -> (x, y, z) Pointmap
        3. publish tf, poseStamped
    */
    
    //step 1
        double x_earth, y_earth, z_earth, x_point, y_point, z_point;
        earth.Forward(ptr->latitude, ptr->longitude, ptr->altitude,
                x_earth, y_earth, z_earth);

    //step 2
        Eigen::Vector4d pose_earth(x_earth, y_earth, z_earth, 1);
        Eigen::Vector3d pose_point;
        pose_point = tf_mat * pose_earth; 

    //step 3 - 1. publish pose stamped
        geometry_msgs::PoseStamped pose;
        pose.header = ptr->header;

        pose.header.frame_id = "map";
        //pose.header.stamp = ros::Time::now();
        pose.pose.position.x = pose_point(0);
        pose.pose.position.y = pose_point(1);
        pose.pose.position.z = pose_point(2); 

        //set gnss state
        std_msgs::Bool gnss_stat_msg;
        if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
        {
            gnss_stat_msg.data = false;
        }
        else
        {
            gnss_stat_msg.data = true;
        }

        //calc orientation
        double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
                               pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));
        std::cout << "distance : " << distance << std::endl;
    
        if (distance > GPS_ORIENTATION_STEP_DIST)
        {
            yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
            _quat = tf::createQuaternionMsgFromYaw(yaw);
            _prev_pose = pose;
        }
        pose.pose.orientation = _quat;
        pose_pub.publish(pose);
        stat_pub.publish(gnss_stat_msg);

        //step3 - 2. Publish tf
        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
        q.setRPY(0, 0, yaw);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ptr->header.stamp, "map", "gps"));
    }
private:
    Geocentric earth;
    Eigen::Matrix<double, 3, 4> tf_mat;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    geometry_msgs::PoseStamped _prev_pose;
    double yaw;
    geometry_msgs::Quaternion _quat;

    ros::Publisher stat_pub, pose_pub;
    ros::Subscriber fix_sub;
};


int main(int argc, char *argv[]){
    ros::init(argc, argv, "fix2tfpose");

    Fix2PointmapPose f;
    ros::spin();

    ros::shutdown();
    return 0;
}