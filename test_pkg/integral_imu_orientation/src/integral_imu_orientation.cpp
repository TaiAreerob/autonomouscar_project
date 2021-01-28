// Software License Agreement (BSD License)
//
// Copyright (c) 2020, Taewook Park <sjrnfu12@naver.com>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the names of the authors nor the names of their
//    affiliated organizations may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

#define RAD2DEG (180.0 / M_PI)
static double wrapToPm(double a_num, const double a_max)
{
  if (a_num >= a_max)
    a_num -= 2.0 * a_max;
  else if (a_num <= -a_max)
    a_num += 2.0 * a_max;
  return a_num;
}

static double wrapToPmPi(const double a_angle_rad)
{
  return wrapToPm(a_angle_rad, M_PI);
}

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}

void imuCB_orientation(const sensor_msgs::ImuConstPtr& ptr){
    const ros::Time current_time = ptr->header.stamp;
    static ros::Time previous_time = current_time;
    const double diff_time = (current_time - previous_time).toSec();

    double imu_roll, imu_pitch, imu_yaw;
    tf::Quaternion imu_orientation;
    tf::quaternionMsgToTF(ptr->orientation, imu_orientation);
    tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

    imu_roll = wrapToPmPi(imu_roll);
    imu_pitch = wrapToPmPi(imu_pitch);
    imu_yaw = wrapToPmPi(imu_yaw);

    static double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
    const double diff_imu_roll = calcDiffForRadian(imu_roll, previous_imu_roll);
    const double diff_imu_pitch = calcDiffForRadian(imu_pitch, previous_imu_pitch);
    const double diff_imu_yaw = calcDiffForRadian(imu_yaw, previous_imu_yaw);

    static double r= 0, p= 0, y= 0;
    r+= diff_imu_roll; p += diff_imu_pitch; y += diff_imu_yaw;
    r = wrapToPmPi(r); p = wrapToPmPi(p); y = wrapToPmPi(y);
    ROS_INFO("diff yaw in degree : %lf - %lf, %lf",imu_yaw, previous_imu_yaw, diff_imu_yaw);

    previous_time = current_time;
    previous_imu_roll = imu_roll;
    previous_imu_pitch = imu_pitch;
    previous_imu_yaw = imu_yaw;
}

void imuCB_integral_angular_velocity(const sensor_msgs::ImuConstPtr& ptr){
    static int cnt = 500;
    
    const ros::Time current_time = ptr->header.stamp;
    static ros::Time previous_time = current_time;

    const double diff_time = (current_time - previous_time).toSec();
    if (fabs(diff_time) < 0.000001) return; //for the first time
    static double yaw = 0;
    double w = ptr->angular_velocity.z;
    yaw += diff_time * w;
    yaw = wrapToPmPi(yaw);
    ROS_INFO("accumulated yaw : %lf(DEG), %lf %lf", yaw * RAD2DEG, diff_time, w);
    previous_time = current_time;
    //if (--cnt == 0){ //periodic initialization
    //    cnt = 500;
    //    yaw = 0;
    //}
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "integral_imu_orientation");
    ros::NodeHandle nh;

    //ros::Subscriber sub = nh.subscribe("/imu", 10, &imuCB);
    ros::Subscriber sub = nh.subscribe("/imu", 10, &imuCB_integral_angular_velocity);
    ros::spin();
}