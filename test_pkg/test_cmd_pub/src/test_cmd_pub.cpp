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
//#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <cmath>
   
#define FRE 10

int main(int argc, char *argv[]){
      ros::init(argc, argv, "test_cmd_pub");
      ros::NodeHandle nh;
      ros::Publisher ackermann_publisher = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd_erp42",100);
	  ros::Publisher degree_publisher = nh.advertise<std_msgs::Float64>("degree_cmd",100);
	  ros::Rate loop_rate(FRE);
  
      geometry_msgs::TwistStamped msg;
      std_msgs::Float64 msg_degree;

      bool variable_velocity = false; //true;
      double T = 8;
      int PLUS = 2;
      static int time = 0;
      double target_v = 0;
	  int floor_T = 100;
	  int target_vel_1 = 1;
	  int target_vel_2 = 3;

      while(ros::ok()){
   	  double linear_v; 
	  double angular_w;

 	  if (!nh.getParam("/test_cmd_pub/linear_v", linear_v))
		throw std::runtime_error("/test_cmd_pub/set linear_v!");
      if (!nh.getParam("/test_cmd_pub/angular_w", angular_w))
        throw std::runtime_error("set angular_w");
	  if (!nh.getParam("/test_cmd_pub/T", T))
        throw std::runtime_error("set T");
      if (!nh.getParam("/test_cmd_pub/PLUS", PLUS))
        throw std::runtime_error("set PLUS");
      if (!nh.getParam("/test_cmd_pub/variable_velocity", variable_velocity))
        throw std::runtime_error("set variable_velocity");
      if (!nh.getParam("/test_cmd_pub/floor_T", floor_T))
        throw std::runtime_error("set floor_T");
      if (!nh.getParam("/test_cmd_pub/target_vel_1", target_vel_1))
        throw std::runtime_error("set target_vel_1");
      if (!nh.getParam("/test_cmd_pub/target_vel_2", target_vel_2))
        throw std::runtime_error("set target_vel_2");
           
	  msg.header.stamp = ros::Time::now();
	  msg.twist.linear.x = linear_v;
	  // variable_velocity = true;
	  // if (variable_velocity){
		// double t = ros::Time::now().toSec();
		// double target_v = linear_v * std::sin(t * 2 * M_PI / T) + PLUS;
		// //double target_v;
		// //ROS_INFO("IF_TIME : %d",time);
		// if((time/floor_T) %2 == 0){
		//   target_v = target_vel_1;
		// }
		// else{
		//   target_v = target_vel_2;
	  // 	}		 
		// msg.twist.linear.x = target_v;
	  // }
    //   msg.twist.angular.z = angular_w; 
  	//   msg_degree.data = (-1.0) *atan(1.12 * (angular_w) / (linear_v)) * (180.0 / 3.14159265358979);

	  ROS_INFO("linear_v : %lf",linear_v);
	  //ROS_INFO("linear_v : %lf",target_v);
	  ROS_INFO("angular_w : %lf",angular_w);
	  ROS_INFO("steer : %lf DEGREE", atan(1.12 * (angular_w) / (linear_v)) * (180.0 / 3.14159265358979)); 
      ackermann_publisher.publish(msg);
	  degree_publisher.publish(msg_degree);
      time++;
	  //ROS_INFO("TIME : %d",time);
	  loop_rate.sleep();
    }
 }
         

