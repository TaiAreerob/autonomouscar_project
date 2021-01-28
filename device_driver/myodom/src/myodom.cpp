#include <ros/ros.h>
//#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
//#include <mutex>
//#include <string>
//#include <endian.h>
//#include <erp42_msgs/erp42_to_pc.h>
//#include <mysen_imu_driver/driver_core.h>
//#include <signal.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>

struct pose
{
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
};


static pose current_pose_imu_odom;

static double encoder_v=0;
static double diff_distance=0;
static double diff_imu_roll=0, diff_imu_pitch=0, diff_imu_yaw=0;
//static std::vector<double> imu_w;
static geometry_msgs::Vector3 imu_w;


class MyOdom{
public:
	MyOdom(){
		encoder = nh1.subscribe("erp42_odometry",100,&MyOdom::erp42_Callback,this);
		imu = nh2.subscribe("imu",1000,&MyOdom::imu_Callback,this);
		myodom_pub = nh3.advertise<nav_msgs::Odometry>("my_odometry", 100);
		last_encoder = ros::Time::now();
		last_imu = ros::Time::now();
	}
	
	void erp42_Callback(const nav_msgs::Odometry::ConstPtr& msg){
		cur_encoder = msg->header.stamp;
		
		dt_encoder = (cur_encoder-last_encoder).toSec();		
		encoder_v = msg->twist.twist.linear.x;			
		diff_distance = encoder_v * dt_encoder; 
        
		ROS_INFO("diff_distance = %lf",diff_distance);

		last_encoder = cur_encoder;
		pub_myodom();
	}
	
	void imu_Callback(const sensor_msgs::Imu::ConstPtr& msg){
		ros::Time cur_imu = msg->header.stamp;
		
		dt_imu = (cur_imu - last_imu).toSec();
		imu_w = msg->angular_velocity;
		
		diff_imu_roll = msg->angular_velocity.x * dt_imu;
        	diff_imu_pitch = msg->angular_velocity.y * dt_imu;
        	diff_imu_yaw = msg->angular_velocity.z * dt_imu;
  
        	current_pose_imu_odom.roll += diff_imu_roll;
        	current_pose_imu_odom.pitch += diff_imu_pitch;
        	current_pose_imu_odom.yaw += diff_imu_yaw;
		
		ROS_INFO("roll =%lf, pitch = %lf, yaw=%lf",current_pose_imu_odom.roll,current_pose_imu_odom.pitch,current_pose_imu_odom.yaw);

		last_imu = cur_imu;
	}

	void pub_myodom(){
		
		current_pose_imu_odom.x += diff_distance * cos(-current_pose_imu_odom.pitch) * cos(current_pose_imu_odom.yaw);
   		current_pose_imu_odom.y += diff_distance * cos(-current_pose_imu_odom.pitch) * sin(current_pose_imu_odom.yaw);
    		current_pose_imu_odom.z += diff_distance * sin(-current_pose_imu_odom.pitch);
  
		geometry_msgs::Quaternion myodom_quat = tf::createQuaternionMsgFromRollPitchYaw(current_pose_imu_odom.roll,current_pose_imu_odom.pitch,current_pose_imu_odom.yaw);
		
		myodom_msg.header.seq++;
        	myodom_msg.header.stamp = cur_encoder;
        	myodom_msg.header.frame_id = "odom";
        	myodom_msg.child_frame_id = "my_base_link";
        	myodom_msg.pose.pose.position.x = current_pose_imu_odom.x;
        	myodom_msg.pose.pose.position.y = current_pose_imu_odom.y;
		myodom_msg.pose.pose.position.z = current_pose_imu_odom.z;
        	myodom_msg.pose.pose.orientation = myodom_quat;
        	myodom_msg.twist.twist.linear.x = encoder_v;
        	myodom_msg.twist.twist.angular = imu_w;
        	myodom_pub.publish(myodom_msg);

		geometry_msgs::TransformStamped myodom_trans;
        	myodom_trans.header.stamp = cur_encoder;
        	myodom_trans.header.frame_id = "odom";
        	myodom_trans.child_frame_id = "my_base_link";
        	myodom_trans.transform.translation.x = current_pose_imu_odom.x;
        	myodom_trans.transform.translation.y = current_pose_imu_odom.y;
        	myodom_trans.transform.translation.z = current_pose_imu_odom.z;
        	myodom_trans.transform.rotation = myodom_quat;
        	myodom_broadcaster_.sendTransform(myodom_trans);

	}

private:
	ros::NodeHandle nh1;
	ros::NodeHandle nh2;
    	ros::NodeHandle nh3;
	ros::Publisher myodom_pub;
	ros::Subscriber encoder;
	ros::Subscriber imu;
	ros::Time cur_encoder;
	ros::Time last_encoder;
	ros::Time last_imu;
	double dt_encoder, dt_imu;
	nav_msgs::Odometry myodom_msg;
	tf::TransformBroadcaster myodom_broadcaster_;
	/*
	static pose current_pose_imu_odom,predict_pose_imu_odom,previous_pose;
    static double offset_odom_x, offset_odom_y, offset_odom_z,
                       offset_odom_roll, offset_odom_pitch, offset_odom_yaw ;
 
    static constexpr double encoder_v=0;
    static constexpr double diff_distance=0;
    static constexpr double diff_imu_roll=0, diff_imu_pitch=0, diff_imu_yaw=0;
    //static std::vector<double> imu_w;
    static constexpr geometry_msgs::Vector3 imu_w;
	*/
};


int main(int argc, char *argv[]){
	ros::init(argc,argv,"myodom");

	MyOdom MO;
	ros::spin();

	return 0;
}
