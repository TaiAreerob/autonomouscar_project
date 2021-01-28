#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

class M{
public:
    M(){
        sub = nh.subscribe("cmd_vel", 10, &M::cmdCB, this);
        pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 10);
    }
    void cmdCB(const geometry_msgs::TwistConstPtr& ptr){
        geometry_msgs::TwistStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.twist = *ptr;
        pub.publish(msg);
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    M m;
    ros::spin();
}