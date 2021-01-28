#include <ros/ros.h>
#include <traffic_light_msgs/Light.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "fake_traffic_light_sender_tw");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<traffic_light_msgs::Light>("traffic_light_erp42", 10);

    int hz = 10;
    ros::Rate r(hz); //1s
    const int COUNTER_INIT = 5 * hz; //10s
    int counter = COUNTER_INIT;
    int idx = 0;
    int sz = 3;
    traffic_light_msgs::Light msg;

    while(ros::ok()){
        switch(idx){
        case 0: ROS_WARN("Straight!"); msg.light = traffic_light_msgs::Light::RED; break;
        case 1: ROS_WARN("Left!"); msg.light = traffic_light_msgs::Light::RED; break;
        case 2: ROS_WARN("Red!"); msg.light = traffic_light_msgs::Light::RED; break;
        default : ROS_ERROR("impossible situation in fake traffic light sender tw!"); exit(18);
        }

        counter--;
        if (counter <= 0){//in every 10s,
            counter = COUNTER_INIT;
            idx = (idx + 1) % sz;
        }
        msg.header.stamp = ros::Time::now();
        pub.publish(msg);
        r.sleep();
    }
}