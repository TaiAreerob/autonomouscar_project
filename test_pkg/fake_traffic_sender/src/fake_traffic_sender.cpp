#include <ros/ros.h>
#include <traffic_light_msgs/Light.h>
#include <std_msgs/Int32.h>
class Traffic_light{
public:
    Traffic_light(){
        //init ros member 
        light_pub = nh.advertise<traffic_light_msgs::Light>("traffic_light_erp42", 10);
        traffic_sign_sub = nh.subscribe("traffic_count",10,&Traffic_light::traffic_CB,this);
    }

    void traffic_CB(const std_msgs::Int32ConstPtr& ptr){
        //ros::Rate loop_rate(100);
		
		count_data = ptr->data;
        double begin = ros::Time::now().toSec();
		//ROS_INFO("BEGIN = %f", begin);
        if(count_data == 1){
            ROS_WARN ("Taffic Sign = YELLOW_3");
            while( ros::Time::now().toSec()- begin < 4){
                //tw : 무한루프는 컴퓨터에 로드가 매우 크게 걸리기 때문에 절대 하면 안됨
                //ROS_INFO("WTF");
		//		traffic_color.data = 2;
                //light_pub.publish(traffic_color);
				//loop_rate.sleep();
		light.light = traffic_light_msgs::Light::YELLOW;
		light_pub.publish(light);
            }
            ROS_WARN ("Taffic Sign = RED_3"); 
            while( ros::Time::now().toSec()- begin < 8){
                //traffic_color.data = 1;
                //light_pub.publish(traffic_color);
				//loop_rate.sleep();
		light.light = traffic_light_msgs::Light::RED;
                light_pub.publish(light);
            }
            ROS_WARN ("Taffic Sign = GREEN_3");
            while( ros::Time::now().toSec()- begin < 12){
                //traffic_color.data = 3;
                //light_pub.publish(traffic_color);
				//loop_rate.sleep();
		light.light = traffic_light_msgs::Light::STRAIGHT;
                light_pub.publish(light);
            }
        }
        else if(count_data == 2){
            ROS_WARN ("Taffic Sign = YELLOW_4");
            while( ros::Time::now().toSec()- begin < 4){
                //traffic_color.data = 5;
                //light_pub.publish(traffic_color);
				//loop_rate.sleep();
		light.light = traffic_light_msgs::Light::YELLOW;
                light_pub.publish(light);
            } 
            ROS_WARN ("Taffic Sign = RED_4");
            while( ros::Time::now().toSec()- begin < 8){
                //traffic_color.data = 4;
                //light_pub.publish(traffic_color);
				//loop_rate.sleep();
		light.light = traffic_light_msgs::Light::RED;
                light_pub.publish(light);
            }
            ROS_WARN ("Taffic Sign = GREEN_4");
            while( ros::Time::now().toSec()- begin < 12){
                //traffic_color.data = 6;
                //light_pub.publish(traffic_color);
				//loop_rate.sleep();
		light.light = traffic_light_msgs::Light::STRAIGHT;
                light_pub.publish(light);
            }
            ROS_WARN ("Taffic Sign = LEFT_4");
            while( ros::Time::now().toSec()- begin < 16){
                //traffic_color.data = 7;
                //light_pub.publish(traffic_color);
				//loop_rate.sleep();
		light.light = traffic_light_msgs::Light::LEFT;
                light_pub.publish(light);
            }
        }
        else
        {
            
        }
        
    }

private:
    //ros member 
    ros::NodeHandle nh;
    //pub
    ros::Publisher light_pub;
    ros::Subscriber traffic_sign_sub;
    //std_msgs::Int32 traffic_color;
    traffic_light_msgs::Light light;
    int count_data;
 
};

int main(int argc,char *argv[]){
    ros::init(argc, argv, "fake_traffic_sender");
    
    Traffic_light TL;
    ros::spin();
}
