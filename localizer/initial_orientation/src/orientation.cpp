#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector_map_msgs/PointArray.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <vector>
//#define MAX_DISTANCE 100

class InitialOrientation{
public:
    InitialOrientation()
    {
        isVectormapSubscribed = false;
        position_sub = nh.subscribe("/fix2tfpose/gnss_pose_raw", 1000, &InitialOrientation::gnssPoseCB, this);
        vector_map_point_sub = nh.subscribe("/vector_map_info/point", 1000, &InitialOrientation::vectorMapCB, this);
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
    }
    
    void gnssPoseCB(const geometry_msgs::PoseStamped::ConstPtr& ptr){
        if (false == isVectormapSubscribed) return;
        geometry_msgs::PoseStamped pose;
        pose.header = ptr->header;
        pose.header.stamp = ros::Time::now();
        x = ptr->pose.position.x;
        y = ptr->pose.position.y;
        double min = 1000000.0;
        double distance, vmap1_x, vmap1_y, vmap2_x, vmap2_y = 0.0;
        for(int i = 0; i<x_vmap.size(); ++i){
            double dx = x - x_vmap[i];
            double dy = y - y_vmap[i];
            distance = sqrt(dx*dx + dy*dy);
            if(distance < min){
                min = distance;
                vmap1_x = x_vmap[i];
                vmap1_y = y_vmap[i];
                vmap2_x = x_vmap[i+1];
                vmap2_y = y_vmap[i+1];
            }
        }
        yaw = atan2(vmap2_y - vmap1_y, vmap2_x - vmap1_x);
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.x = q[0];
        pose.pose.orientation.y = q[1];
        pose.pose.orientation.z = q[2];
        pose.pose.orientation.w = q[3];
        
        pose_pub.publish(pose);
    }

   void vectorMapCB(const vector_map_msgs::PointArray::ConstPtr& ptr){
         for(int i=0; i<ptr->data.size(); ++i){
             x_vmap.push_back(ptr->data[i].ly);
             y_vmap.push_back(ptr->data[i].bx);
         }
         isVectormapSubscribed = true;  
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Subscriber position_sub;;
    ros::Subscriber vector_map_point_sub;
    double x, y;
    double yaw;
    bool isVectormapSubscribed;
    std::vector<double> x_vmap, y_vmap;
};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "Initial_Pose");
    InitialOrientation ps;

    ros::spin();   
}