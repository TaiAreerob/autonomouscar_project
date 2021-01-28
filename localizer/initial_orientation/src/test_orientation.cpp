#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
        vector_map_point_sub = nh.subscribe("/vector_map_info/point", 1000, &InitialOrientation::vectorMapCB, this);
        position_sub = nh.subscribe("initialpose", 1000, &InitialOrientation::initialPoseCB, this);
        pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initial", 1000);
    }

     void vectorMapCB(const vector_map_msgs::PointArray::ConstPtr& ptr){
         for(int i=0; i<ptr->data.size(); ++i){
             xx.push_back(ptr->data[i].ly);
             yy.push_back(ptr->data[i].bx);
         }  
    }
    
    void initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ptr){
        pose.header = ptr->header;
        x = ptr->pose.pose.position.x;
        y = ptr->pose.pose.position.y;
        double min = 100.0;
        double distance, vmap1_x, vmap1_y, vmap2_x, vmap2_y = 0.0;
        int size = xx.size();
        for(int i = 0; i<size; ++i){
            double dx = x - xx[i];
            double dy = y - yy[i];
            distance = sqrt(dx*dx + dy*dy);
            if(distance < min){
                if(i+1 > size-1){
                    vmap1_x = xx[i];
                    vmap1_y = yy[i];
                    vmap2_x = xx[0];
                    vmap2_y = yy[0];
                }
                else{
                    vmap1_x = xx[i];
                    vmap1_y = yy[i];
                    vmap2_x = xx[i+1];
                    vmap2_y = yy[i+1];
                }
                min = distance;          
            }
        }
        yaw = atan2(vmap2_y - vmap1_y, vmap2_x - vmap1_x);
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);

        pose.header.frame_id = "map";
        pose.pose.pose.position.x = x;
        pose.pose.pose.position.y = y;
        pose.pose.pose.position.z = 0;
        pose.pose.pose.orientation.x = q[0];
        pose.pose.pose.orientation.y = q[1];
        pose.pose.pose.orientation.z = q[2];
        pose.pose.pose.orientation.w = q[3];
        pose.pose.covariance.assign(0);

        pose_pub.publish(pose);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub;
    ros::Subscriber position_sub;;
    ros::Subscriber vector_map_point_sub;
    double x, y;
    double yaw;

    geometry_msgs::PoseWithCovarianceStamped pose;
    std::vector<double> xx, yy;
    double x_vmap, y_vmap;
};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "Initial_Pose");
    InitialOrientation ps;

    ros::spin();
    
}

