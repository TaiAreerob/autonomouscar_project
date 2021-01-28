#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class StoplineDistDetector{
public:
    StoplineDistDetector(){
        if (!nh.getParam("stopline_dist_detector/a_vec", a_vec)) throw std::runtime_error("set a_vec!");
        if (!nh.getParam("stopline_dist_detector/b_vec", b_vec)) throw std::runtime_error("set b_vec!");
        if (!nh.getParam("stopline_dist_detector/c_vec", c_vec)) throw std::runtime_error("set c_vec!");

        if ((a_vec.size() != b_vec.size()) || (b_vec.size() != c_vec.size())) throw std::runtime_error("line definition error. check a, b, c vec's length");
        n_line = a_vec.size(); 

        cur_pose_sub = nh.subscribe("current_pose", 100, &StoplineDistDetector::currentPoseCB, this);
    }

    void currentPoseCB(const geometry_msgs::PoseStampedConstPtr& ptr){
        cur_pose = ptr->pose;

        for(size_t i = 0 ; i < n_line; ++i){
            double a = a_vec[i], b = b_vec[i], c = c_vec[i];
            double x = cur_pose.position.x, y = cur_pose.position.y;
            double dist = std::fabs(a*x + b*y + c) / std::sqrt(a*a + b*b);

            ROS_INFO("[%lu] dist : %lf ", i + 1, dist);
        }
    }


private:
    size_t n_line;
    ros::NodeHandle nh;
    ros::Subscriber cur_pose_sub;
    std::vector<double> a_vec;
    std::vector<double> b_vec;
    std::vector<double> c_vec;
    
    geometry_msgs::Pose cur_pose;
};
int main(int argc, char *argv[]){
    ros::init(argc, argv, "stopline_dist_detector");

    StoplineDistDetector s;
    ros::spin();
    
}
