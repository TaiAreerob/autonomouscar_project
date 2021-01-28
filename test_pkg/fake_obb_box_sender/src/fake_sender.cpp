#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "fake_sender");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("transformed_obb_boxes", 10);
    ros::Rate r(10);
    jsk_recognition_msgs::BoundingBoxArray boxes;
    boxes.header.frame_id = "map";
    boxes.header.stamp = ros::Time::now();
    jsk_recognition_msgs::BoundingBox box;
    
    //parking kcity goal area 1
    box.header.frame_id = "map";
    box.header.stamp = ros::Time::now();
    box.pose.position.x = 207.629;
    box.pose.position.y = -186.590;
    box.pose.orientation.z = 0.7040;
    box.pose.orientation.w = 0.7101;
    box.dimensions.x = 4;
    box.dimensions.y = 2;
    box.dimensions.z = 2;
    boxes.boxes.push_back(box);

    box.header.frame_id = "map";
    box.header.stamp = ros::Time::now();
    box.pose.position.x = 205.039;
    box.pose.position.y = -183.952;
    box.pose.orientation.z = 0.6975;
    box.pose.orientation.w = 0.7164;
    box.dimensions.x = 4;
    box.dimensions.y = 2;
    box.dimensions.z = 2;
    boxes.boxes.push_back(box);

    box.header.frame_id = "map";
    box.header.stamp = ros::Time::now();
    box.pose.position.x = 202.662;
    box.pose.position.y = -182.739;
    box.pose.orientation.z = 0.6975;
    box.pose.orientation.w = 0.7164;
    box.dimensions.x = 4;
    box.dimensions.y = 2;
    box.dimensions.z = 2;
    boxes.boxes.push_back(box);

    box.header.frame_id = "map";
    box.header.stamp = ros::Time::now();
    box.pose.position.x = 200.197;
    box.pose.position.y = -181.352;
    box.pose.orientation.z = 0.6975;
    box.pose.orientation.w = 0.7164;
    box.dimensions.x = 4;
    box.dimensions.y = 2;
    box.dimensions.z = 2;
    boxes.boxes.push_back(box);


    box.header.frame_id = "map";
    box.header.stamp = ros::Time::now();
    box.pose.position.x = 197.664;
    box.pose.position.y = -180.036;
    box.pose.orientation.z = 0.6975;
    box.pose.orientation.w = 0.7164;
    box.dimensions.x = 4;
    box.dimensions.y = 2;
    box.dimensions.z = 2;
    boxes.boxes.push_back(box);

    //parking soongsil goal area 2
    // box.header.frame_id = "map";
    // box.header.stamp = ros::Time::now();
    // box.pose.position.x = 59.855;
    // box.pose.position.y = 68.087;
    // box.pose.orientation.z = 0.8302;
    // box.pose.orientation.w = 0.5573;
    // box.dimensions.x = 4;
    // box.dimensions.y = 2;
    // box.dimensions.z = 2;
    // boxes.boxes.push_back(box);
    while(ros::ok()){
        auto cur = ros::Time::now();
        boxes.header.stamp = cur;
        for(auto& box : boxes.boxes)
            box.header.stamp = cur;
        pub.publish(boxes);
        r.sleep();            

    }
}