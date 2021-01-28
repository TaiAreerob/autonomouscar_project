#include <ros/ros.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <vector>
//#include <autoware_msgs/DetectedObjectArray.h>

using namespace std;
//aaaaa
class Poly{

public:
	Poly() : seq(0), subseq(0){
		if(!nh.getParam("/park/park_start", poly_pose_start))
			throw runtime_error("set starting park pose");
		if(!nh.getParam("/park/park_end", poly_pose_end))
			throw runtime_error("set ending park pose");
		
		poly_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("poly_visualizer",1);

		//box_arr_msg.clear();
		//box_arr_msg.boxes.resize(0);

	}
	
	void pub(){
		polygon_arr.header.frame_id = "poly";
                polygon_arr.header.seq = seq++;
                polygon_arr.header.stamp = ros::Time::now();
		polygon_arr.labels.emplace_back(0);
                polygon_arr.likelihood.emplace_back(1.0);

		for(int i = 0; i < 8; i += 2){
			p.x = poly_pose_start[i];
			p.y = poly_pose_start[i+1];
			p.z = 0;
			
			poly.polygon.points.emplace_back(p);
			
			poly.header.frame_id = "poly";
			poly.header.seq = subseq++;
			poly.header.stamp = ros::Time::now();
		}
		polygon_arr.polygons.emplace_back(poly);

		poly.polygon.points.clear();
		poly.polygon.points.resize(0);

		poly_pub.publish(polygon_arr);
		polygon_arr.polygons.clear();
		polygon_arr.polygons.resize(0);
		polygon_arr.labels.clear();
		polygon_arr.labels.resize(0);
		polygon_arr.likelihood.clear();
		polygon_arr.likelihood.resize(0);

	}
private:
	ros::NodeHandle nh;
	ros::Publisher poly_pub;
	vector<double> poly_pose_start;
	vector<double> poly_pose_end;

	//autoware_msgs::DetectedObject& output;
	jsk_recognition_msgs::PolygonArray polygon_arr;
	//jsk_recognition_msgs::BoundingBoxArray box_arr_msg;
	geometry_msgs::PolygonStamped poly;
	geometry_msgs::Point32 p;
	unsigned int seq, subseq;
};
int main(int argc, char* argv[]){
	ros::init(argc, argv, "poly");
	Poly pl;

	//while(ros::ok()){
	//	ros::spinOnce();
		pl.pub();
	//}

	return 0;
}
