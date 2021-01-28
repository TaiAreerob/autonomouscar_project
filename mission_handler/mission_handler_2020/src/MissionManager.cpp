// Software License Agreement (BSD License)
//
// Copyright (c) 2020, Taewook Park <sjrnfu12@naver.com>, jaehun Kim, Sanguk Yu, Hongchal Jo
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
#include <mission_handler_2020/Mission.h>
#include <mission_handler_2020/IsPointInPolygon.h>
#include <vector>
#include <mutex>
#include <thread>

#include <jsk_recognition_msgs/PolygonArray.h> //내가 추가
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_msgs/DetectedObjectArray.h> //add by hc 
#include <std_msgs/Int32.h>

struct MissionInfo{
    int mission_identifier;
    geometry_msgs::Polygon area;
    Mission* mission_ptr;
};

class MissionManager{
public:
    void genPolyMsg(geometry_msgs::Polygon& poly_ret, std::vector<double> ppoly, const char* name){
        static int label;
        static int ll;

        polyArr.header.frame_id = "map";
        polyArr.header.seq = 1;
        polyArr.header.stamp = ros::Time::now(); //일단...

        geometry_msgs::PolygonStamped poly;

        poly.header.frame_id = "map";
		poly.header.seq = 1;
		poly.header.stamp = ros::Time::now();

        int size = ppoly.size();
        int x,y;
        for(int i = 0; i < size; i += 2){
            geometry_msgs::Point32 p;
            p.x = ppoly[i];
			p.y = ppoly[i+1];
			p.z = 0;
            x = p.x;
            y = p.y;
            poly.polygon.points.emplace_back(p);
        }

        polyArr.polygons.emplace_back(poly);
        polyArr.labels.emplace_back(label++);
        polyArr.likelihood.emplace_back(ll);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = name;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.z=1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
//      marker.scale.x = 10;
//      marker.scale.y = 10;
        marker.scale.z = 5;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.pose.position.x = x;
        marker.pose.position.y = y;

        marker.text = name;
        markera.markers.emplace_back(marker);

        //return value
        poly_ret = poly.polygon;
    }

    void initMissions(){
        //must be aligned with .yaml file
        auto createMissionHandler = [&](std::string mission_name, int mission_type)-> Mission* {
            if (mission_type == 0) //parking
                return new MissionParking(nh, mission_name);
            else if (mission_type == 1) // obstacle_avoidance
                return new MissionObstacleAvoidance(nh, mission_name);
            else if (mission_type == 2) // traffic light
                return new MissionTrafficLight(nh, mission_name);
            else if (mission_type == 3) //emergency obstacle
                return new MissionEmergencyObstacle(nh, mission_name);
            else
                throw std::runtime_error(std::string() + "unknown mission type : " + std::to_string(mission_type));
        };

        std::vector<std::string> mission_names;
        std::vector<int> mission_types;

        if (!nh.getParam("mission_names", mission_names)) throw std::runtime_error("set mission_names!");
        if (!nh.getParam("mission_types", mission_types)) throw std::runtime_error("set mission_types!");

        size_t n_mission = mission_names.size();

        //init MissionNone
        MissionInfo info;
        info.mission_identifier = 0;
        info.mission_ptr = new MissionNone(nh, "MissionNone");
        mission_infos.push_back(info);

        //init Missions
        std::vector<double> poly_vec;
        geometry_msgs::Polygon poly;
        for(size_t i = 0 ; i < n_mission; ++i){
            std::string mission_name = mission_names[i];
            int mission_type = mission_types[i];
    		if(!nh.getParam(mission_name + "/poly", poly_vec)) throw std::runtime_error( std::string() + "set " + mission_name + "/poly!!!");
            genPolyMsg(poly, poly_vec, mission_name.c_str());

            info.mission_identifier = mission_infos.back().mission_identifier + 1;
            info.mission_ptr = createMissionHandler(mission_name, mission_type);
            info.area = poly;

            mission_infos.push_back(info);
        }
        
        //init mission handling variables
        mission_identifier_cur = 0;
        mission_none = &mission_infos[0]; 
        cur_mission = &mission_infos[mission_identifier_cur];
    }

    MissionManager(){
        //init mission instances
        initMissions();

        //init ros members
        //pub
        poly_pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("mission_poly_visualizer",10);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("mission_text_visualizer",10);
        objects_pub = nh.advertise<autoware_msgs::DetectedObjectArray>("tracked_objects",10);
        erp42_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd_erp42", 10);////////////debug
        tracked_obj_pub = nh.advertise<visualization_msgs::MarkerArray>("fake_tracked_obj_viz", 10);

        //sub
        curposeSub = nh.subscribe("current_pose", 10, &MissionManager::curposeCB, this);
        curTwistSub = nh.subscribe("current_velocity", 10, &MissionManager::curTwistCB, this);
        objects_sub = nh.subscribe("detection/tracked_objects", 10, &MissionManager::objects_infoCB,this);//need to change autoware code
        traffic_light_sub = nh.subscribe("traffic_light_erp42", 10, &MissionManager::traffic_lightCB, this);
        twist_cmd_sub = nh.subscribe("twist_cmd", 10, &MissionManager::twist_cmdCB, this);
        cmd_vel_sub = nh.subscribe("cmd_vel", 10, &MissionManager::cmd_velCB, this);
        twist_purepursuit_sub = nh.subscribe("twist_raw_pure_pursuit", 10, &MissionManager::twist_cmd_purepursuitCB, this);
        objects_center_points_sub = nh.subscribe("transformed_obb_boxes", 10, &MissionManager::obstacle_center_points_CB_quadtree, this);
        //init thread
        std::thread(&MissionManager::missionProcesingThread, this).detach();
        std::thread(&MissionManager::missionAreaDebugThread, this).detach();
    }

    void missionAreaDebugThread(){
        ros::Rate r(10);
        while(ros::ok()){
            v2m_lock.lock();//I'm lazy to make new lock for each thread..
            poly_pub.publish(polyArr);
            marker_pub.publish(markera);
            cur_mission->mission_ptr->visualize();
            v2m_lock.unlock();
            r.sleep();
        }
    }

    void missionProcesingThread(){
        ros::Rate loop_rate(50);

        /* reset mission processing variables */
        //not use mission handler
        bool not_use_mission_handler;
        if (!nh.getParam("not_use_mission_handler", not_use_mission_handler)) throw std::runtime_error("set not_use_mission_handler!");
        
        //mission_activate_ary
        std::vector<bool> mission_activate_ary_wo_none;
        if (!nh.getParam("mission_activate_ary", mission_activate_ary_wo_none)) throw std::runtime_error("set mission_activate_ary!");
        std::vector<bool> mission_activate_ary;
        mission_activate_ary.push_back(true); //mission_none -> .yaml file does not include flag for missionNone
        mission_activate_ary.insert(mission_activate_ary.end(), mission_activate_ary_wo_none.begin(), mission_activate_ary_wo_none.end());

        //isMissionDoneVec
        std::vector<bool> isMissionDoneVec;
        isMissionDoneVec.resize(mission_infos.size());
        std::fill_n(isMissionDoneVec.begin(), isMissionDoneVec.size(), false);

        while(ros::ok()){
            if (not_use_mission_handler){
                loop_rate.sleep();
                continue;
            }
            
            v2m_lock.lock();
            m2v_lock.lock();

            //process mission
            if (mission_activate_ary[mission_identifier_cur]) 
                m2v = cur_mission->mission_ptr->processMission(v2m);

            //find current mission
            //assume there's no intersaction between mission area
            int mission_identifier_localizing = 0;
            for(size_t i = 0 ; i < mission_infos.size(); ++i){
                if (true == isPointInPolygon(v2m.curpos.pose.position.x, 
                    v2m.curpos.pose.position.y, 
                    mission_infos[i].area))
                {
                    if(isMissionDoneVec[i] == false)
                        mission_identifier_localizing = (int)i;
                    break;
                }
            }
            
            //find next mission
            int mission_idx_next = 0;
            bool changed = false;
            if (mission_identifier_localizing != mission_identifier_cur) {
                changed = true;
                mission_idx_next = mission_identifier_localizing; //default is mission none
            }
            if ((cur_mission != mission_none) && m2v.isMissionDone) {
                mission_idx_next = 0; //mission none
                changed = true;
            }

            //clean current mission and change to next
            if (changed){
                if (mission_idx_next == 0) isMissionDoneVec[mission_identifier_cur] = true;

                cur_mission->mission_ptr->clean();                
                mission_identifier_cur = mission_idx_next;
                cur_mission = &mission_infos[mission_identifier_cur];
                m2v.clear();
                ROS_WARN("switch to mission[%d] : %s", mission_identifier_cur, cur_mission->mission_ptr->to_string().c_str());
            }
            
            m2v_lock.unlock();
            v2m_lock.unlock(); //no need to lock v2m anymore

            loop_rate.sleep();
        }
    }
    void curTwistCB(const geometry_msgs::TwistStampedConstPtr& ptr){
        v2m_lock.lock();
        v2m.linear_velocity = ptr->twist.linear.x;
        v2m_lock.unlock();
    }

    void curposeCB(const geometry_msgs::PoseStampedConstPtr& ptr){
        v2m_lock.lock();        
        v2m.curpos = *ptr;
        v2m_lock.unlock();
    }   

    //hc
    //avoid obstacle 
    void addVirtualTrackedObject(autoware_msgs::DetectedObjectArray& d_obj, std::string map_type){
        v2m_lock.lock();
        double z_cur = v2m.curpos.pose.position.z;
        v2m_lock.unlock();
        if (map_type == "kcity"){
            autoware_msgs::DetectedObject d;
        
            d.id = 27158; //kw
            d.label = "unknown";

            std::vector<geometry_msgs::Point32> points;
            geometry_msgs::Point32 p;
            p.z = z_cur;

            p.x = 151.820;
            p.y = -178.255;
            points.push_back(p);
            p.x = 150.749;
            p.y = -177.216;
            points.push_back(p);
            p.x = 149.700;
            p.y = -179.692;
            points.push_back(p);
            p.x = 150.781;
            p.y = -180.104;
            points.push_back(p);


            p.x = 149.773;
            p.y = -181.974;
            points.push_back(p);
            p.x = 148.631;
            p.y = -181.332;
            points.push_back(p);
            p.x = 147.769;
            p.y = -182.601;
            points.push_back(p);
            p.x = 149.008;
            p.y = -183.247;
            points.push_back(p);
            p.x = 148.068;
            p.y = -184.922;
            points.push_back(p);
            p.x = 147.232;
            p.y = -184.002;
            points.push_back(p);
            p.x = 146.206;
            p.y = -185.829;
            points.push_back(p);
            p.x = 147.515;
            p.y = -186.264;
            points.push_back(p);


            p.x = 146.120;
            p.y = -188.451;
            points.push_back(p);
            p.x = 145.226;
            p.y = -187.466;
            points.push_back(p);
            p.x = 144.362;
            p.y = -188.790;
            points.push_back(p);
            p.x = 145.651;
            p.y = -189.394;
            points.push_back(p);
            p.x = 145.145;
            p.y = -190.617;
            points.push_back(p);
            p.x = 143.053;
            p.y = -189.340;
            points.push_back(p);
            p.x = 143.009;
            p.y = -190.985;
            points.push_back(p);
            p.x = 144.121;
            p.y = -192.334;
            points.push_back(p);
            d.convex_hull.polygon.points = points;

            d.velocity_reliable = true;
            d.indicator_state = 3;
            
            d_obj.objects.push_back(d);
        }
        else if (map_type == "soongsil") {
            autoware_msgs::DetectedObject d;
            
            d.id = 27158; //kw
            d.label = "unknown";
        
            std::vector<geometry_msgs::Point32> points;
            geometry_msgs::Point32 p;
            p.z = z_cur;

            p.x = 25.343;
            p.y = -5.442;
            points.push_back(p);
            p.x = 26.232;
            p.y = -6.951;
            points.push_back(p);
            p.x = 28.431;
            p.y = -6.959;
            points.push_back(p);
            p.x = 28.791;
            p.y = -5.416;
            points.push_back(p);
            p.x = 29.841;
            p.y = -5.870;
            points.push_back(p);
            p.x = 29.586;
            p.y = -7.070;
            points.push_back(p);

            p.x = 31.812;
            p.y = -6.832;
            points.push_back(p);
            p.x = 32.163;
            p.y = -5.786;
            points.push_back(p);
            p.x = 33.964;
            p.y = -5.889;
            points.push_back(p);
            p.x = 33.958;
            p.y = -7.041;
            points.push_back(p);


            p.x = 35.956;
            p.y = -7.353;
            points.push_back(p);
            p.x = 36.011;
            p.y = -5.905;
            points.push_back(p);
            p.x = 36.262;
            p.y = -6.320;
            points.push_back(p);
            p.x = 38.917;
            p.y = -5.967;
            points.push_back(p);
            p.x = 38.757;
            p.y = -7.324;
            points.push_back(p);

            p.x = 41.510;
            p.y = -7.517;
            points.push_back(p);
            p.x = 41.205;
            p.y = -7.371;
            points.push_back(p);
            p.x = 41.015;
            p.y = -5.974;
            points.push_back(p);
            p.x = 43.215;
            p.y = -6.135;
            points.push_back(p);
            p.x = 44.508;
            p.y = -7.539;
            points.push_back(p);

            d.convex_hull.polygon.points = points;

            d.velocity_reliable = true;
            d.indicator_state = 3;
            
            d_obj.objects.push_back(d);
        }
        else throw std::runtime_error("[mission manager] check map_type!");
    }

    visualization_msgs::MarkerArray genVizMsg(const autoware_msgs::DetectedObjectArray& d_obj){
        visualization_msgs::MarkerArray m_ary;
        
        int id = 0;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "nsns";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        for(const auto& p : d_obj.objects.back().convex_hull.polygon.points){
            geometry_msgs::Point p_msg;
            p_msg.x = p.x;
            p_msg.y = p.y;
            marker.points.push_back(p_msg);
        }
        marker.scale.x = 0.3;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        m_ary.markers.push_back(marker);
        return m_ary;
    }

    void objects_infoCB(const autoware_msgs::DetectedObjectArrayConstPtr& msg){
        m2v_lock.lock();
        bool isAvoidMission = m2v.isAvoidMission;
        m2v_lock.unlock();

        if (m2v.isAvoidMission){
            auto d_obj = *msg;
            addVirtualTrackedObject(d_obj, "kcity");
            objects_pub.publish(d_obj);
            
            //visualize
            auto viz_msg = genVizMsg(d_obj);
            tracked_obj_pub.publish(viz_msg);
        }
    }

    //traffic light
    void traffic_lightCB(const traffic_light_msgs::LightConstPtr& ptr){
        v2m_lock.lock();
        v2m.cur_light = *ptr;
        v2m.cur_light_updated = true;
        v2m_lock.unlock();
    }

    void twist_cmdCB(const geometry_msgs::TwistStampedConstPtr& ptr){
        m2v_lock.lock();
        bool isParkingMission = m2v.isParkingMission;
        bool isAvoidMission = m2v.isAvoidMission;
        m2v_lock.unlock();

        if (false == isParkingMission && false == isAvoidMission) publish_erp42_twist_cmd(*ptr);
    }

    void cmd_velCB(const geometry_msgs::TwistConstPtr& ptr){
        m2v_lock.lock();
        bool isParkingMission = m2v.isParkingMission;
        m2v_lock.unlock();
        if (true == isParkingMission) {
            geometry_msgs::TwistStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.twist = *ptr;
            publish_erp42_twist_cmd(msg);
        }
    }
    void twist_cmd_purepursuitCB(const geometry_msgs::TwistStampedConstPtr& ptr){
        m2v_lock.lock();
        bool isAvoidMission = m2v.isAvoidMission;
        m2v_lock.unlock();

        if (true == isAvoidMission) publish_erp42_twist_cmd(*ptr);
    }
    void publish_erp42_twist_cmd(const geometry_msgs::TwistStamped& msg){
        erp42_twist_pub.publish(msg);
    }
    void obstacle_center_points_CB_quadtree(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& ptr){
        double x;
        double y;
        geometry_msgs::Point32 p;
        std::vector<geometry_msgs::Point32> obstacle_center_points;
        for(auto& box : ptr->boxes){
            double x = box.pose.position.x;
            double y = box.pose.position.y;
            p.x = x; p.y = y;
            obstacle_center_points.push_back(p);
        }

        v2m_lock.lock();
        v2m.obstacle_center_points_updated = true;
        v2m.obstacle_center_points = obstacle_center_points;
        v2m_lock.unlock();
    }
private:
    //mission related members
    MissionInfo* cur_mission;
    MissionInfo* mission_none;
    std::vector<MissionInfo> mission_infos;
    int mission_identifier_cur;
  
    //재훈코드
    jsk_recognition_msgs::PolygonArray polyArr;
    visualization_msgs::MarkerArray markera; //jaehoon

    //concurrency members
    Vehicle2Mission v2m;
    Mission2Vehicle m2v;
    std::mutex v2m_lock;
    std::mutex m2v_lock;

    //ros members
    //sub
    ros::Subscriber traffic_light_sub; //traffic light detection
    ros::Subscriber curposeSub;
    ros::Subscriber curTwistSub;
    ros::Subscriber objects_sub;  //avoid obstacle
    ros::Subscriber twist_cmd_sub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber twist_purepursuit_sub;
    ros::Subscriber objects_center_points_sub;
    //pub
    ros::Publisher erp42_twist_pub;
    ros::Publisher poly_pub;
    ros::Publisher marker_pub;
    ros::Publisher objects_pub; //avoid obstacle
    ros::Publisher tracked_obj_pub;
    //nh
    ros::NodeHandle nh;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "mission_manager");
    MissionManager m;
    ros::spin();
}