#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>

struct Waypoint{
        double x, y, z, yaw, velocity;
        bool change_flag;
};

void error_handling(std::string str);

class Waypoint2Vectormap{//capable with openplanner vectormap
public:
    void parseWaypointCSV(std::string filename){
        std::ifstream in(filename);
        if (!in.is_open()) error_handling("no such file : " + filename);

        std::string line, element;
        Waypoint w;
        std::getline(in, line);//flush first line

        while(true){
            std::getline(in, line);
            if (in.eof()) break;

            std::stringstream ss_line(line);

            std::getline(ss_line, element, ',');
            w.x = std::stod(element);
            std::getline(ss_line, element, ',');
            w.y = std::stod(element);
            std::getline(ss_line, element, ',');
            w.z = std::stod(element);
            std::getline(ss_line, element, ',');
            w.yaw = std::stod(element);
            std::getline(ss_line, element, ',');
            w.velocity = std::stod(element);
            std::getline(ss_line, element);
            w.change_flag = element == "0";

            waypointVec.push_back(w);
        }    

        std::cout << "x,y,z,yaw,velocity,change_flag\n";
        for(int i = 0; i < 10; ++i)
            std::cout << waypointVec[i].x << ' ' 
                      << waypointVec[i].y << ' ' 
                      << waypointVec[i].z << ' ' 
                      << waypointVec[i].yaw << ' '
                      << waypointVec[i].velocity << ' '
                      << waypointVec[i].change_flag << '\n';
        sz_point = waypointVec.size();
    }  

    void genData(){
        const size_t ID_STARTNUM = 10001;
        pid_vec.resize(sz_point);
        nid_vec.resize(sz_point);
        did_vec.resize(sz_point);
        Lnid_vec.resize(sz_point - 1);
        Bnid_vec.resize(sz_point - 1);
        Fnid_vec.resize(sz_point - 1);
        H_vec.resize(sz_point);
        Bx_vec.resize(sz_point);
        Ly_vec.resize(sz_point);
        dir_vec.resize(sz_point);
        r_vec.resize(sz_point);
        for(size_t i = 0 ; i < sz_point - 1; ++i){
            pid_vec[i]      = ID_STARTNUM + i;
            nid_vec[i]      = ID_STARTNUM + i;
            did_vec[i]      = ID_STARTNUM + i;
            Lnid_vec[i]     = ID_STARTNUM + i;
            Bnid_vec[i]     = ID_STARTNUM + i;
            Fnid_vec[i]     = ID_STARTNUM + i + 1;
            H_vec[i]        = waypointVec[i].z;
            Bx_vec[i]       = waypointVec[i].y;
            Ly_vec[i]       = waypointVec[i].x;
        }
        pid_vec.back() = ID_STARTNUM + sz_point - 1;
        nid_vec.back() = ID_STARTNUM + sz_point - 1;
        did_vec.back() = ID_STARTNUM + sz_point - 1;
        H_vec.back() = waypointVec.back().z;
        Bx_vec.back() = waypointVec.back().y;
        Ly_vec.back() = waypointVec.back().x;

        //for dtlane
        r_vec[0] = r_vec[1] = 90000000000;
        dir_vec[0] = dir_vec[1] = std::atan((Bx_vec[1] - Bx_vec[0]) / (Ly_vec[1] - Ly_vec[0]));
        
        for(size_t i = 0 ; i < sz_point - 2; ++i){
            double dx = Bx_vec[i+2] - Bx_vec[i+1];
            double dy = Ly_vec[i+2] - Ly_vec[i+1];
            dir_vec[i+2] = std::atan(dx / dy);
            r_vec[i+2] = 1 / (dir_vec[i+2] - dir_vec[i+1]);
        }
        
    }

    void saveAsAutowareVectormap(std::string dst_dir){
        if (dst_dir.back() != '/') dst_dir += '/';
        
        std::ofstream idx(dst_dir + "idx.csv"),
                    dtlane(dst_dir + "dtlane.csv"),
                    lane(dst_dir + "lane.csv"),
                    node(dst_dir + "node.csv"),
                    point(dst_dir + "point.csv");
        
        idx <<  "ID,KIND,fname\n"
            <<  "1,K001,poledata.csv\n"
            <<  "2,K002,utilitypole.csv\n"
            <<  "3,K003,roadsign.csv\n"
            <<  "4,K004,signaldata.csv\n"
            <<  "5,K005,streetlight.csv\n"
            <<  "6,P001,whiteline.csv\n"
            <<  "7,P002,stopline.csv\n"
            <<  "8,P003,zebrazone.csv\n"
            <<  "9,P004,crosswalk.csv\n"
            <<  "10,P005,road_surface_mark.csv\n"
            <<  "11,R001,roadedge.csv\n"
            <<  "12,R002,gutter.csv\n"
            <<  "13,R003,curb.csv\n"
            <<  "14,S001,guardrail.csv\n";

        node << "NID,PID\n";
        for(size_t i = 0 ; i < sz_point; ++i)
            node << nid_vec[i] << ',' << pid_vec[i] << '\n';

        point << "PID,B(Lat),L(Long),H,Bx,Ly,ReF,MCODE1,MCODE2,MCODE3\n";
        for (size_t i = 0 ; i < sz_point; ++i)
            point << pid_vec[i] << ",0,0," << H_vec[i] << ',' << Bx_vec[i] << ',' << Ly_vec[i] << ',' <<"0,0,0,0\n";

        lane << "LnID,DID,BLID,FLID,BNID,FNID,JCT,BLID2,BLID3,BLID4"
            << ",FLID2,FLID3,FLID4,CrossID,Span,LCnt,Lno,LaneType,LimitVel,RefVel,RoadSecID,LaneChgFG,\n";
        for(size_t i = 0 ; i < sz_point - 1; ++i)
        lane << Lnid_vec[i] << ',' << did_vec[i] << ',' << "0,0," << Bnid_vec[i] << ',' << Fnid_vec[i] << ','
            << "0,0,0,0,0,0,0,0,1,1,1,0,20,20,0,0,0\n";

        dtlane << "DID,Dist,PID,Dir,Apara,r,slope,cant,LW,RW\n";
        for(size_t i = 0 ; i < sz_point; ++i)
            dtlane << did_vec[i] << ',' << i << ',' << pid_vec[i] << ',' << dir_vec[i] << ",0," 
            << r_vec[i] <<",0,0,2,2\n";
        idx.close(); dtlane.close(); node.close(); point.close(); lane.close();
    }


private:
    std::vector<Waypoint> waypointVec;
    size_t sz_point;
    std::vector<int> pid_vec, nid_vec, did_vec, Lnid_vec, Bnid_vec, Fnid_vec;
    std::vector<double> H_vec, Bx_vec, Ly_vec, dir_vec, r_vec; // // 
};



int main(int argc, char *argv[]){
    std::cout << "This code uses the output from - \n"
                << "https://tools.tier4.jp/feature/vector_map_builder_ll2/"
                << "'s waypoint. \nThe format is "
                << "x,y,z,yaw,velocity,change_flag\\n\n"
                << "only single waypoint is capable with this software\n";
    
    if (argc != 3)
        error_handling("Usage : %s <waypoint.csv> <target_dir>\n");

    Waypoint2Vectormap w2v;
    w2v.parseWaypointCSV(argv[1]);
    w2v.genData();
    w2v.saveAsAutowareVectormap(argv[2]);
}



void error_handling(std::string str){
    std::cerr << str << '\n';
    exit(3);
}
