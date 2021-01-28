#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <sstream>
/* 
    input file format must be 
    x1,y1,x1,y1
    1,2,3,4
    5,6,7,8
    ...
    */
class PGMMaker{
public:
    struct XY{
        double x, y;
    };
    struct Line{
        Line(XY p1, XY p2) : p1(p1), p2(p2) {}
        XY p1, p2;
    };
    typedef std::vector<std::vector<uint8_t>> PGMMapType;
public:
    PGMMaker(double x_start_meter, 
        double y_start_meter, 
        double width_meter, 
        double height_meter, 
        double resolution,
        double thickness) : 
        x_start_meter(x_start_meter),
        y_start_meter(y_start_meter),
        width_meter(width_meter),
        height_meter(height_meter),
        resolution(resolution),
        thickness_meter(thickness)
        { 
        if (width_meter <= 0) {std::cout << "width_meter <= 0\n"; exit(1);}
        if (height_meter <= 0) {std::cout << "height_meter <= 0\n"; exit(1);}

        //discretize real values
        width_pgm = (unsigned)(width_meter / resolution);
        height_pgm = (unsigned)(height_meter / resolution);
    }
    PGMMapType getPGMmap(){
        std::vector<std::vector<uint8_t>> pgm(height_pgm); //y
        for(unsigned i = 0 ; i < height_pgm; ++i){
            pgm[i] = std::vector<uint8_t>(width_pgm);
            std::fill_n(pgm[i].begin(), pgm[i].size(), 205);
        }
        std::cout << pgm.size() << ' ' << pgm.back().size() << '\n';
        return pgm;
    }

    auto XYtoIndex(XY xy){
        unsigned x_idx = (unsigned)((xy.x - x_start_meter) / resolution);
        x_idx = std::max<unsigned>(x_idx, 0); x_idx = std::min<unsigned>(x_idx, width_pgm - 1);

        unsigned y_idx = (unsigned)((xy.y - y_start_meter) / resolution);
        y_idx = std::max<unsigned>(y_idx, 0); y_idx = std::min<unsigned>(y_idx, height_pgm - 1);

        return std::make_pair(x_idx, y_idx);
    }

    void savePGM(const PGMMapType& map, std::string path, std::string map_name){
        if (path.back() != '/') path += '/';

        std::ofstream map_yaml(path + map_name + ".yaml");
        map_yaml << "image: ./" << map_name << ".pgm" << '\n';
        map_yaml << "resolution: " << resolution << '\n';
        map_yaml << "origin: " << "[" << x_start_meter << ", " << y_start_meter << ", 0.0]\n";
        map_yaml << "negate: 0\n";
        map_yaml << "occupied_thresh: 0.9\n";
        map_yaml << "free_thresh: 0.1\n\n";
        map_yaml.close();

        std::ofstream map_pgm(path + map_name + ".pgm");
        map_pgm << "P5\n"; //pgm magic number
        map_pgm << "# CREATOR: tw " << resolution << "00 m/pix\n";
        map_pgm << width_pgm << ' ' << height_pgm << '\n';
        map_pgm << 255 << '\n'; //1byte maximum value
        
        size_t cnt = 0;
        for(unsigned y = 0 ; y < height_pgm; ++y){
            for(unsigned x = 0; x < width_pgm; ++x){
                map_pgm << map[height_pgm - 1 - y][x];
                cnt++;
            }
        }
        std::cout << "cnt : " << cnt << '\n';
        map_pgm.flush();
        map_pgm.close();
        std::cout << "saved!!\n";
    }

    void readLineCSV(std::string filename){
        std::ifstream in(filename);
        if (!in.is_open()){
            std::cout << "error when open : " << filename << '\n';
            exit(10);
        }
        std::string line, word;
        std::stringstream ss;

        lineVec.clear();
        double x1, y1, x2, y2;
        std::getline(in, line); //flush first line
        while(true){
            std::getline(in, line);
            //std::cout << "line : " << line << '\n';
            if (in.eof()) break;

            ss.clear(); ss.str(line);

            std::getline(ss, word, ',');
            x1 = std::stod(word);
            std::getline(ss, word, ',');
            y1 = std::stod(word);
            std::getline(ss, word, ',');
            x2 = std::stod(word);
            std::getline(ss, word, ',');
            y2 = std::stod(word);
            
            XY p1{x1, y1}, p2{x2, y2};
            lineVec.emplace_back(p1, p2);
        }

        double x_min = 100000, y_min=100000, x_max = -100000, y_max=-100000;
        for(auto& line : lineVec){
            if (x_min > line.p1.x) x_min = line.p1.x;
            if (x_min > line.p2.x) x_min = line.p1.x;
            if (x_max < line.p1.x) x_max = line.p2.x;
            if (x_max < line.p2.x) x_max = line.p2.x;
            if (y_min > line.p1.y) y_min = line.p1.y;
            if (y_min > line.p2.y) y_min = line.p1.y;
            if (y_max < line.p1.y) y_max = line.p2.y;
            if (y_max < line.p2.y) y_max = line.p2.y;
        }
        std::cout << "xmin, xmax, ymin, ymax : " << x_min << ' ' << x_max << ' ' << y_min << ' ' << y_max <<'\n';
        bReadLine = true;
    }

    void drawLineInPGM(PGMMapType& map){
        if (!bReadLine){
            std::cerr << "call readLineCSV() before this function " <<'\n';
            exit(10);
        }

        constexpr uint8_t OCCUPIED = 0;
        
        for(auto line : lineVec){
            auto [x_idx_start, y_idx_start] = XYtoIndex(line.p1);
            auto [x_idx_end, y_idx_end] = XYtoIndex(line.p2);

            //if (x_idx_start > x_idx_end) std::swap(x_idx_start, x_idx_end);
            //if (y_idx_start > y_idx_end) std::swap(y_idx_start, y_idx_end);
            //std::cout << x_idx_start << ' ' << x_idx_end << ' ' << y_idx_start << ' ' << y_idx_end << '\n';

            int x_gap = x_idx_end - x_idx_start;
            int y_gap = y_idx_end - y_idx_start;
            int longer_gap = std::max(std::abs(x_gap), std::abs(y_gap));

            for(int i = 0 ; i < longer_gap; ++i){
                int x_idx = x_idx_start + 1.0 * (x_gap) / longer_gap * i;
                int y_idx = y_idx_start + 1.0 * (y_gap) / longer_gap * i;
                map[y_idx][x_idx] = OCCUPIED;
            }
        }

        auto IDX_Y = [=](int y_idx) {
            int idx = y_idx; 
            idx = std::max<int>(idx, 0); idx = std::min<int>(idx, height_pgm - 1);
            return idx;};
        auto IDX_X = [=](int x_idx) {
            int idx = x_idx; 
            idx = std::max<int>(idx, 0); idx = std::min<int>(idx, width_pgm - 1);
            return idx;};
        
        //inflate layers
        PGMMapType map_tmp;
        int inflation_iteration = thickness_meter / resolution / 2 + 1;
        std::cout << "thickness iteration : " << inflation_iteration << '\n';
        for (int i = 0 ; i < inflation_iteration; ++i){
            map_tmp = map;
            for(int y = 0 ; y < height_pgm; ++y){
                for (int x = 0; x < width_pgm; ++x){
                    if (map[y][x] == OCCUPIED){
                        map_tmp[IDX_Y(y-1)][IDX_X(x)] = OCCUPIED;
                        map_tmp[IDX_Y(y)][IDX_X(x-1)] = OCCUPIED;
                        map_tmp[IDX_Y(y+1)][IDX_X(x)] = OCCUPIED;
                        map_tmp[IDX_Y(y)][IDX_X(x+1)] = OCCUPIED;
                    }
                }
            }
            map = map_tmp;
        } 
    }

    
private:
    double x_start_meter, y_start_meter, width_meter, height_meter;
    double resolution;
    double thickness_meter;
    unsigned width_pgm, height_pgm;
    std::vector<Line> lineVec;
    bool bReadLine;
};


int main(int argc, char *argv[]){
    if (argc != 9){
        std::cout << "usage : " << argv[0] << " <x_start> <y_start> <width> <height> <resolution> <thickness> <path> <map_name>. all units are meter\n";
        exit(18);
    }

    PGMMaker p(std::atof(argv[1]),
                std::atof(argv[2]),
                std::atof(argv[3]),
                std::atof(argv[4]),
                std::atof(argv[5]),
                std::atof(argv[6]));
    auto m = p.getPGMmap();
    p.readLineCSV("pparking.csv");
    p.drawLineInPGM(m);
    p.savePGM(m, argv[7], argv[8]);
}