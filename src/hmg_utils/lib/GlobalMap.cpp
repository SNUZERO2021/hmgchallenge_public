#include <iostream>
#include "ros/ros.h"
#include "GlobalMap.h"

using namespace std;

GlobalMap::GlobalMap(){
}



GlobalMap::GlobalMap(double a1, double b1, double c1, double a2, double b2, double c2, int w, int h){
    this->a1 = a1;
    this->b1 = b1;
    this->c1 = c1;
    this->a2 = a2;
    this->b2 = b2;
    this->c2 = c2;
    width = w;
    height = h;
}

// void GlobalMap::loadSegmentMap(string segment_map_path){
//     const char delimiter = ' ';
//     string in_line;
//     ifstream in(segment_map_path);
//     while(getline(in, in_line)){
//         stringstream ss(in_line);
//         string token;
//         vector<string> tokens;
//         while(getline(ss,token,delimiter)) tokens.push_back(token);
//         int px = stoi(tokens[0]);
//         int py = stoi(tokens[1]);
//         vector<int> seg;
//         for(int i=2;i<tokens.size();i++) seg.push_back(stoi(tokens[i]));
//         global_segment_map.push_back(PNode(px,py,seg));
//     }
// }

// void GlobalMap::loadSegmentMap(string segment_map_path){
//     const char delimiter = ' ';
//     string in_line;
//     ifstream in(segment_map_path);
//     while(getline(in, in_line)){
//         stringstream ss(in_line);
//         string token;
//         vector<string> tokens;
//         while(getline(ss,token,delimiter)) tokens.push_back(token);
//         int px = stoi(tokens[0]);
//         int py = stoi(tokens[1]);
//         vector<int> seg;
//         for(int i=2;i<tokens.size();i++) seg.push_back(stoi(tokens[i]));
//         global_segment_map[int(px*GW/width)][int(py*GH/height)].push_back(PNode(px,py,seg));
//     }
// }

void GlobalMap::loadSegmentMap(string segment_map_path){
    const char delimiter = ' ';
    string in_line;
    ifstream in(segment_map_path);
    while(getline(in, in_line)){
        stringstream ss(in_line);
        string token;
        vector<string> tokens;
        while(getline(ss,token,delimiter)) tokens.push_back(token);
        int px = stoi(tokens[0]);
        int py = stoi(tokens[1]);
        vector<int> seg;
        for(int i=2;i<tokens.size();i++) seg.push_back(stoi(tokens[i]));
        global_segment_map[px*HASH + py]= seg;
    }
}


void GlobalMap::loadIntersectionMap(string intersection_map_path){
    const char delimiter = ' ';
    string in_line;
    ifstream in(intersection_map_path);
    while(getline(in, in_line)){
        stringstream ss(in_line);
        string token;
        vector<string> tokens;
        while(getline(ss,token,delimiter)) tokens.push_back(token);
        int px = stoi(tokens[0]);
        int py = stoi(tokens[1]);
        global_intersection_map[px*HASH + py]= true;
    }
}

void GlobalMap::loadHeightMap(string height_map_path){
    const char delimiter = ' ';
    string in_line;
    ifstream in(height_map_path);
    while(getline(in, in_line)){
        stringstream ss(in_line);
        string token;
        vector<string> tokens;
        while(getline(ss,token,delimiter)) tokens.push_back(token);
        int px = stoi(tokens[0]);
        int py = stoi(tokens[1]);
        double h = stod(tokens[2]);
        global_height_map[px*HASH + py]= h;
    }
}

void GlobalMap::loadSegmentInfo(string segment_info_path){
    const char delimiter = ' ';
    ifstream in(segment_info_path);
    string first_line;
    getline(in, first_line);
    int seg_cnt = stoi(first_line);
    for(int i = 0;i<seg_cnt;i++){
        string in_line; stringstream ss; string token; vector<string> tokens;
        getline(in, in_line);
        ss = stringstream(in_line);
        tokens.clear();
        while(getline(ss,token, delimiter)) tokens.push_back(token);
        int index = stoi(tokens[0]);
        int point_size = stoi(tokens[1]);
        int cubic_size = point_size - 1;
        vector<string> id({tokens[2], tokens[3], tokens[4]});

        getline(in, in_line);
        ss = stringstream(in_line);
        tokens.clear();
        while(getline(ss,token, delimiter)) tokens.push_back(token);
        vector<int> edge;
        for(int i = 0;i<15;i++) edge.push_back(stoi(tokens[i]));

        getline(in, in_line);
        ss = stringstream(in_line);
        tokens.clear();
        while(getline(ss,token, delimiter)) tokens.push_back(token);
        double dist = stod(tokens[0]);

        getline(in, in_line);
        ss = stringstream(in_line);
        tokens.clear();
        while(getline(ss,token, delimiter)) tokens.push_back(token);
        vector<double> boundary;
        for(int i = 0;i<4;i++) boundary.push_back(stod(tokens[i]));

        vector<double> x, y, z, s;
        for(int j = 0;j<point_size;j++){
            getline(in, in_line);
            ss = stringstream(in_line);
            tokens.clear();
            while(getline(ss,token, delimiter)) tokens.push_back(token);
            x.push_back(stod(tokens[0]));
            y.push_back(stod(tokens[1]));
            z.push_back(stod(tokens[2]));
            s.push_back(stod(tokens[3]));
        }

        vector<vector<double>> cx(4), cy(4), cz(4);
        for(int j = 0;j<cubic_size;j++){
            getline(in, in_line);
            ss = stringstream(in_line);
            tokens.clear();
            while(getline(ss,token, delimiter)) tokens.push_back(token);
            for(int k =0;k<4;k++){
                cx[k].push_back(stod(tokens[k]));
                cy[k].push_back(stod(tokens[k+4]));
                cz[k].push_back(stod(tokens[k+8]));
            }
        }

        segment.push_back(CubicSpline2D(x, y, z, s, cx, cy, cz, edge, id, boundary, point_size, index, dist));
    }
}
    

pii GlobalMap::XYToPixel(double x, double y){
    int px = int(a1*x+b1*y+c1);
    int py = int(a2*x+b2*y+c2);
    return {px, py};
}

pdd GlobalMap::PixelToXY(int px, int py){
    double x = (double)(px - c1) / a1;
    double y = (double)(py - c2) / b2;
    return {x, y};
}

// vector<int> GlobalMap::getSegmentArray(double x, double y){
//     pii p = XYToPixel(x,y);
//     int px,py;
//     int l = 0, r=global_segment_map.size();
//     while(r-l>1){
//         int m = (l+r)>>1;
//         px = global_segment_map[m].x;
//         py = global_segment_map[m].y;
//         if(p.first == px && p.second == py){
//             l = m;
//             break;
//         }
//         if(p.first<px){
//             r = m;
//             continue;
//         }
//         else if(p.first == px){
//             if(p.second < py){
//                 r = m;
//                 continue;
//             }
//         }
//         l = m;
//     }
//     px = global_segment_map[l].x;
//     py = global_segment_map[l].y;
//     if(p.first == px && p.second == py) return global_segment_map[l].segments;
//     else{
//         vector<int> v;
//         return v;
//     }
// }

// vector<int> GlobalMap::getSegmentArray(double x, double y){
//     pii p = XYToPixel(x,y);
//     int px,py;
//     int w,h;
//     w = int(px*GW/width);
//     h = int(py*GH/height);
//     int l = 0, r=global_segment_map[w][h].size();
//     if(r == 0) {
//         vector<int> v;
//         return v;
//     }
//     while(r-l>1){
//         int m = (l+r)>>1;
//         px = global_segment_map[w][h][m].x;
//         py = global_segment_map[w][h][m].y;
//         if(p.first == px && p.second == py){
//             l = m;
//             break;
//         }
//         if(p.first<px){
//             r = m;
//             continue;
//         }
//         else if(p.first == px){
//             if(p.second < py){
//                 r = m;
//                 continue;
//             }
//         }
//         l = m;
//     }
//     px = global_segment_map[w][h][l].x;
//     py = global_segment_map[w][h][l].y;
//     if(p.first == px && p.second == py) return global_segment_map[w][h][l].segments;
//     else{
//         vector<int> v;
//         return v;
//     }
// }

vector<int> GlobalMap::getSegmentArray(double x, double y){
    pii p = XYToPixel(x,y);
    return global_segment_map[p.first*HASH + p.second];
}

double GlobalMap::getHeight(double x, double y){
    pii p = XYToPixel(x,y);
    return global_height_map[p.first*HASH + p.second];
}

bool GlobalMap::isIntersection(double x, double y){
    pii p = XYToPixel(x,y);
    return global_intersection_map[p.first*HASH + p.second];
}

vector<hmg_utils::BoxArray> GlobalMap::PredictionService(const vector<hmg_utils::Prediction> &predictions, const vector<double> &times){
    double time_threshold_ = 3.0;
    if(predictions.size()==0 || predictions[0].t.size() == 0) {
        vector<hmg_utils::BoxArray> rt;
        return rt;
    }
    double init_time_ =  predictions[0].t[0];
    
    vector<hmg_utils::BoxArray> rt;
    for(double t : times){
        hmg_utils::BoxArray cur;
        cur.time = t;
        vector<hmg_utils::Box> boxes;
        for(hmg_utils::Prediction obj : predictions){
            int size = obj.t.size();
            CubicSpline2D seg = segment[obj.segment_index];
            hmg_utils::Box box;
            box.object_info = obj.object_info;
            //if(t < obj.t[0] || t> obj.t[size-1]) continue;
            if(t-init_time_ > time_threshold_) break;
            int l = 0, r = size -1;
            while(r-l>1){
                int m = (l+r) >> 1;
                if(obj.t[m]>t) r = m;
                else l = m;
            }
            double t1 = obj.t[l], t2=obj.t[l+1];
            double ss[2],ll[2];
            ss[0] = (obj.s_min[l]*(t2-t)+obj.s_min[l+1]*(t-t1))/(t2-t1) - obj.object_info.length/2;
            ss[1] = (obj.s_max[l]*(t2-t)+obj.s_max[l+1]*(t-t1))/(t2-t1) + obj.object_info.length/2;
            ll[0] = (obj.l_min[l]*(t2-t)+obj.l_min[l+1]*(t-t1))/(t2-t1) - obj.object_info.width/2;
            ll[1] = (obj.l_max[l]*(t2-t)+obj.l_max[l+1]*(t-t1))/(t2-t1) + obj.object_info.width/2;
            box.v = (obj.v[l]*(t2-t)+obj.v[l+1]*(t-t1))/(t2-t1);
            box.a = (obj.a[l]*(t2-t)+obj.a[l+1]*(t-t1))/(t2-t1);
            box.segment_index = obj.segment_index;
            box.t = t;
            
            vector<double> x,y;
            for(int i=0;i<2;i++){
                for(int j=0;j<2;j++){
                    PoseState ps = seg.sl_to_xy(SLState(ss[i],ll[j]));
                    x.push_back(ps.x);
                    y.push_back(ps.y);
                }
            }
            box.x = x;
            box.y = y;
            box.s_min = ss[0];
            box.s_max = ss[1];
            boxes.push_back(box);
        }
        cur.boxes = boxes;
        rt.push_back(cur);
    }
    return rt;
}

vector<hmg_utils::BoxArray> GlobalMap::PedestrianService(const vector<hmg_utils::Pedestrian> &predictions, const vector<double> &times){
    double time_threshold_ = 3.0;
    if(predictions.size()==0 || predictions[0].t.size()==0) {
        vector<hmg_utils::BoxArray> rt;
        return rt;
    }
    double init_time_ = predictions[0].t[0];

    vector<hmg_utils::BoxArray> rt;
    for(double t : times){
        hmg_utils::BoxArray cur;
        cur.time = t;
        vector<hmg_utils::Box> boxes;
        for(hmg_utils::Pedestrian obj : predictions){
            int size = obj.t.size();
            hmg_utils::Box box;
            box.object_info = obj.object_info;
            if(t - init_time_ > time_threshold_) break;
            int l = 0, r = size -1;
            while(r-l>1){
                int m = (l+r) >> 1;
                if(obj.t[m]>t) r = m;
                else l = m;
            }
            double t1 = obj.t[l], t2=obj.t[l+1];
            double xx[2],yy[2];
            xx[0] = (obj.x[l]*(t2-t)+obj.x[l+1]*(t-t1))/(t2-t1) - 1.0;
            xx[1] = (obj.x[l]*(t2-t)+obj.x[l+1]*(t-t1))/(t2-t1) + 1.0;
            yy[0] = (obj.y[l]*(t2-t)+obj.y[l+1]*(t-t1))/(t2-t1) - 1.0;
            yy[1] = (obj.y[l]*(t2-t)+obj.y[l+1]*(t-t1))/(t2-t1) + 1.0;
            box.v = hypot(obj.vx[l],obj.vy[l]);
            box.t = t;
            
            vector<double> x,y;
            for(int i=0;i<2;i++){
                for(int j=0;j<2;j++){
                    x.push_back(xx[i]);
                    y.push_back(yy[j]);
                }
            }
            box.x = x;
            box.y = y;
            boxes.push_back(box);
        }
        cur.boxes = boxes;
        rt.push_back(cur);
    }
    return rt;
}