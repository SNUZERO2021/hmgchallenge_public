#include "ros/ros.h"
#include "ros/package.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <dirent.h>
#include <sys/types.h>
#include <algorithm>
#include <vector>
#include <string>
#include <map>

#include "cubic_spline_2d.h"

using namespace std;

typedef pair<int, int> pii;
typedef pair<double, double> pdd;

int idcnt = 0;
map<pii, int> mm;
vector<vector<int>> G, GG;

pii XY_To_Pixel(double x, double y){
    double a1 = 10;
    double b1 = 0;
    double c1 = 13500;
    double a2 = 0;
    double b2 = 10;
    double c2 = 15000;
   
    int px = int(a1*x+b1*y+c1);
    int py = int(a2*x+b2*y+c2);
    return {px, py};
}

pdd Pixel_To_XY(int px, int py){
    double a1 = 0.1;
    double b1 = 0.0;
    double c1 = -1350.0;
    double a2 = 0.0;
    double b2 = 0.1;
    double c2 = -1500.0;
   
    double x = a1*px+b1*py+c1;
    double y = a2*px+b2*py+c2;
    return {x, y};
}

char add_one(char c){
    if(c == '9') return 'A';
    else return c+1;
}

char sub_one(char c){
    if(c == 'A') return 9;
    else return c-1;
}

int char_to_num(char c){
    if('0' <= c && c <= '9') return c - '0';
    else return c - 'A';
}

string format_string(vector<string>&names, int i){
    if(i==-1) return "\t\t\t";
    if(names[i].size()==7){
        string ret = names[i];
        ret += '\t';
        ret += '\t';
        return ret;
    }
    else{
        string ret = names[i];
        return ret;
    }
}

int match_string(vector<string> &names, string expr){
    for(int i = 0;i<names.size();i++){
        string str = names[i];
        if(expr.size() != str.size()) continue;

        bool match = true;
        for(int j = 0; j<expr.size();j++){
            if(expr[j] == '#') continue;
            if(expr[j] != str[j]) match = false;
        }
        if(match) return i;
    }
    return -1;
}

void build_graph(vector<string>& names){
    G = vector<vector<int>>(names.size(), vector<int>(6, -1));
    printf("names size %d\n", (int)names.size());
    for(int i =0;i<names.size();i++){
        for(int k = 0;k<6;k++) G[i][k] = -1;
        string str = names[i];
        cout << str << endl;
        if(str.length() == 7){
            string expr;

            // next_straight
            expr = "############";
            expr[0] = str[0];
            expr[1] = str[1];
            expr[2] = str[2];
            expr[3] = str[3];
            expr[6] = '_';
            expr[7] = str[5];
            expr[8] = str[6];
            expr[9] = 'S';
            G[i][0] = match_string(names, expr);
            if(G[i][0] == -1){
                expr = str;
                expr[5] = add_one(str[5]);
                G[i][0] = match_string(names, expr);
                if(G[i][0]==-1){
                    expr[6] = add_one(str[6]);
                    G[i][0] = match_string(names, expr);
                    if(G[i][0]==-1){
                        expr[6] = sub_one(str[6]);
                        G[i][0] = match_string(names, expr);
                    }
                }
            }

            // next_left
            expr = "############";
            expr[0] = str[0];
            expr[1] = str[1];
            expr[2] = str[2];
            expr[3] = str[3];
            expr[6] = '_';
            expr[7] = str[5];
            expr[8] = str[6];
            expr[9] = 'L';
            G[i][1] = match_string(names, expr);
            

            // next_right
            expr = "############";
            expr[0] = str[0];
            expr[1] = str[1];
            expr[2] = str[2];
            expr[3] = str[3];
            expr[6] = '_';
            expr[7] = str[5];
            expr[8] = str[6];
            expr[9] = 'R';
            G[i][2] = match_string(names, expr);
            

            // left
            expr = str;
            if(str[6]=='0') G[i][3] = -1;
            else{
                expr[6] = sub_one(str[6]);
                G[i][3] = match_string(names, expr);
            }

            // right
            expr = str;
            if(str[6]=='9') G[i][4] = -1;
            else{
                expr[6] = add_one(str[6]);
                G[i][4] = match_string(names, expr);
            }
        }
        else if(str.length() == 12){
            string expr;

            // next_straight
            expr = "#######";
            expr[0] = str[2];
            expr[1] = str[3];
            expr[2] = str[4];
            expr[3] = str[5];
            expr[4] = '_';
            expr[5] = str[10];
            expr[6] = str[11];
            G[i][0] = match_string(names, expr);

            // next_left
            G[i][1] = -1;

            // next_right
            G[i][2] = -1;

            // left
            expr = str;
            if(str[8]=='0' || str[11]=='0') G[i][3] = -1;
            else{
                expr[8] = sub_one(str[8]);
                expr[11] = sub_one(str[11]);
                G[i][3] = match_string(names, expr);
            }

            // right
            expr = str;
            if(str[8]=='9' || str[11]=='9') G[i][4] = -1;
            else{
                expr[8] = add_one(str[8]);
                expr[11] = add_one(str[11]);
                G[i][4] = match_string(names, expr);
            }
        }
        else{
            cout << "assert?" << endl;
            printf("!!!!!!!!!!!!!! name which length is not 7 and 12 !!!!!!!!!!!!!!!!!");
            assert(false);
        }
    }
    // find back segment
    for(int i = 0;i<names.size();i++){
        if(names[i].length() == 7){
            string expr = names[i];
            expr[5] = sub_one(names[i][5]);
            G[i][5] = match_string(names, expr);
            if(G[i][5] != -1) continue;
        }

        for(int j = 0;j<names.size();j++){
            if(G[j][0] == i){
                G[i][5] = j; break;
            }
        }
        if(G[i][5] == -1){
            for(int j = 0;j<names.size();j++){
                if(G[j][1] == i){
                    G[i][5] = j; break;
                }
            }
            if(G[i][5] == -1){
                for(int j = 0;j<names.size();j++){
                    if(G[j][2] == i){
                        G[i][5] = j; break;
                    }
                }
            }
        }
    }
    
    stringstream save_path;
    save_path << ros::package::getPath("global_map") <<"/config/graph_visualize.txt";
    FILE* fpw = fopen(save_path.str().c_str(), "w");
    for(int i = 0;i<names.size();i++){
        fprintf(fpw,"\n");
        fprintf(fpw,"%s\t%s\t%s\n", format_string(names, G[i][1]).c_str(), format_string(names, G[i][0]).c_str(), format_string(names, G[i][2]).c_str());
        fprintf(fpw,"\t\t\t\t  ^\n");
        fprintf(fpw,"\t\t\t\t  |\n");
        fprintf(fpw,"%s<-\t%s->\t%s\n", format_string(names, G[i][3]).c_str(), format_string(names, i).c_str(), format_string(names, G[i][4]).c_str());
        fprintf(fpw,"\t\t\t\t  ^\n");
        fprintf(fpw,"\t\t\t\t  |\n");
        fprintf(fpw,"\t\t\t\t%s\n", format_string(names, G[i][5]).c_str());
        fprintf(fpw,"\n");
        fprintf(fpw,"-----------------------------------------------");
        fprintf(fpw,"\n");
    }
    fclose(fpw);

    idcnt = 0;
    for(int i = 0;i<names.size();i++){
        for(int j = 0;j<3;j++){
            if(G[i][j] == -1) continue;
            mm[{i,j}] = idcnt++;
        }
    }
    GG = vector<vector<int>>(idcnt, vector<int>(15, -1));

    for(int i = 0;i<names.size(); i++){
        for(int j = 0;j<3;j++){
            if(G[i][j] == -1) continue;
            int index = mm[{i,j}];

            for(int k = 0;k<3;k++){
                if(G[G[i][j]][k] == -1) continue;
                GG[index][k] = mm[{G[i][j], k}];
            }
            if(G[i][3] != -1){
                for(int k = 0;k<3;k++){
                    if(G[G[i][3]][k] == -1) continue;
                    GG[index][k+3] = mm[{G[i][3], k}];
                }
            }
            if(G[G[i][j]][3] != -1){
                for(int k = 0;k<3;k++){
                    if(GG[index][k+3] == -1){
                        for(int l = 0;l<names.size();l++){
                            if(G[l][k] == G[G[i][j]][3]){
                                GG[index][k+3] = mm[{l,k}];
                            }
                        }
                    }
                }
            }

            if(G[i][4] != -1){
                for(int k = 0;k<3;k++){
                    if(G[G[i][4]][k] == -1) continue;
                    GG[index][k+6] = mm[{G[i][4], k}];
                }
            }
            if(G[G[i][j]][4] != -1){
                for(int k = 0;k<3;k++){
                    if(GG[index][k+6] == -1){
                        for(int l = 0;l<names.size();l++){
                            if(G[l][k] == G[G[i][j]][4]){
                                GG[index][k+6] = mm[{l,k}];
                            }
                        }
                    }
                }
            }

            GG[index][9] = (names[i].length() == 7) ? 0 : 1;
            GG[index][10] = (names[G[i][j]].length() == 7) ? 0 : 1;
            GG[index][11] = (names[i].length() == 7) ? char_to_num(names[i][5]) : 0;
            GG[index][12] = (names[G[i][j]].length() == 7) ? char_to_num(names[G[i][j]][5]) : 0;
            GG[index][13] = (names[i].length() == 7) ? char_to_num(names[i][6]) : char_to_num(names[i][8]);
            GG[index][14] = (names[i].length() == 7) ? char_to_num(names[i][6]) : char_to_num(names[i][11]);
        }
    }
}

CubicSpline2D smooth(CubicSpline2D & csp, vector<int> & boundary_index, string id1, string id2, string id3){
    static int iter = 5;
    static double alpha = 0.5;
    vector<double> x = csp.get_x();
    vector<double> y = csp.get_y();
    vector<double> original_z = csp.get_z();
    vector<double> original_x = x;
    vector<double> original_y = y;
    int sz = csp.get_size();

    for(int cnt = 0; cnt<iter;cnt++){
        vector<int> permutation;
        for(int i = 0;i<sz;i++) permutation.push_back(i);
        random_shuffle(permutation.begin(), permutation.end());

        for(int i =0;i<sz;i++){
            int p = permutation[i];
            if(p==boundary_index[0] || p==boundary_index[1] || p==boundary_index[2] || p==boundary_index[3]) continue;

            vector<double> new_x, new_y;
            for(int j = 0;j<sz;j++){
                if(j==p) continue;
                new_x.push_back(x[j]);
                new_y.push_back(y[j]);
            }
            CubicSpline2D tmp_csp(new_x,new_y, original_z);
            tmp_csp.set_id(vector<string>({id1, id2, id3}));

            PoseState ps;
            ps.x = original_x[p];
            ps.y = original_y[p];
            SLState sl = tmp_csp.transform(ps);
            sl.l *= alpha;
            ps = tmp_csp.sl_to_xy(sl);

            x[p] = ps.x;
            y[p] = ps.y;
        }   
    }
    return CubicSpline2D(x,y, original_z);
}

CubicSpline2D get_spline(string id1, string id2, string id3){
    if(id1 == "#"){
        string id[2] = {id2, id3};
        string path[2];
        for(int i = 0;i<2;i++){
            stringstream ss;
            ss << ros::package::getPath("global_map") << "/segment/" << id[i] << ".txt";
            path[i] = ss.str();
        }

        vector<int> boundary_index;
        boundary_index.push_back(0);
        boundary_index.push_back(0);
        vector<double> x;
        vector<double> y;
        vector<double> z;
        const char delimiter =  ' ';
        for(int i =0;i<2;i++){
            string in_line;
            ifstream in(path[i]);
            double tmpx, tmpy, tmpz;
            while(getline(in, in_line)){
                stringstream ss(in_line);
                string token;
                vector<string> tokens;
                while(getline(ss,token,delimiter))tokens.push_back(token);
                if(tokens.size()<2) continue;
                double _x = stod(tokens[0]);
                double _y = stod(tokens[1]);
                double _z = stod(tokens[2]);
                tmpx = _x;
                tmpy = _y;
                tmpz = _z;
                if(!x.empty() && !y.empty() && norm(x.back()-_x, y.back() - _y) < 4.0) continue;
                x.push_back(_x);
                y.push_back(_y);
                z.push_back(_z);
            }
            if(x.back() != tmpx){
                if(norm(x.back()-tmpx, y.back()-tmpy) < 2.0){
                    x.pop_back();
                    y.pop_back();
                    z.pop_back();
                    x.push_back(tmpx);
                    y.push_back(tmpy);
                    z.push_back(tmpz);
                }
                else{
                    x.push_back(tmpx);
                    y.push_back(tmpy);
                    z.push_back(tmpz);
                }
            }
            boundary_index.push_back(x.size()-1);
        }
        CubicSpline2D raw_csp = CubicSpline2D(x,y, z);
        CubicSpline2D csp = smooth(raw_csp, boundary_index, id1, id2, id3);
        vector<string> _id({id1, id2, id3});
        csp.set_id(_id);
        vector<double> boundary;
        for(int i : boundary_index){
            boundary.push_back(csp.get_s()[i]);
        }
        csp.set_boundary(boundary);
        return csp;
    }
    else{
        string id[3] = {id1, id2, id3};
        string path[3];
        for(int i = 0;i<3;i++){
            stringstream ss;
            ss << ros::package::getPath("global_map") << "/segment/" << id[i] << ".txt";
            path[i] = ss.str();
        }

        vector<int> boundary_index;
        boundary_index.push_back(0);
        vector<double> x;
        vector<double> y;
        vector<double> z;
        const char delimiter =  ' ';
        for(int i =0;i<3;i++){
            string in_line;
            ifstream in(path[i]);
            double tmpx, tmpy, tmpz;
            while(getline(in, in_line)){
                stringstream ss(in_line);
                string token;
                vector<string> tokens;
                while(getline(ss,token,delimiter))tokens.push_back(token);
                if(tokens.size()<2) continue;
                double _x = stod(tokens[0]);
                double _y = stod(tokens[1]);
                double _z = stod(tokens[2]);
                tmpx = _x;
                tmpy = _y;
                tmpz = _z;
                if(!x.empty() && !y.empty() && norm(x.back()-_x, y.back() - _y) < 4.0) continue;
                x.push_back(_x);
                y.push_back(_y);
                z.push_back(_z);
            }
            if(x.back() != tmpx){
                if(norm(x.back()-tmpx, y.back()-tmpy) < 2.0){
                    x.pop_back();
                    y.pop_back();
                    z.pop_back();
                    x.push_back(tmpx);
                    y.push_back(tmpy);
                    z.push_back(tmpz);
                }
                else{
                    x.push_back(tmpx);
                    y.push_back(tmpy);
                    z.push_back(tmpz);
                }
            }
            boundary_index.push_back(x.size()-1);
        }
        CubicSpline2D raw_csp = CubicSpline2D(x,y, z);
        CubicSpline2D csp = smooth(raw_csp, boundary_index, id1, id2, id3);
        vector<string> _id({id1, id2, id3});
        csp.set_id(_id);
        vector<double> boundary;
        for(int i : boundary_index){
            boundary.push_back(csp.get_s()[i]);
        }
        csp.set_boundary(boundary);
        return csp;
    }
}


static void list_dir(const char *path)
{
    ROS_INFO("START LOADING SEGMENT TXT FILES");
    clock_t begin = clock();
    stringstream save_path;
    save_path << ros::package::getPath("global_map") <<"/config/segment_info.txt";
    cout<<save_path.str();
    ofstream out(save_path.str());
    struct dirent *entry;
    DIR *dir = opendir(path);
    if (dir == NULL) {
        return;
    }
    vector<string> v;
    int cnt = 0;
    while ((entry = readdir(dir)) != NULL) {
        cnt++;
        string s(entry->d_name);
        //int len = strlen(entry->d_name);
        int len = s.length();
        if(len<4) continue;
        v.push_back(s.substr(0,len-4));
        // printf("%d %s\n",cnt, entry->d_name);
    }
    sort(v.begin(), v.end());    
    printf("start build graph\n");

    /* build graph */
    build_graph(v);
    printf("completed to build graph\n");

    out << to_string(idcnt)<<endl;

    /* build segment cubic spline */
    vector<CubicSpline2D> cubics;

    for(int i = 0;i<v.size();i++) {
        string id_cur = v[i];
        string id_prev = (G[i][5] == -1) ? "#" : v[G[i][5]];
        for(int j = 0; j < 3; j++){
            if(G[i][j] == -1) continue;
            int index = mm[{i,j}];
            string id_next = v[G[i][j]];
            CubicSpline2D cs = get_spline(id_prev, id_cur, id_next);
            cs.set_index(index);
            cubics.push_back(cs);
        }
    }
    
    for(CubicSpline2D & cs : cubics) {
        int index = cs.get_index();
        int size = cs.get_size();
        vector<double> x = cs.get_x();
        vector<double> y = cs.get_y();
        vector<double> z = cs.get_z();
        vector<double> s = cs.get_s();
        vector<vector<double>> cx = cs.get_sx().get_coefficient();
        vector<vector<double>> cy = cs.get_sy().get_coefficient();
        vector<vector<double>> cz = cs.get_sz().get_coefficient();
        vector<double> boundary = cs.get_boundary();
        vector<string> id = cs.get_id();
        vector<int> edge = GG[index];

        // calculate dist to intersection
        double dist = boundary[2];
        int cur = index;
        while(true){
            if(GG[cur][0] != -1) cur = GG[cur][0];
            else if(GG[cur][1] != -1) cur = GG[cur][1];
            else if(GG[cur][2] != -1) cur = GG[cur][2];
            else break;           

            if(GG[cur][9]==1) break;
            dist += cubics[cur].get_boundary()[2] - cubics[cur].get_boundary()[1];
        }
        cs.set_dist(dist);

        out << to_string(index) <<" "<< to_string(size) << " " << id[0] << " " << id[1] << " " <<id[2] << endl;

        for(int e : edge) out << to_string(e) << " ";
        out << endl;

        out<<to_string(dist)<<endl;

        for(double bs : boundary) out << to_string(bs) << " ";
        out << endl;

        for(int i=0;i<size;i++){
            out <<to_string(x[i]) <<" "<< to_string(y[i]) <<" "<< to_string(z[i]) <<" "<<to_string(s[i]) <<endl;
        }
        for(int i=0;i<size-1;i++){
            for(int j=0;j<4;j++) out << to_string(cx[j][i]) << " ";
            for(int j=0;j<4;j++) out << to_string(cy[j][i]) << " ";
            for(int j=0;j<4;j++) out << to_string(cz[j][i]) << " ";
            out << endl;
        }
    }
    printf("completed to build spline txt file\n");


    /* build segment map */
    
    printf("check idcnt == # of cubics\n");
    assert(idcnt == cubics.size());
    printf("check completed\n");
    
    double threshold = 2.0;

    stringstream segment_map_path;
    stringstream height_map_path;
    segment_map_path << ros::package::getPath("global_map") <<"/config/segment_map.txt";
    height_map_path << ros::package::getPath("global_map") << "/config/height_map.txt";
    ofstream segment_map_out(segment_map_path.str());
    ofstream height_map_out(height_map_path.str());

    int width = 24300;
    int height = 21400;
    double resolution = 0.1;
    double step = 0.1;
    double rstep = 0.05;
    vector<vector<vector<int>>> possible_map;
    for(int i=0;i<width;i++){
        vector<vector<int>> v;
        for(int j=0;j<height;j++){
            vector<int> u;
            v.push_back(u);
        }
        possible_map.push_back(v);
    }
    printf("prebuild possible map\n");

    
    for(int index = 0; index < cubics.size(); index++){
        if(index%100 == 0) printf("possible map segment index %d\n", index);
        CubicSpline2D &cubic = cubics[index];
        vector<string> _id = cubic.get_id();
        // ROS_INFO("cubic name : %s %s %s",_id[0].c_str(), _id[1].c_str(), _id[2].c_str());
        double s = 0;
        PoseState ps;
        pii pixel;
        double theta;
        while(s<cubic.get_length()){
            ps = cubic.sl_to_xy(SLState(s,0));
            pixel = XY_To_Pixel(ps.x, ps.y);
            theta = cubic.calc_yaw(s);
            for(int px = -3; px<=3;px++){
                for(int py = -60;py<=60;py++){
                    double x = ps.x + px*rstep*cos(theta) - py*rstep*sin(theta);
                    double y = ps.y + px*rstep*sin(theta) + py*rstep*cos(theta);
                    pii cur = XY_To_Pixel(x,y);
                    //cout << x <<" " << y << " "<<theta <<" " <<ps.x << " "<<ps.y<< " "<< cur.first << " " << cur.second << endl;
                    if(possible_map[cur.first][cur.second].size()>0 && possible_map[cur.first][cur.second].back()==index) continue;
                    possible_map[cur.first][cur.second].push_back(index);
                }
            }
            s += step;
        }
    }



    for(int i = 0;i<width;i++){
        for(int j=0;j<height;j++){
            if(possible_map[i][j].size()==0) continue;
            segment_map_out << to_string(i) << " "<<to_string(j);
            for(int v : possible_map[i][j]) segment_map_out << " " << to_string(v);
            segment_map_out << endl; 
        }
    }

    for(int i=0;i<width;i++){
        for(int j=0;j<height;j++){
            if(possible_map[i][j].size()==0) continue;
            pdd xy = Pixel_To_XY(i,j);
            int index = possible_map[i][j][0];
            height_map_out << to_string(i) << " " << to_string(j);
            double h = cubics[index].calc_z(cubics[index].find_s(xy.first,xy.second,0,false));
            height_map_out << " " << to_string(h) << endl;
        }
    }

    segment_map_out.close();
    height_map_out.close();

    printf("completed to build possible map\n");
    printf("completed to build height map\n");

    cout << "completed to save" << endl;
    clock_t end = clock();
    cout << "elapsed time : "<<double(end-begin)/CLOCKS_PER_SEC<<endl;
    closedir(dir);
}

using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "build_cubic_spline");
    stringstream ss;
    ss << ros::package::getPath("global_map") << "/segment";
    list_dir(ss.str().c_str());
}