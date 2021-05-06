// #include "ros/ros.h"
// #include "ros/package.h"

// #include <iostream>
// #include <fstream>
// #include <iomanip>
// #include <stdio.h>
// #include <dirent.h>
// #include <sys/types.h>
// #include <algorithm>
// #include <vector>
// #include <string>

// #include "cubic_spline_2d.h"

// using namespace std;
// #define MAX_SEGMENT 300

// typedef pair<int, int> pii;
// typedef pair<double, double> pdd;


// int G[MAX_SEGMENT][9];

// pii XY_To_Pixel(double x, double y){
//     double a1 = 10;
//     double b1 = 0;
//     double c1 = 13500;
//     double a2 = 0;
//     double b2 = 10;
//     double c2 = 15000;
   
//     int px = int(a1*x+b1*y+c1);
//     int py = int(a2*x+b2*y+c2);
//     return {px, py};
// }

// pdd Pixel_To_XY(int px, int py){
//     double a1 = 0.1;
//     double b1 = 0.0;
//     double c1 = -1350.0;
//     double a2 = 0.0;
//     double b2 = 0.1;
//     double c2 = -1500.0;
   
//     double x = a1*px+b1*py+c1;
//     double y = a2*px+b2*py+c2;
//     return {x, y};
// }


// string format_string(vector<string>&names, int i){
//     if(i==-1) return "\t\t\t";
//     if(names[i].size()==7){
//         string ret = names[i];
//         ret += '\t';
//         ret += '\t';
//         return ret;
//     }
//     else{
//         string ret = names[i];
//         return ret;
//     }
// }

// int match_string(vector<string> &names, string expr){
//     for(int i = 0;i<names.size();i++){
//         string str = names[i];
//         if(expr.size() != str.size()) continue;

//         bool match = true;
//         for(int j = 0; j<expr.size();j++){
//             if(expr[j] == '#') continue;
//             if(expr[j] != str[j]) match = false;
//         }
//         if(match) return i;
//     }
//     return -1;
// }

// void build_graph(vector<string>& names){
//     printf("names size %d", (int)names.size());
//     for(int i =0;i<names.size();i++){
//         for(int k = 0;k<5;k++) G[i][k] = -1;
//         string str = names[i];
//         if(str.length() == 7){
//             string expr;

//             // next_straight
//             expr = "############";
//             expr[0] = str[0];
//             expr[1] = str[1];
//             expr[2] = str[2];
//             expr[3] = str[3];
//             expr[6] = '_';
//             expr[7] = str[5];
//             expr[8] = str[6];
//             expr[9] = 'S';
//             G[i][0] = match_string(names, expr);
//             if(G[i][0] == -1){
//                 char c = str[5];
//                 expr = str;
//                 expr[5] = c+1;
//                 G[i][0] = match_string(names, expr);
//                 if(G[i][0]==-1){
//                     expr[6] = str[6]+1;
//                     G[i][0] = match_string(names, expr);
//                     if(G[i][0]==-1){
//                         expr[6] = str[6]-1;
//                         G[i][0] = match_string(names, expr);
//                     }
//                 }
//             }

//             // next_left
//             expr = "############";
//             expr[0] = str[0];
//             expr[1] = str[1];
//             expr[2] = str[2];
//             expr[3] = str[3];
//             expr[6] = '_';
//             expr[7] = str[5];
//             expr[8] = str[6];
//             expr[9] = 'L';
//             G[i][1] = match_string(names, expr);
            

//             // next_right
//             expr = "############";
//             expr[0] = str[0];
//             expr[1] = str[1];
//             expr[2] = str[2];
//             expr[3] = str[3];
//             expr[6] = '_';
//             expr[7] = str[5];
//             expr[8] = str[6];
//             expr[9] = 'R';
//             G[i][2] = match_string(names, expr);
            

//             // left
//             expr = str;
//             if(str[6]=='0') G[i][3] = -1;
//             else{
//                 expr[6] = str[6]-1;
//                 G[i][3] = match_string(names, expr);
//             }

//             // right
//             expr = str;
//             if(str[6]=='9') G[i][4] = -1;
//             else{
//                 expr[6] = str[6]+1;
//                 G[i][4] = match_string(names, expr);
//             }

//             // intersection flag
//             G[i][5] = 0;

//             // sector number, lane number
//             G[i][6] = str[5]-'0';
//             G[i][7] = str[6]-'0';
//             G[i][8] = str[6]-'0';
//         }
//         else if(str.length() == 12){
//             string expr;

//             // next_straight
//             expr = "#######";
//             expr[0] = str[2];
//             expr[1] = str[3];
//             expr[2] = str[4];
//             expr[3] = str[5];
//             expr[4] = '_';
//             expr[5] = str[10];
//             expr[6] = str[11];
//             G[i][0] = match_string(names, expr);

//             // next_left
//             G[i][1] = -1;

//             // next_right
//             G[i][2] = -1;

//             // left
//             expr = str;
//             if(str[8]=='0' || str[11]=='0') G[i][3] = -1;
//             else{
//                 expr[8] = str[8]-1;
//                 expr[11] = str[11]-1;
//                 G[i][3] = match_string(names, expr);
//             }

//             // right
//             expr = str;
//             if(str[8]=='9' || str[11]=='9') G[i][4] = -1;
//             else{
//                 expr[8] = str[8]+1;
//                 expr[11] = str[11]+1;
//                 G[i][4] = match_string(names, expr);
//             }

//             // intersection flag
//             G[i][5] = 1;

//             G[i][6] = 0;

//             // lane number from
//             G[i][7] = str[8]-'0';

//             // lane number to
//             G[i][8] = str[11]-'0';
//         }
//         else{
//             printf("!!!!!!!!!!!!!! name which length is not 7 and 12 !!!!!!!!!!!!!!!!!");
//             assert(false);
//         }
//     }
    
//     stringstream save_path;
//     save_path << ros::package::getPath("prediction_tracker") <<"/config/graph_visualize.txt";
//     FILE* fpw = fopen(save_path.str().c_str(), "w");
//     for(int i = 0;i<names.size();i++){
//         fprintf(fpw,"\n");
//         fprintf(fpw,"%s\t%s\t%s\n", format_string(names, G[i][1]).c_str(), format_string(names, G[i][0]).c_str(), format_string(names, G[i][2]).c_str());
//         fprintf(fpw,"\t\t\t\t  ^\n");
//         fprintf(fpw,"\t\t\t\t  |\n");
//         fprintf(fpw,"%s<-\t%s->\t%s\n", format_string(names, G[i][3]).c_str(), format_string(names, i).c_str(), format_string(names, G[i][4]).c_str());
//         fprintf(fpw,"\n");
//         fprintf(fpw,"-----------------------------------------------");
//         fprintf(fpw,"\n");
//     }
//     fclose(fpw);
// }

// CubicSpline2D get_spline(string path){
//     vector<double> x;
//     vector<double> y;
//     const char delimiter =  ' ';
//     string in_line;
//     ifstream in(path);
//     while(getline(in, in_line)){
//         stringstream ss(in_line);
//         string token;
//         vector<string> tokens;
//         while(getline(ss,token,delimiter))tokens.push_back(token);
//         if(tokens.size()<2) continue;
//         x.push_back(stod(tokens[0]));
//         y.push_back(stod(tokens[1]));
//     }

//     return CubicSpline2D(x,y);
// }


// static void list_dir(const char *path)
// {
//     clock_t begin = clock();
//     stringstream save_path;
//     save_path << ros::package::getPath("prediction_tracker") <<"/config/segment_info.txt";
//     cout<<save_path.str();
//     ofstream out(save_path.str());
//     struct dirent *entry;
//     DIR *dir = opendir(path);
//     if (dir == NULL) {
//         return;
//     }
//     vector<string> v;
//     int cnt = 0;
//     while ((entry = readdir(dir)) != NULL) {
//         cnt++;
//         string s(entry->d_name);
//         //int len = strlen(entry->d_name);
//         int len = s.length();
//         if(len<4) continue;
//         v.push_back(s.substr(0,len-4));
//         printf("%d %s\n",cnt, entry->d_name);
//     }
//     printf("start");
//     sort(v.begin(), v.end());
//     printf("check1");
//     out << to_string(v.size())<<endl;
//     printf("start build graph");

//     /* build graph */
//     build_graph(v);
//     printf("completed to build graph\n");

//     /* build segment cubic spline */
//     vector<CubicSpline2D> cubics;
//     int index = 0;
//     for(string id : v) {
//         stringstream ss;
//         ss << ros::package::getPath("prediction_tracker") << "/segment/" << id << ".txt";
//         CubicSpline2D cs = get_spline(ss.str());
//         cs.setName(id);
//         int size = cs.get_size();
//         vector<double> x = cs.get_x();
//         vector<double> y = cs.get_y();
//         vector<double> s = cs.get_s();
//         vector<vector<double>> cx = cs.get_sx().get_coefficient();
//         vector<vector<double>> cy = cs.get_sy().get_coefficient();

//         cubics.push_back(cs);
        
//         out << to_string(index) <<" "<< to_string(size) << " " << id << endl;

//         for(int i=0;i<9;i++) out << to_string(G[index][i]) << " ";
//         out << endl;

//         for(int i=0;i<size;i++){
//             out <<to_string(x[i]) <<" "<< to_string(y[i]) <<" "<<to_string(s[i]) <<endl;
//         }
//         for(int i=0;i<size-1;i++){
//             for(int j=0;j<4;j++) out << to_string(cx[j][i]) << " ";
//             for(int j=0;j<4;j++) out << to_string(cy[j][i]) << " ";
//             out << endl;
//         }
//         index ++;
//         printf("completed to build spline %s\n", id.c_str());
//     }
//     printf("completed to build spline txt file\n");
//     /* build segment map, segment nearest */
    
    
//     double threshold = 2.0;

//     stringstream segment_map_path;
//     segment_map_path << ros::package::getPath("prediction_tracker") <<"/config/segment_map.txt";
//     ofstream segment_map_out(segment_map_path.str());

//     stringstream segment_nearest_path;
//     segment_nearest_path << ros::package::getPath("prediction_tracker") <<"/config/segment_nearest.txt";
//     ofstream segment_nearest_out(segment_nearest_path.str());

//     int width = 24300;
//     int height = 21400;
//     double resolution = 0.1;
//     double step = 0.1;
//     double rstep = 0.05;
//     vector<vector<vector<int>>> possible_map;
//     for(int i=0;i<width;i++){
//         vector<vector<int>> v;
//         for(int j=0;j<height;j++){
//             vector<int> u;
//             v.push_back(u);
//         }
//         possible_map.push_back(v);
//     }
//     ROS_INFO("prebuild possible map");

//     index = 0;
//     for(CubicSpline2D &cubic : cubics){
//         ROS_INFO("cubic name : %s",cubic.get_id().c_str());
//         double s = 0;
//         PoseState ps;
//         pii pixel;
//         double theta;
//         while(s<cubic.get_length()){
//             ps = cubic.sl_to_xy(SLState(s,0));
//             pixel = XY_To_Pixel(ps.x, ps.y);
//             theta = cubic.calc_yaw(s);
//             for(int px = -3; px<=3;px++){
//                 for(int py = -60;py<=60;py++){
//                     double x = ps.x + px*rstep*cos(theta) - py*rstep*sin(theta);
//                     double y = ps.y + px*rstep*sin(theta) + py*rstep*cos(theta);
//                     pii cur = XY_To_Pixel(x,y);
//                     //cout << x <<" " << y << " "<<theta <<" " <<ps.x << " "<<ps.y<< " "<< cur.first << " " << cur.second << endl;
//                     if(possible_map[cur.first][cur.second].size()>0 && possible_map[cur.first][cur.second].back()==index) continue;
//                     possible_map[cur.first][cur.second].push_back(index);
//                 }
//             }
//             s += step;
//         }
//         index ++;
//     }

//     cout << "completed to build possible map" << endl;

//     for(int i = 0;i<width;i++){
//         for(int j=0;j<height;j++){
//             if(possible_map[i][j].size()==0) continue;
//             segment_map_out << to_string(i) << " "<<to_string(j);
//             for(int v : possible_map[i][j]) segment_map_out << " " << to_string(v);
//             segment_map_out << endl; 
//         }
//     }

//     segment_map_out.close();

//     for(int i=0;i<width;i++){
//         for(int j=0;j<height;j++){
//             if(possible_map[i][j].size()==0) continue;
//             int min_index = -1;
//             double min_distance = INF;
//             if(possible_map[i][j].size()>1){
//                 for(int cur : possible_map[i][j]){
//                     pdd xy = Pixel_To_XY(i,j);
//                     PoseState ps;
//                     ps.x = xy.first;
//                     ps.y = xy.second;
//                     SLState sl = cubics[cur].transform(ps,false);
//                     if(sl.l < min_distance){
//                         min_index = cur;
//                         min_distance = sl.l;
//                     }
//                 }
//             }
//             else min_index = possible_map[i][j][0];
//             if(min_index == -1) continue;
//             segment_nearest_out <<to_string(i) <<" "<<to_string(j)<<" "<<to_string(min_index)<<endl;
//         }
//     }

//     segment_nearest_out.close();
    

//     /*
//     for(int x = 0; x<2430;x++){
//         for(int y = 0; y<2140; y++){
//             int nearest_segment = -1; double key = 1e10;
//             PoseState ps;
//             ps.x = x;
//             ps.y = y;
//             vector<int> cur;
//             //clock_t begin = clock();
//             for(int i = 0;i<cubics.size();i++){
//                 SLState sl = cubics[i].transform(ps,false);
//                 if(sl.l < threshold) {
//                     cur.push_back(i);
//                     if(sl.l < key){
//                         key=sl.l;
//                         nearest_segment=i;
//                     }
//                 }
//             }
//             //clock_t end = clock();
//             //printf("elapsed time : %lf\n",double(end-begin)/CLOCKS_PER_SEC);
//             if(cur.size()>0){
//                 segment_map_out<<to_string(x)<<" "<<to_string(y);
//                 for(int i : cur) segment_map_out<<" " << i;
//             }
//             segment_map_out<<endl;
//             if(nearest_segment >=0){
//                 segment_nearest_out<<to_string(x)<<" "<<to_string(y);
//                 segment_nearest_out << " "<<to_string(nearest_segment) <<endl;
//             }   
//         }
//         if(x%10==0)printf("x :%d completed\n",x);
//     }
//     printf("completed to build nearest segment txt file\n");
//     segment_map_out.close();
//     segment_nearest_out.close();
//     out.close();*/
//     cout << "completed to save" << endl;
//     clock_t end = clock();
//     cout << "elapsed time : "<<double(end-begin)/CLOCKS_PER_SEC<<endl;
//     closedir(dir);
// }

// using namespace std;

// int main(int argc, char **argv){
//     ros::init(argc, argv, "build_segment_info");
//     stringstream ss;
//     ss << ros::package::getPath("prediction_tracker") << "/segment";
//     list_dir(ss.str().c_str());
// }
