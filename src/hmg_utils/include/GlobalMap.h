#ifndef GLOBALMAP
#define GLOBALMAP

#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <vector>
#include <unordered_map>

#include "hmg_utils/BoxArray.h"
#include "hmg_utils/Prediction.h"
#include "hmg_utils/Pedestrian.h"

#include "cubic_spline_2d.h"
#include "utils.h"

using namespace std;

typedef pair<int, int> pii;
typedef pair<double, double> pdd;


const long long HASH = 30000;

class GlobalMap{
    public:
    int width;
    int height;
    //vector<PNode> global_segment_map[GW+11][GH+11];
    unordered_map<long long, vector<int>> global_segment_map;
    unordered_map<long long, double> global_height_map;
    unordered_map<long long, bool> global_intersection_map;
    //vector<PNode> global_segment_map;
    vector<CubicSpline2D> segment;
    double a1,b1,c1,a2,b2,c2; //px = a1x+b1y+c1, py = a2x+b2y+c2

    GlobalMap();
    GlobalMap(double a1, double b1, double c1, double a2, double b2, double c2, int w, int h);
    void loadSegmentMap(string segment_map_path);
    void loadHeightMap(string height_map_path);
    void loadIntersectionMap(string intersection_map_path);
    void loadSegmentInfo(string segment_info_path);
    pii XYToPixel(double x, double y);
    pdd PixelToXY(int px, int py);
    vector<int> getSegmentArray(double x, double y);
    double getHeight(double x, double y);
    bool isIntersection(double x, double y);
    vector<hmg_utils::BoxArray> PredictionService(const vector<hmg_utils::Prediction> &predictions, const vector<double> &times);
    vector<hmg_utils::BoxArray> PedestrianService(const vector<hmg_utils::Pedestrian> &predictions, const vector<double> &times);
};

#endif