#ifndef DECISION_MAKER_TOOLS
#define DECISION_MAKER_TOOLS

#include <vector>
#include "ros/ros.h"
#include "ros/package.h"
#include <hmg_utils/Target.h>
#include <hmg_utils/Targets.h>
#include "GlobalMap.h"

using namespace std;

#define MAX_SEGMENT 100
#define bool_to_string(x) x?"True":"False"

const double pi = 3.14159265358979f;

enum State{
    URGENT,
    BUSY,
    NORMAL,
};

enum taskState{
	DRIVING_SECTION,
	INTERSECTION_STRAIGHT,
	INTERSECTION_LEFT,
	INTERSECTION_RIGHT,
};

enum motionState{
    DRIVE,
    LANE_CHANGE,
    CAUTION,
    STOP,
};

inline int abs(int x){return (x>0)?(x):(-x);}
inline bool isGreen(int light){return (light & 1) == 1;}
inline bool isLeft(int light){return ((light >> 1) & 1) == 1;}
inline bool isYellow(int light){return ((light >> 2) & 1) == 1;}
inline bool isRed(int light){return ((light >> 3) & 1) == 1;}

hmg_utils::Target _Target(int flag, int segment, double l_offset, double s, double velocity, string target_front, string target_back, int priority);

struct Vehicle{
    string id;
    double s;
    double speed;
    Vehicle() {}
    Vehicle(string _id, double _s, double _speed) : id(_id), s(_s), speed(_speed) {}
    bool operator < (const Vehicle & o){
        return s<o.s;
    }
};

struct SegmentCmp{
    int segment;
    double l;
    int s_range;
    SegmentCmp() {}
    SegmentCmp(int a, double b, int c) : segment(a), l(b), s_range(c) {}

    bool operator < (const SegmentCmp & o){
        if(abs(abs(l) - abs(o.l)) < 0.3){
            int priority[3] = {2,0,1};
            if(priority[s_range] != priority[o.s_range]) return priority[s_range] < priority[o.s_range];
            else return abs(l) < abs(o.l);
        }
        else return abs(l) < abs(o.l);
    }
};

struct Mission{
    int task;
    int last_sector;
    vector<int> segments;
    vector<int> desired_lanes;
    Mission() {}
    Mission(int _task, int _last_sector, vector<int> _segments, vector<int> _desired_lanes) : task(_task), last_sector(_last_sector), segments(_segments), desired_lanes(_desired_lanes) {}
};

struct MissionRecognizer{
    vector<Mission> mission_sequence;
    int mission_size;
    int ind;
    int init_segment;

    MissionRecognizer(){
        mission_sequence.clear();
        mission_size =0;
        ind = 0;
        init_segment = -1;
    }
    
    void update(int current_segment);
    int current_task();
    int next_task();
    int next_next_task();
    int dist_to_desired_lanes(int current_lane);
    int dir_to_desired_lanes(int current_lane);
    bool in_desired_lanes(int current_lane);
    bool is_current_last_sector(int current_sector);
    bool is_next_last_sector(int next_sector);
    vector<int> near_segments(vector<int> & edge, vector<vector<int>> & G);
    void check_mission(vector<string> & names);
    void loadMissionInfo(string mission_info_path, vector<string> & names);
    void loadWaypoint(GlobalMap & global_map, vector<CubicSpline2D> & cubics, vector<string> & names, string waypoint_path, bool debug = true);
};

struct LaneChangeHandler{
    int current_mission_id;
    int current_segment;
    double current_s;
    int current_lane;
    int current_lane_enter_segment;
    double current_lane_enter_s;
    double current_lane_enter_segment_length;
    int current_lane_next_segment;

    LaneChangeHandler(){
        current_mission_id = -1;
        current_segment = -1;
        current_s = 0;
        current_lane = -1;
        current_lane_enter_segment = -1;
        current_lane_enter_s = 0;
        current_lane_enter_segment_length = 0;
        current_lane_next_segment = -1;
    }

    void update(int mid, double s, CubicSpline2D & cur);
    void update(int mid, double s, CubicSpline2D & cur, CubicSpline2D & next);
    double get_dist();
};

struct LightHandler{
    int prev_light;
    bool go_signal;
    double time_light_changed;
    double duration_light_changed;
    LightHandler() {
        prev_light = 0;
        go_signal = true;
        time_light_changed = 0;
        duration_light_changed;
    }
    void update(double t, int light, int current_task, int next_task, bool can_stop);
};

struct Lane{
    double cost = 0;
    double s;
    double d_left;
    bool jam = false;
    bool obstacle_exist = false;
    double obstacle_dist = INF;
    double speed;
    double ttrd; // time to reach desired lanes
    bool very_right = false;
    bool very_left = false;
    int segment;
    int lane;
    int l_dist;
    int dir;
    vector<Vehicle> vehicles;
};

vector<int> match_string(vector<string> &names, string expr);
bool match_expr(string name, string expr);
void print_lanes(vector<string> &names, vector<Lane> & lanes);
int char_to_int(char c);

#endif
