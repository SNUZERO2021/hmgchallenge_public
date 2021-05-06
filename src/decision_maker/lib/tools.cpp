#include <vector>
#include "GlobalMap.h"
#include "tools.h"
#include "ros/ros.h"
#include "proj.h"

using namespace std;

hmg_utils::Target _Target(int flag, int segment, double l_offset, double s, double velocity, string target_front, string target_back, int priority){
    hmg_utils::Target tg;
    tg.flag = flag;
    tg.segment = segment;
    tg.l_offset = l_offset;
    tg.s = s;
    tg.velocity = velocity;
    tg.target_front = target_front;
    tg.target_back = target_back;
    tg.priority = priority;
    return tg;
}

void MissionRecognizer::update(int current_segment){
    if(ind >= mission_size-1) return;
    bool next_mission = false;
    for(int next_mission_segment : mission_sequence[ind+1].segments){
        if(next_mission_segment == current_segment){
            next_mission = true;
            break;
        }
    }
    if(next_mission) ++ind;
}

int MissionRecognizer::current_task(){
    return mission_sequence[ind].task;    
}

int MissionRecognizer::next_task(){
    if(ind >= mission_size-1) return DRIVING_SECTION;
    else return mission_sequence[ind+1].task;    
}

int MissionRecognizer::next_next_task(){
    if(ind >= mission_size-2) return DRIVING_SECTION;
    else return mission_sequence[ind+2].task;    
}



int MissionRecognizer::dist_to_desired_lanes(int current_lane){
    if(ind >= mission_size) return 0;
    int ret = 100;
    for(int l : mission_sequence[ind].desired_lanes){
        ret = min(ret, abs(current_lane - l));
    }
    return ret;
}

int MissionRecognizer::dir_to_desired_lanes(int current_lane){
    if(ind >= mission_size) return 0;
    if(ind >= mission_size) return 0;
    int val = 100;
    int key = -1;
    for(int l : mission_sequence[ind].desired_lanes){
        if(val > abs(current_lane - l)){
            val = abs(current_lane - l);
            key = l;
        }
    }
    if(key > current_lane) return 1;
    else if(key == current_lane) return 0;
    else return -1;
}

bool MissionRecognizer::in_desired_lanes(int current_lane){
    if(ind >= mission_size) return true;
    for(int l : mission_sequence[ind].desired_lanes){
        if(l == current_lane) return true;
    }
    return false;
}

bool MissionRecognizer::is_current_last_sector(int current_sector){
    if(ind < 0 || ind >= mission_size) return false;
    return (mission_sequence[ind].last_sector == current_sector);
}

bool MissionRecognizer::is_next_last_sector(int next_sector){
    if(ind+1 < 0 || ind+1 >= mission_size) return false;
    return (mission_sequence[ind+1].last_sector == next_sector);
}

vector<int> MissionRecognizer::near_segments(vector<int> & edge, vector<vector<int>> & G){
    vector<int> ret;
    int _next_task = next_task();
    int _next_next_task = next_next_task();

    // next segment
    if(edge[CURRENT_IS_INTER] == 0 && edge[NEXT_IS_INTER] == 0){
        if(is_current_last_sector(edge[NEXT_SECTOR_NUM])){
            if(_next_task==INTERSECTION_LEFT){
                if(edge[NEXT_LEFT] != -1) ret.push_back(edge[NEXT_LEFT]);
                else{
                    int tmp_left = edge[LEFT_STRAIGHT];
                    if(tmp_left != -1 && G[tmp_left][NEXT_LEFT] != -1) ret.push_back(G[tmp_left][NEXT_LEFT]);
                    else ret.push_back(-1);
                }
            } 
            else if(_next_task==INTERSECTION_RIGHT){
                if(edge[NEXT_RIGHT] != -1) ret.push_back(edge[NEXT_RIGHT]);
                else{
                    int tmp_right = edge[RIGHT_STRAIGHT];
                    if(tmp_right != -1 && G[tmp_right][NEXT_RIGHT] != -1) ret.push_back(G[tmp_right][NEXT_RIGHT]);
                    else ret.push_back(-1);
                }
            }
            else {
                if(edge[NEXT_STRAIGHT] != -1) ret.push_back(edge[NEXT_STRAIGHT]);
                else{
                    if(edge[LEFT_STRAIGHT] != -1 && G[edge[LEFT_STRAIGHT]][NEXT_STRAIGHT] != -1) ret.push_back(G[edge[LEFT_STRAIGHT]][NEXT_STRAIGHT]);
                    else if(edge[RIGHT_STRAIGHT] != -1 && G[edge[RIGHT_STRAIGHT]][NEXT_STRAIGHT] != -1) ret.push_back(G[edge[RIGHT_STRAIGHT]][NEXT_STRAIGHT]);
                    else ret.push_back(edge[NEXT_STRAIGHT]);
                }
            }
        }
        else{
            ret.push_back(edge[NEXT_STRAIGHT]);
        }
    }
    else if(edge[CURRENT_IS_INTER] == 1 && edge[NEXT_IS_INTER] == 0){
        if(is_next_last_sector(edge[NEXT_SECTOR_NUM])){
            if(_next_next_task==INTERSECTION_LEFT){
                if(edge[NEXT_LEFT] != -1) ret.push_back(edge[NEXT_LEFT]);
                else{
                    int tmp_left = edge[LEFT_STRAIGHT];
                    if(tmp_left != -1 && G[tmp_left][NEXT_LEFT] != -1) ret.push_back(G[tmp_left][NEXT_LEFT]);
                    else ret.push_back(-1);
                }
            } 
            else if(_next_next_task==INTERSECTION_RIGHT){
                if(edge[NEXT_RIGHT] != -1) ret.push_back(edge[NEXT_RIGHT]);
                else{
                    int tmp_right = edge[RIGHT_STRAIGHT];
                    if(tmp_right != -1 && G[tmp_right][NEXT_RIGHT] != -1) ret.push_back(G[tmp_right][NEXT_RIGHT]);
                    else ret.push_back(-1);
                }
            }
            else {
                if(edge[NEXT_STRAIGHT] != -1) ret.push_back(edge[NEXT_STRAIGHT]);
                else{
                    if(edge[LEFT_STRAIGHT] != -1 && G[edge[LEFT_STRAIGHT]][NEXT_STRAIGHT] != -1) ret.push_back(G[edge[LEFT_STRAIGHT]][NEXT_STRAIGHT]);
                    else if(edge[RIGHT_STRAIGHT] != -1 && G[edge[RIGHT_STRAIGHT]][NEXT_STRAIGHT] != -1) ret.push_back(G[edge[RIGHT_STRAIGHT]][NEXT_STRAIGHT]);
                    else ret.push_back(edge[NEXT_STRAIGHT]);
                }
            }
        }
        else{
            ret.push_back(edge[NEXT_STRAIGHT]);
        }
    }
    else if(edge[CURRENT_IS_INTER] == 0 && edge[NEXT_IS_INTER] == 1){
        ret.push_back(edge[NEXT_STRAIGHT]);
    }
    else ret.push_back(edge[NEXT_STRAIGHT]);

    if(ret.back() == -1){
        ret.pop_back();
        if(edge[NEXT_STRAIGHT] != -1) ret.push_back(edge[NEXT_STRAIGHT]);
        else if(edge[NEXT_LEFT] != -1) ret.push_back(edge[NEXT_LEFT]);
        else ret.push_back(edge[NEXT_RIGHT]);
    }

    // left segment
    if(edge[CURRENT_IS_INTER] == 0 && is_current_last_sector(edge[CURRENT_SECTOR_NUM])){
        if(_next_task==INTERSECTION_LEFT && edge[LEFT_LEFT] != -1) ret.push_back(edge[LEFT_LEFT]);
        else if(_next_task==INTERSECTION_RIGHT && edge[LEFT_RIGHT] != -1) ret.push_back(edge[LEFT_RIGHT]);
        else ret.push_back(edge[LEFT_STRAIGHT]);
    }
    else ret.push_back(edge[LEFT_STRAIGHT]);
    if(ret.back() == -1){
        ret.pop_back();
        if(edge[LEFT_STRAIGHT] != -1) ret.push_back(edge[LEFT_STRAIGHT]);
        else if(edge[LEFT_LEFT] != -1) ret.push_back(edge[LEFT_LEFT]);
        else ret.push_back(edge[LEFT_RIGHT]);
    }

    // right segment
    if(edge[CURRENT_IS_INTER] == 0 && is_current_last_sector(edge[CURRENT_SECTOR_NUM])){
        if(_next_task==INTERSECTION_LEFT && edge[RIGHT_LEFT] != -1) ret.push_back(edge[RIGHT_LEFT]);
        else if(_next_task==INTERSECTION_RIGHT && edge[RIGHT_RIGHT] != -1) ret.push_back(edge[RIGHT_RIGHT]);
        else ret.push_back(edge[RIGHT_STRAIGHT]);
    }
    else ret.push_back(edge[RIGHT_STRAIGHT]);
    if(ret.back() == -1){
        ret.pop_back();
        if(edge[RIGHT_STRAIGHT] != -1) ret.push_back(edge[RIGHT_STRAIGHT]);
        else if(edge[RIGHT_LEFT] != -1) ret.push_back(edge[RIGHT_LEFT]);
        else ret.push_back(edge[RIGHT_RIGHT]);
    }

    return ret;
}

void MissionRecognizer::check_mission(vector<string> & names){
    string taskstr[4] = {"DRIVING_SECTION\t\t\t", "INTERSECTION_STRAIGHT\t", "INTERSECTION_LEFT\t\t", "INTERSECTION_RIGHT\t\t"};
    char taskChar[4] = {'D', 'S', 'L', 'R'};
    printf("[MissionRecognizer] mission size : %d, start ind : %d\n\n", mission_size, ind);
    for(int i =0;i<mission_size;i++){
        Mission& m = mission_sequence[i];
        printf("[Mission %d]\t task : %s\n",i,taskstr[m.task].c_str());
        printf("\t\t last_sector : %d\n", m.last_sector);
        printf("\t\t segments : ");
        if(m.segments.empty()){
            printf("[MissionRecognizer] Error. Current mission's segments empty.\n");
            assert(false);
        }
        for(int seg : m.segments){
            printf("%s ", names[seg].c_str());
        }
        printf("\n\t\t desired_lanes : ");
        if(m.desired_lanes.empty()){
            printf("[MissionRecognizer] Error. Current mission's desired_lanes empty.\n");
            assert(false);
        }
        for(int l : m.desired_lanes){
            printf("%d ", l);
        }
        
        printf("\n\n");
        if(i+1 < mission_size && m.task == 0){
            for(int l : m.desired_lanes){
                string expr = "############";
                expr[0] = names[m.segments.front()][0];
                expr[1] = names[m.segments.front()][1];
                expr[2] = names[m.segments.front()][2];
                expr[3] = names[m.segments.front()][3];
                if(m.last_sector >= 10) expr[7] = m.last_sector-10 + 'A';
                else expr[7] = m.last_sector + '0';
                if(l >= 10) expr[8] = l-10 + 'A';
                else expr[8] = l + '0';
                expr[9] = taskChar[mission_sequence[i+1].task];

                if(match_string(names, expr).empty()){
                    printf("[MissionRecognizer] Error. No segment exist named %s. Please check task/last_sector/desired_lanes.\n", expr.c_str());
                    assert(false);
                }
                else{
                    // printf("[MissionRecognizer] Segment %s exists.\n", expr.c_str());
                }
            }
        }

        if(i!=0){
            if(mission_sequence[i-1].task ==0 && mission_sequence[i].task == 0){
                printf("[MissionRecognizer] Error. Two consecutive mission have DRIVING_SECTION task.\n");
                assert(false);
            }
            else if(mission_sequence[i-1].task !=0 && mission_sequence[i].task != 0){
                printf("[MissionRecognizer] Error. Two consecutive mission have INTERSECTION task.\n");
                assert(false);
            }
        }
        printf("\n");
    }
}

void MissionRecognizer::loadMissionInfo(string mission_info_path, vector<string> & names){
    const char delimiter = ' ';
    string in_line;
    ifstream in(mission_info_path);
    bool first_line = true;
    while(getline(in, in_line)){
        if(first_line){
            first_line = false;
            stringstream ss(in_line);
            string token;
            vector<string> tokens;
            while(getline(ss,token,delimiter)) tokens.push_back(token);
            if(tokens.size() != 1){
                printf("[loadMissionInfo] first line tokens size %d is not 1\n", (int)(tokens.size()));
                assert(false);
            }
            init_segment = stoi(tokens[0]);
            continue;
        }
        stringstream ss(in_line);
        string token;
        vector<string> tokens;
        while(getline(ss,token,delimiter)) tokens.push_back(token);

        if(tokens.size() < 4){
            printf("[loadMissionInfo] tokens size %d is less than 4\n", (int)(tokens.size()));
            assert(false);
        }

        int task;
        if(tokens[0][0] == 'D') task = 0;
        else if(tokens[0][0] == 'S') task = 1;
        else if(tokens[0][0] == 'L') task = 2;
        else if(tokens[0][0] == 'R') task = 3;
        else{
            printf("[loadMissionInfo] tokens[0][0] %c is not one of (D, S, L, R)\n", tokens[0][0]);
            assert(false);
        }
        int last_sector = stoi(tokens[1]);
        string sector_name = tokens[2];
        vector<int> segments;
        if(sector_name.size() == 4){
            string expr = "#######";
            expr[0] = sector_name[0];
            expr[1] = sector_name[1];
            expr[2] = sector_name[2];
            expr[3] = sector_name[3];
            segments = match_string(names, expr);
        }
        else if(sector_name.size() == 6){
            string expr = "############";
            expr[0] = sector_name[0];
            expr[1] = sector_name[1];
            expr[2] = sector_name[2];
            expr[3] = sector_name[3];
            expr[4] = sector_name[4];
            expr[5] = sector_name[5];
            if(task == INTERSECTION_STRAIGHT) expr[9] = 'S';
            else if(task == INTERSECTION_LEFT) expr[9] = 'L';
            else if(task == INTERSECTION_RIGHT) expr[9] = 'R';
            else{
                printf("[loadMissionInfo] intersection segment wrong task : %d\n", task);
                assert(false);
            }
            segments = match_string(names, expr);
        }
        vector<int> desired_lanes;
        for(int i = 3;i<tokens.size();i++) desired_lanes.push_back(stoi(tokens[i]));
        mission_sequence.push_back(Mission(task, last_sector, segments, desired_lanes));
    }
    mission_size = mission_sequence.size();
    ind = -1;
    for(int i = 0;i<mission_sequence.size();i++){
        for(int seg : mission_sequence[i].segments){
            if(seg == init_segment){
                ind = i;
                break;
            }
        }
    }
    if(ind == -1){
        printf("[loadMissionInfo] init_segment : %s, %d --> no corresponding mission\n", names[init_segment].c_str(), init_segment);
        assert(false);
    }
}

void MissionRecognizer::loadWaypoint(GlobalMap & global_map, vector<CubicSpline2D> & cubics, vector<string> & names, string waypoint_path, bool debug){
    vector<pair<double, double>> waypoints;
	const char delimiter =  ',';
    string in_line;
    ifstream in(waypoint_path);

    // read waypoint --> save as global x, y
    while(getline(in, in_line)){
        stringstream ss(in_line);
        string token;
        vector<string> tokens;
        while(getline(ss,token,delimiter))tokens.push_back(token);
        if(tokens.size() != 2){
            printf("[loadWaypoint] tokens size = %d not 2\n", (int)tokens.size());
            assert(false);
        }
        vector<double> xy = latlong_to_xy(stod(tokens[0])*180/pi, stod(tokens[1])*180/pi);
        if(debug) printf("[loadWaypoint] x : %lf, y : %lf\n", xy[0], xy[1]);
        waypoints.push_back({xy[0], xy[1]});
    }

    int sz = waypoints.size();

    printf("[loadWaypoint] %d waypoints loaded.\n\n", sz);

    // get possible segments of each waypoint
    vector<vector<int>> possible_segments;
    for(pair<double ,double> p : waypoints){
        possible_segments.push_back(global_map.getSegmentArray(p.first, p.second));
    }
    // sort possible segments of each waypoint ( 1. smaller abs(l), 2. s_range(middle > next > prev) )
    vector<vector<SegmentCmp>> sorted_segments;
    for(int i = 0;i<sz;i++){
        vector<int> & v = possible_segments[i];
        if(v.size() == 0) {
            for(int asdf = 0;asdf<5;asdf++) printf("[loadWaypoint] waypoint %d 's possible_segments size = 0\n", i);
            // assert(false);
            continue;
        }
        sorted_segments.push_back(vector<SegmentCmp>());
        for(int ind : v){
            PoseState ps;
            ps.x = waypoints[i].first;
            ps.y = waypoints[i].second;

            SLState sl = cubics[ind].transform(ps);
            double l = sl.l;

            vector<double> boundary = cubics[ind].get_boundary();
            int s_range;
            if(sl.s < boundary[1]) s_range =0;
            else if(sl.s < boundary[2]) s_range = 1;
            else s_range = 2;

            if(s_range != 0) sorted_segments.back().push_back(SegmentCmp(ind, l, s_range));
        }
        sort(sorted_segments.back().begin(), sorted_segments.back().end());

        // delete a segment if it provides same intersection info except starting waypoint
        if(i != 0){
            int cur = 1;
            while(cur < sorted_segments.back().size()){
                bool deleted = false;
                vector<string> id_cur = cubics[sorted_segments.back()[cur].segment].get_id();
                for(int prev = 0;prev<cur;prev++){
                    vector<string> id_prev = cubics[sorted_segments.back()[prev].segment].get_id();
                    if(id_cur[1].size() != id_prev[1].size()) continue;
                    if(id_cur[1].size() == 7){
                        if(id_cur[1].substr(0,4) == id_prev[1].substr(0,4)){
                            sorted_segments.back().erase(sorted_segments.back().begin() + cur);
                            deleted = true;
                            break;
                        }
                    }
                    else if(id_cur[1].size() == 12){
                        if(id_cur[1].substr(2,2) == id_prev[1].substr(2,2)){
                            sorted_segments.back().erase(sorted_segments.back().begin() + cur);
                            deleted = true;
                            break;
                        }
                    }                      
                }
                if(!deleted) cur++;
            }
        }
        if(debug){
            printf("[loadWaypoint] %d sorted segments\n", i);
            for(auto seg : sorted_segments[i]){
                vector<string> ids = cubics[seg.segment].get_id();
                printf("%s %s %s\n", ids[0].c_str(), ids[1].c_str(), ids[2].c_str());
            }
            printf("\n");
        }
    }

    sz = sorted_segments.size();

    // make explore order
    // ex) 3 waypoints, possible segments size = (1, 2, 2)
    // --> (0, 0, 0), (0, 0, 1), (0, 1, 0), (0, 1, 1) 
    vector<vector<int>> explore;
    vector<int> cur(sz, 0);
    while(1){
        explore.push_back(cur);

        // count ++
        cur[sz-1]++;
        for(int i = sz-1;i>=0;i--){
            if(cur[i] >= sorted_segments[i].size()){
            // if(cur[i] >= 2){
                if(i!=0) {
                    cur[i] = 0;
                    cur[i-1]++;
                    continue;
                }
                else break;
            }
            else break;
        }

        if(cur[0] >= sorted_segments[0].size()) break;
        // if(cur[0] >= 2) break;
        if(explore.size() > 1000000) break;
    }

    printf("[loadWaypoint] explore size : %d\n", (int)explore.size());

    sort(explore.begin(), explore.end(), 
        [](const vector<int> & a, const vector<int> & b) -> bool{
            int suma = 0, sumb = 0;
            for(auto i : a) suma += i;
            for(auto i : b) sumb += i;
            return suma < sumb;
        });

    // for each explore, check valid, if valid --> make mission info txt and return
    for(int explore_ind = 0;explore_ind < (int)explore.size();explore_ind++){
        vector<int>& v = explore[explore_ind];
        bool check = true;

        // check inters connected
        vector<string> inters;
        for(int i=0;i<sz;i++){
            int ind = sorted_segments[i][v[i]].segment;
            vector<string> id = cubics[ind].get_id();
            
            vector<string> inters_segment;

            if(i==0){
                // see second and third segments of 3 segments
                // then make list of intersections
                // ex) segment = [C2F2_34, C2F2G1_34S04, F2G1_04] --> inters_segment = [C2, F2, G1]
                if(id[1].length() == 12){
                    inters_segment.push_back(id[1].substr(0,2));
                    inters_segment.push_back(id[1].substr(2,2));
                    inters_segment.push_back(id[1].substr(4,2));
                }
                else if(id[2].length() == 12){
                    inters_segment.push_back(id[2].substr(0,2));
                    inters_segment.push_back(id[2].substr(2,2));
                    inters_segment.push_back(id[2].substr(4,2));
                }
                else{
                    inters_segment.push_back(id[1].substr(0,2));
                    inters_segment.push_back(id[1].substr(2,2));
                }
            }
            else{
                // see middle segment among 3 segments
                // then make list of intersections
                // ex) segment = [C2F2_34, C2F2G1_34S04, F2G1_04] --> inters_segment = [F2]
                // ex) segment = [C0C1C2_14S04, C1C2_04, C1C2A1_04R06] --> inters_segment = [C1, C2]
                if(id[1].length() == 12){
                    inters_segment.push_back(id[1].substr(2,2));
                }
                else{
                    inters_segment.push_back(id[1].substr(0,2));
                    inters_segment.push_back(id[1].substr(2,2));
                }
            }

            if(debug){
                printf("[loadWaypoint] %d inters : ", i);
                for(string ids : inters){
                    printf("%s ", ids.c_str());
                }
                printf("\n");

                printf("[loadWaypoint] %d inter segment : ", i);
                for(string ids : inters_segment){
                    printf("%s ", ids.c_str());
                }
                printf("\n\n");
            }

            // 지금까지 빌드된 intersection들 중 최근 2개와 비교, 같은것이 있으면 이어서 붙이게끔 or 불가능하면 invalid / 같은것이 없으면 뒤에 이어 붙이기
            // ex) inters = [A0, A1, C2] & inters_segment = [C2, F2] --> inters = [A0, A1, C2, F2]
            // ex) inters = [A0, A1, C2, C0] & inters_segment = [C2, F2] --> invalid
            bool same_contains = false;
            int same_j = -1;
            int same_k = -1;

            for(int j = (int)inters.size()-2;j<(int)inters.size();j++){
                for(int k = 0;k<(int)inters_segment.size();k++){
                    if(j < 0) continue;
                    if(inters[j] == inters_segment[k]){
                        same_contains = true;
                        same_j = j;
                        same_k = k;
                        break;
                    }
                }
            }
            if(same_contains){
                for(int offset = -1;offset<=1;offset++){
                    if(same_j + offset < 0 || same_j + offset >= (int)inters.size()) continue;
                    if(same_k + offset < 0 || same_k + offset >= (int)inters_segment.size()) continue;
                    if(inters[same_j + offset] != inters_segment[same_k + offset]){
                        check = false;
                        break;
                    }
                }
                if(!check) break;

                for(int offset = 1;offset<=1;offset++){
                    if(same_j + offset >= (int)inters.size() && same_k + offset < (int)inters_segment.size()){
                        inters.push_back(inters_segment[same_k + offset]);
                    }
                }

            }
            else{
                for(string str : inters_segment) inters.push_back(str);
            }
        }
        if(!check) continue;

        printf("[loadWaypoint] explore ind : %d\n\n", explore_ind);
        printf("[loadWaypoint] final intersections : ");
        for(string ids : inters){
            printf("%s ", ids.c_str());
        }
        printf("\n\n");
        printf("[loadWaypoint] selected segments : ");
        for(int i =0;i<sz;i++){
            printf("%s ", names[sorted_segments[i][v[i]].segment].c_str());
        }
        printf("\n\n");

        // check inters connecting segment really exist
        // and make mission info
        // ex) inters = [A0, A1, C2, C0] --> check segment A0A1, A1C2, C2C0, A0A1C2, A1C2C0
        int intersz = (int)inters.size();
        if(intersz <= 1) continue;

        vector<int> last_sectors(2*intersz-3);
        vector<char> tasks(2*intersz-3);
        vector<vector<int>> desired_lanes(2*intersz-3);
        vector<string> sectors(2*intersz-3);

        // make mission_info
        if(intersz >= 3){
            for(int i = 0;i<intersz-2;i++){
                string sector = inters[i] + inters[i+1] + inters[i+2];

                string expr = sector + "######";

                vector<int> matches = match_string(names, expr);
                if(matches.empty()){
                    check = false;
                    break;
                }

                last_sectors[2*i+1] = 0;
                last_sectors[2*i] = char_to_int(names[matches[0]][7]);
                tasks[2*i+1] = names[matches[0]][9];
                tasks[2*i] = 'D';
                for(int m : matches){
                    desired_lanes[2*i+1].push_back(char_to_int(names[m][8]));
                    desired_lanes[2*i].push_back(char_to_int(names[m][8]));
                }
                sectors[2*i+1] = sector;
                sectors[2*i] = sector.substr(0,4);

                expr = sectors[2*i]+"###";
                matches = match_string(names, expr);
                if(matches.empty()){
                    check = false;
                    break;
                }
            }
            string sector = inters[intersz-2] + inters[intersz-1];
            string expr = sector + "###";
            vector<int> matches = match_string(names, expr);
            if(matches.empty()){
                check = false;
            }
            for(int m : matches){
                desired_lanes[2*intersz-4].push_back(char_to_int(names[m][6]));
            }
            last_sectors[2*intersz-4] = 100;
            tasks[2*intersz-4] = 'D';
            sectors[2*intersz-4] = sector;
        }
        else{
            string sector = inters[0] + inters[1];
            string expr = sector + "###";
            vector<int> matches = match_string(names, expr);
            if(matches.empty()){
                check = false;
            }
            for(int m : matches){
                desired_lanes[0].push_back(char_to_int(names[m][6]));
            }
            last_sectors[0] = 100;
            tasks[0] = 'D';
            sectors[0] = sector;
        }
        
        if(!check) continue;

        // init segment
        int init_segment = sorted_segments[0][v[0]].segment;

        // make mission_info txt
        stringstream mission_info_path;
        mission_info_path << ros::package::getPath("global_map") << "/mission_config/mission_info.txt";
        ofstream mission_info_out(mission_info_path.str());
        mission_info_out<<init_segment<<endl;
        for(int i = 0;i<2*intersz-3;i++){
            mission_info_out<<tasks[i]<<' '<<last_sectors[i]<<' '<<sectors[i];
            for(int d : desired_lanes[i]) mission_info_out<<' '<<d;
            mission_info_out<<endl;
        }
        mission_info_out.close();
        return;
    }
    printf("[loadWaypoint] can not find possible route!\n");
    assert(false);
}


void LaneChangeHandler::update(int mid, double s, CubicSpline2D & cur){
    if(current_mission_id != mid){
        current_mission_id = mid;
        current_lane = cur.get_edge()[LANE_NUM1];
        current_lane_enter_segment = cur.get_index();
        current_lane_next_segment = -1;
        current_lane_enter_s = s;
        current_lane_enter_segment_length = cur.get_boundary()[2] - cur.get_boundary()[1];
    }

    if(current_lane != cur.get_edge()[LANE_NUM1]){
        current_lane = cur.get_edge()[LANE_NUM1];
        current_lane_enter_segment = cur.get_index();
        current_lane_next_segment = -1;
        current_lane_enter_s = s;
        current_lane_enter_segment_length = cur.get_boundary()[2] - cur.get_boundary()[1];
    }
    current_s = s;
    current_segment = cur.get_index();
}

void LaneChangeHandler::update(int mid, double s, CubicSpline2D & cur, CubicSpline2D & next){
    if(current_mission_id != mid){
        current_mission_id = mid;
        current_lane = cur.get_edge()[LANE_NUM1];
        current_lane_enter_segment = cur.get_index();
        current_lane_next_segment = next.get_index();
        current_lane_enter_s = s;
        current_lane_enter_segment_length = cur.get_boundary()[2] - cur.get_boundary()[1];
    }

    if(current_lane != cur.get_edge()[LANE_NUM1]){
        current_lane = cur.get_edge()[LANE_NUM1];
        current_lane_enter_segment = cur.get_index();
        current_lane_next_segment = next.get_index();
        current_lane_enter_s = s;
        current_lane_enter_segment_length = cur.get_boundary()[2] - cur.get_boundary()[1];
    }

    current_s = s;
    current_segment = cur.get_index();
}

double LaneChangeHandler::get_dist(){
    if(current_segment == current_lane_enter_segment) return current_s - current_lane_enter_s;
    else if(current_segment == current_lane_next_segment) return current_s - current_lane_enter_s + current_lane_enter_segment_length;
    else return 100;
}

void LightHandler::update(double t, int light, int current_task, int next_task, bool can_stop){
    if(current_task != DRIVING_SECTION) {
        go_signal = true;
        return;
    }
    if(light == -1) return;

    if(prev_light != light){
        prev_light = light;
        time_light_changed = t;
        duration_light_changed = 0;
    }
    else{
        prev_light = light;
        duration_light_changed = t - time_light_changed;
    }
    if(next_task == INTERSECTION_LEFT || next_task == INTERSECTION_STRAIGHT){
        if(light == 1) go_signal = true;
        else if(light == 2) go_signal = !can_stop;
        else go_signal = false;
    }
    else{
        go_signal = true;
    }
}

vector<int> match_string(vector<string> &names, string expr){
    vector<int> ret;
    for(int i = 0;i<names.size();i++){
        string str = names[i];
        if(expr.size() != str.size()) continue;

        bool match = true;
        for(int j = 0; j<expr.size();j++){
            if(expr[j] == '#') continue;
            if(expr[j] != str[j]) match = false;
        }
        if(match) ret.push_back(i);
    }
    return ret;
}

bool match_expr(string name, string expr){
    if(name.size() != expr.size()) return false;
    for(int i =0;i<name.size();i++){
        if(expr[i] == '#') continue;
        if(name[i] != expr[i]) return false;
    }
    return true;
}

void print_lanes(vector<string> &names, vector<Lane> & lanes){
    for(int i =0;i<lanes.size();i++){
        Lane & l = lanes[i];
        printf("[decision_maker] name %s, lane %d, l_dist %d, dir %d, cost %lf, d_left %lf, jam %s, speed %lf, ttrd %lf, vehicles %d, obstacle %s %lf\n",
            names[l.segment].c_str(), l.lane, l.l_dist, l.dir, l.cost, l.d_left, bool_to_string(l.jam), l.speed, l.ttrd, (int)l.vehicles.size(), bool_to_string(l.obstacle_exist), l.obstacle_dist
        );
    }
}

int char_to_int(char c){
    if('0' <= c && c <= '9') return c - '0';
    else return c - 'A';
}