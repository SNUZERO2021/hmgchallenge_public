#ifndef HISTORY
#define HISTORY

#include <iostream>
#include <string>
#include <queue>
#include "hmg_utils/ObjectInfo.h"
#include "hmg_utils/Position.h"

#include "ros/ros.h"
#include "ros/time.h"

using namespace std;


class History{
    public:
    hmg_utils::ObjectInfo object_info;
    queue<hmg_utils::Position> history;
    int size;
    double last_time;
    vector<int> segment_array;

    History();
    History(hmg_utils::ObjectInfo obj_info);
    bool check_id(string id);
    void insert(hmg_utils::Position pos);
    void update(double cur_time, double threshold);
    void updateSegmentArray(vector<int> segments);
};

#endif