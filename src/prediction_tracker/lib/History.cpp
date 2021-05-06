#include <iostream>

#include <hmg_utils/Position.h>
#include <hmg_utils/ObjectInfo.h>

#include "History.h"

using namespace std;



History::History(){
    size = 0;
    last_time = 0.0;
}

History::History(hmg_utils::ObjectInfo obj_info){
    object_info = obj_info;
    size = 0;
    last_time = 0.0;
}

bool History::check_id(string id){
    return object_info.id.compare(id);
}

void History::insert(hmg_utils::Position pos){
    history.push(pos);
    last_time = pos.t;
    size ++;
}


void History::update(double cur_time, double threshold){
    while(!history.empty()){
        if(history.front().t > cur_time - threshold) break;
        history.pop();
        size --;
    }
}

void History::updateSegmentArray(vector<int> segments){
    segment_array = segments;
}