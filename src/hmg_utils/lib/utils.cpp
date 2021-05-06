#include "ros/ros.h"
#include "utils.h"
#include <vector>
#include <sstream>

using namespace std;

void time_check(int node_num, int flag, string tag, bool debug){
    static double t[4];
    static vector<double> times[4];
    static vector<string> tags[4];
    static bool debugs[4];
    static double max_times[4];
    static bool first = true;

    if(first){
        for(int i =0;i<4;i++) max_times[i] = 0;
        first = false;
    }

    if(node_num != PREDICTION_TRACKER && node_num != DECISION_MAKER && node_num != PATH_PLANNING && node_num != LIGHT_CONTROL){
        ROS_INFO("time check what node num ??");
        assert(false);
    }

    if(flag == SET_DEBUG){
        debugs[node_num] = debug;
        return;
    }

    if(!debugs[node_num]) return;

    // init
    if(flag == START){
        double sum = 0;
        for(int i = 0; i<times[node_num].size();i++){
            sum += times[node_num][i];
            stringstream ss;
            if(node_num == PREDICTION_TRACKER) ss << "[prediction_tracker] ";
            else if(node_num == DECISION_MAKER) ss << "[decision_maker] ";
            else if(node_num == PATH_PLANNING) ss << "[PP] ";
            else if(node_num == LIGHT_CONTROL) ss << "[light_control] ";
            ss<<"ELASPED TIME ";
            if(tags[node_num][i] != "") ss <<"<" << tags[node_num][i] << "> " << times[node_num][i] << "sec ";
            else ss << times[node_num][i] << "sec ";
            ROS_INFO("%s", ss.str().c_str());
        }
        max_times[node_num] = max(sum, max_times[node_num]);
        stringstream ss;
        if(node_num == PREDICTION_TRACKER) ss << "[prediction_tracker] ";
        else if(node_num == DECISION_MAKER) ss << "[decision_maker] ";
        else if(node_num == PATH_PLANNING) ss << "[PP] ";
        else if(node_num == LIGHT_CONTROL) ss << "[light_control] ";
        ss<<"TOTAL ELASPED TIME "<<sum<<"sec";
        //if(sum > 0.1) assert(false);
        ROS_INFO("%s", ss.str().c_str());

        stringstream ss2;
        if(node_num == PREDICTION_TRACKER) ss2 << "[prediction_tracker] ";
        else if(node_num == DECISION_MAKER) ss2 << "[decision_maker] ";
        else if(node_num == PATH_PLANNING) ss2 << "[PP] ";
        else if(node_num == LIGHT_CONTROL) ss2 << "[light_control] ";
        ss2<<"MAX TOTAL ELASPED TIME "<<max_times[node_num]<<"sec";
        ROS_INFO("%s", ss2.str().c_str());
        t[node_num] = clock();
        times[node_num].clear();
        tags[node_num].clear();
    }
    // time check
    else if(flag == CHECK){
        double tmp = clock();
        times[node_num].push_back((tmp-t[node_num])/CLOCKS_PER_SEC);
        tags[node_num].push_back(tag);
        t[node_num] = tmp;
    }
    // print
    else{
        ROS_INFO("time check flag is not START nor CHECK");
        //assert(false);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// point add(const point & a, const point & b){return point(a.x+b.x, a.y+b.y);}
// point subtract(const point & a, const point & b) {return point(a.x-b.x, a.y-b.y);}
// point reverse_point(const point & v) {return point(-v.x, -v.y);}
// point perpendicular(const point & v) {return point(v.y, -v.x);}
// double dotProduct(const point & a, const point & b) {return a.x*b.x + a.y*b.y;}
// point multiply_point(double t, const point & v){ return point(t*v.x,t*v.y);}
double lengthSquared (const point & v) {return v.x*v.x + v.y*v.y;}

point tripleProduct (const point & a, const point & b, const point & c){
    double ac = a*c; // perform a.dot(c)
    double bc = b*c; // perform b.dot(c)
    return b*ac - a*bc;
    // return point(b.x * ac - a.x * bc, b.y*ac - a.y*bc);
}

point averagePoint(const vector<point> & vertices){
    point avg(0,0);
    for(auto & v : vertices){
        avg.x += v.x;
        avg.y += v.y;
    }
    avg.x /= vertices.size();
    avg.y /= vertices.size();
    return avg;
}

int indexOfFurthestPoint(const vector<point> & vertices, const point & d){
    double maxProduct = d *vertices[0];
    int index = 0;
    for (int i = 1; i < vertices.size(); i++) {
        double product = d * vertices[i];
        if (product > maxProduct) {
            maxProduct = product;
            index = i;
        }
    }
    return index;
}

point support(const vector<point> & vertices1, const vector<point> & vertices2, const point & d){
    int i = indexOfFurthestPoint(vertices1, d);
    int j = indexOfFurthestPoint(vertices2, -d);
    return vertices1[i]-vertices2[j];
}


point closest_point_to_origin(const point & a){
    return a;
}

point closest_point_to_origin(const point & a, const point & b){
    point ba = a-b;
    if(lengthSquared(ba) < EPS) return a;
    double u = -(b*ba)/lengthSquared(ba);
    if(u>1) return a;
    if(u<0) return b;
    return a*u + b*(1-u);
}

point closest_point_to_origin(const point & a, const point & b, const point & c){
    int abc = ccw(a,b,c);
    if(abc==0){
        point rt1 = closest_point_to_origin(a,b);
        point rt2 = closest_point_to_origin(a,c);
        if(lengthSquared(rt1)>lengthSquared(rt2)) return rt2;
        return rt1;
    }
    if(abc+ccw(a,b,point(0,0))==0){
        return closest_point_to_origin(a,b);
    }
    if(abc+ccw(b,c,point(0,0))==0){
        return closest_point_to_origin(b,c);
    }
    if(abc+ccw(c,a,point(0,0))==0){
        return closest_point_to_origin(c,a);
    }
    return point(0,0);
}

double gjk(const vector<point> &vertices1, const vector<point> &vertices2) {
    // printf("start gjk!!\n");
    point a, b, c, d;
    double dc, da;
    
    point position1 = averagePoint (vertices1); // not a CoG but
    point position2 = averagePoint (vertices2); // it's ok for GJK )

    // initial direction from the center of 1st body to the center of 2nd body
    d = position1 - position2;
    
    // if initial direction is zero – set it to any arbitrary axis (we choose X)
    if ((d.x == 0) && (d.y == 0))
        d.x = 1.f;
    
    // set the first support as initial point of the new simplex
    a = support(vertices1, vertices2, d);
    b = support(vertices1, vertices2, -d);

    d = closest_point_to_origin(a,b);
    double dist = lengthSquared(d);

    // if (dotProduct (a, d) <= 0)
    //     return false; // no collision
    
    while (1) {
        d = -d;
        
        if(abs(d.x)<EPS && abs(d.y)<EPS) return 0.0;
        
        c =support(vertices1, vertices2, d);
        if(lengthSquared(closest_point_to_origin(a,b,c)) < EPS) return 0.0;

        dc = d*c;
        da = d*a;

        // printf("a : %lf %lf, b : %lf %lf, c : %lf %lf, d : %lf %lf \n dc : %lf, da : %lf\n dist : %lf\n", a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y, dc, da, sqrt(lengthSquared(d)));

        if(dc - da < EPS) return sqrt(lengthSquared(d));

        point p1 = closest_point_to_origin(a,c);
        point p2 = closest_point_to_origin(c,b);

        // printf("a : %lf %lf, b : %lf %lf, c : %lf %lf, d : %lf %lf, p1 : %lf %lf, p2 : %lf %lf \n dc : %lf, da : %lf\n dist : %lf\n", a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y, p1.x, p1.y, p2.x, p2.y, dc, da, sqrt(lengthSquared(d)));
        if(lengthSquared(p1) > lengthSquared(p2)){
            // printf("select p2\n");
            a = c;
            d = p2;
        }
        else{
            // printf("select p1\n");
            b = c;
            d = p1;
        }
        // printf("d selected : %lf %lf\n", d.x, d.y);
        if(dist - lengthSquared(d) < EPS) break;
        dist = lengthSquared(d);
    }
    return sqrt(dist);
}

// int iter_count = 0;

// int gjk (const vec2 * vertices1, size_t count1,
//          const vec2 * vertices2, size_t count2) {
    
//     size_t index = 0; // index of current vertex of simplex
//     vec2 a, b, c, d, ao, ab, ac, abperp, acperp, simplex[3];
    
//     vec2 position1 = averagePoint (vertices1, count1); // not a CoG but
//     vec2 position2 = averagePoint (vertices2, count2); // it's ok for GJK )

//     // initial direction from the center of 1st body to the center of 2nd body
//     d = subtract (position1, position2);
    
//     // if initial direction is zero – set it to any arbitrary axis (we choose X)
//     if ((d.x == 0) && (d.y == 0))
//         d.x = 1.f;
    
//     // set the first support as initial point of the new simplex
//     a = simplex[0] = support (vertices1, count1, vertices2, count2, d);
    
//     if (dotProduct (a, d) <= 0)
//         return 0; // no collision
    
//     d = reverse_point (a); // The next search direction is always towards the origin, so the next search direction is reverse_point(a)
    
//     while (1) {
//         iter_count++;
        
//         a = simplex[++index] = support (vertices1, count1, vertices2, count2, d);
        
//         if (dotProduct (a, d) <= 0)
//             return 0; // no collision
        
//         ao = reverse_point (a); // from point A to Origin is just negative A
        
//         // simplex has 2 points (a line segment, not a triangle yet)
//         if (index < 2) {
//             b = simplex[0];
//             ab = subtract (b, a); // from point A to B
//             d = tripleProduct (ab, ao, ab); // normal to AB towards Origin
//             if (lengthSquared (d) == 0)
//                 d = perpendicular (ab);
//             continue; // skip to next iteration
//         }
        
//         b = simplex[1];
//         c = simplex[0];
//         ab = subtract (b, a); // from point A to B
//         ac = subtract (c, a); // from point A to C
        
//         acperp = tripleProduct (ab, ac, ac);
        
//         if (dotProduct (acperp, ao) >= 0) {
            
//             d = acperp; // new direction is normal to AC towards Origin
            
//         } else {
            
//             abperp = tripleProduct (ac, ab, ab);
            
//             if (dotProduct (abperp, ao) < 0)
//                 return 1; // collision
            
//             simplex[0] = simplex[1]; // swap first element (point C)

//             d = abperp; // new direction is normal to AB towards Origin
//         }
        
//         simplex[1] = simplex[2]; // swap element in the middle (point B)
//         --index;
//     }
    
//     return 0;
// }