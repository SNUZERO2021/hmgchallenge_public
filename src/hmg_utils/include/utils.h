#ifndef FRENET_OPTIMAL_TRAJECTORY_UTILS_H
#define FRENET_OPTIMAL_TRAJECTORY_UTILS_H

#include <cmath>
#include <tuple>
#include <vector>
#include <stack>
#include <ros/console.h>
#include "ros/ros.h"

using namespace std;

typedef vector<double> Point;
typedef vector<double> Pose;

const double INF = 1e9;
const double EPS = 1e-6;

class PoseState{
    public:
        double x=0;
        double y=0;
        double yaw=0;
        double vx=0;
        double vy=0;
        double yawrate=0;
        double ax=0;
        double ay=0;
        Pose getPose(){
            Pose pose;
            pose.push_back(x);
            pose.push_back(y);
            pose.push_back(yaw);
            return pose;
        }
        void setPose(Pose pose){
            x = pose[0];
            y = pose[1];
            yaw = pose[2];
        }
        PoseState transform(Pose pose0, bool inverse = false){
            PoseState transformed;
            if(inverse == true){
                double x0 = pose0[0];
                double y0 = pose0[1];
                double yaw0 = pose0[2];
                pose0[0] = -x0*cos(yaw0) -y0*sin(yaw0);
                pose0[1] = x0*sin(yaw0) -y0*cos(yaw0);
                pose0[2] = -yaw0;
            }
            double tmpx = x - pose0[0];
            double tmpy = y - pose0[1];
            transformed.x = tmpx*cos(pose0[2]) + tmpy*sin(pose0[2]);
            transformed.y = -tmpx*sin(pose0[2]) + tmpy*cos(pose0[2]);
            transformed.yaw = yaw - pose0[2];
            transformed.yaw = remainder(transformed.yaw,2*M_PI);
            transformed.yawrate = yawrate;
            transformed.vx = vx*cos(pose0[2]) + vy*sin(pose0[2]);
            transformed.vy = -vx*sin(pose0[2]) + vy*cos(pose0[2]);
            transformed.ax = ax*cos(pose0[2]) + ay*sin(pose0[2]);
            transformed.ay = -ax*sin(pose0[2]) + ay*cos(pose0[2]);
            return transformed;
        }
};

struct SLState{
    public:
        double s;
        double l;
        double ds;
        double dl;
        double dds;
        double ddl;

    SLState():SLState(0,0){}
    SLState(double s1, double l1): s(s1), l(l1){}
};

inline double norm(double x, double y){
    return sqrt(pow(x,2) + pow(y,2));
}

inline void as_unit_vector(tuple<double, double>& vec){
    double magnitude = norm(get<0>(vec), get<1>(vec));
    if (magnitude > 0){
        get<0>(vec) = get<0>(vec) / magnitude;
        get<1>(vec) = get<1>(vec) / magnitude;
    }
}

inline double dot(const tuple<double, double>& vec1, const tuple<double, double>& vec2){
    return get<0>(vec1) * get<0>(vec2) + get<1>(vec1) * get<1>(vec2);
}


struct point{
    double x,y;
    double theta;
    point() : point(0,0){}
    point(double x1, double y1): x(x1),y(y1), theta(0){}
    void update(point p){
        theta = atan2(y-p.y,x-p.x);
    }
    bool operator <(const point& o){
        if(abs(theta-o.theta)>EPS) return theta<o.theta;
        if(abs(y-o.y)>EPS) return y<o.y;
        return x<o.x;
    }
    point operator +(const point &o) const{return point(x+o.x, y+o.y);}
    point operator -(const point &o) const{return point(x-o.x, y-o.y);}
    point operator -() const{return point(-x, -y);}
    double operator * (const point &o) const{ return x*o.x + y*o.y;}
    point operator *(const double t) const{return point(x*t, y*t);}
};

struct line{
    point s,e;
    line(): line(0,0,0,0){}
    line(double x1, double y1, double x2, double y2):s(x1,y1),e(x2,y2){}
    line(point p1, point p2):s(p1),e(p2){}
};

inline int ccw(const point &a, const point &b, const point &c){
    double rt = (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
    if(abs(rt)<EPS)return 0;
    if(rt<0) return -1;
    return 1;
}

inline bool intersect(const line &l1, const line &l2){
    int ab = ccw(l1.s,l1.e, l2.s)*ccw(l1.s,l1.e,l2.e);
    int cd = ccw(l2.s,l2.e,l1.s)*ccw(l2.s,l2.e,l1.e);

    if(ab==0 && cd ==0){
        if((l1.s.x>l2.s.x && l1.s.x>l2.e.x&&l1.e.x>l2.s.x&&l1.e.x>l2.e.x) ||(l1.s.x<l2.s.x && l1.s.x<l2.e.x&&l1.e.x<l2.s.x&&l1.e.x<l2.e.x)) return false;
        if((l1.s.y>l2.s.y && l1.s.y>l2.e.y&&l1.e.y>l2.s.y&&l1.e.y>l2.e.y) ||(l1.s.y<l2.s.y && l1.s.y<l2.e.y&&l1.e.y<l2.s.y&&l1.e.y<l2.e.y)) return false;
    }
    return (ab<=0)&&(cd<=0);
}

inline double sq(double x){ return x*x;}

inline double dist(const point &a, const point &b){return sqrt(sq(a.x-b.x)+sq(a.y-b.y));}
inline double dist2(const point &a, const point &b){return sq(a.x-b.x)+sq(a.y-b.y);}

inline double area(const point &a, const point &b){
    return 0.5*abs(a.y*b.x-a.x*b.y);
}

inline point vec(const point &s, const point &e){
    return point(e.x-s.x,e.y-s.y);
}

inline double length(const line &l){
    return dist(l.s,l.e);
}

inline bool valid(const point &p, const point &s, const point &e){ //수선의 발이 line(s,e)위에 있는지 확인
    double d = sq(dist(s,e));
    double x = sq(dist(p,s));
    double y = sq(dist(p,e));
    if(d+x<y||d+y<x) return false;
    return true;
}

inline bool valid2(const point &p, const point &s, const point &e){ //수선의 발이 line(s,e)위에 있는지 확인
    double d = dist2(s,e);
    double x = dist2(p,s);
    double y = dist2(p,e);
    if(d+x<y||d+y<x) return false;
    return true;
}

inline double line_distance(const line &l1, const line &l2){
    double rt = INF;
    rt = min<double>(rt, dist(l1.s, l2.s));
    rt = min<double>(rt, dist(l1.s, l2.e));
    rt = min<double>(rt, dist(l1.e, l2.s));
    rt = min<double>(rt, dist(l1.e, l2.e));

    if(valid(l1.s, l2.s, l2.e)) rt = min<double>(rt, 2.0*area(vec(l1.s,l2.s),vec(l1.s, l2.e))/length(l2));
    if(valid(l1.e, l2.s, l2.e)) rt = min<double>(rt, 2.0*area(vec(l1.e,l2.s),vec(l1.e, l2.e))/length(l2));
    if(valid(l2.s, l1.s, l1.e)) rt = min<double>(rt, 2.0*area(vec(l2.s,l1.s),vec(l2.s, l1.e))/length(l1));
    if(valid(l2.e, l1.s, l1.e)) rt = min<double>(rt, 2.0*area(vec(l2.e,l1.s),vec(l2.e, l1.e))/length(l1));

    return rt;
}

inline double line_distance2(const line &l1, const line &l2){
    double rt = INF;
    rt = min<double>(rt, dist2(l1.s, l2.s));
    rt = min<double>(rt, dist2(l1.s, l2.e));
    rt = min<double>(rt, dist2(l1.e, l2.s));
    rt = min<double>(rt, dist2(l1.e, l2.e));

    rt = sqrt(rt);

    if(valid(l1.s, l2.s, l2.e)) rt = min<double>(rt, 2.0*area(vec(l1.s,l2.s),vec(l1.s, l2.e))/length(l2));
    if(valid(l1.e, l2.s, l2.e)) rt = min<double>(rt, 2.0*area(vec(l1.e,l2.s),vec(l1.e, l2.e))/length(l2));
    if(valid(l2.s, l1.s, l1.e)) rt = min<double>(rt, 2.0*area(vec(l2.s,l1.s),vec(l2.s, l1.e))/length(l1));
    if(valid(l2.e, l1.s, l1.e)) rt = min<double>(rt, 2.0*area(vec(l2.e,l1.s),vec(l2.e, l1.e))/length(l1));

    return rt;
}

inline double getClearance_original(const vector<point>& A, const vector<point>& B){
    double rt = INF;

    int a = A.size();
    int b = B.size();
    for(int i=0; i<a;i++){
        line l1 = line(A[i],A[(i+1)%a]);
        for(int j=0;j<b;j++){
            line l2 = line(B[j],B[(j+1)%b]);
            if(rt > line_distance(l1, l2)){
            }
            rt = min<double>(rt, line_distance(l1,l2));
        }
    }

    // int a = A.size();
    // int b = B.size();
    // int min_a, min_b;
    // for(int i = 0 ; i<a;i++){
    //     for(int j = 0; j<b;j++){
    //         double d = dist(A[i], B[j]);
    //         if(rt>d){
    //             rt = d;
    //             min_a = i;
    //             min_b = j;
    //         }
    //     }
    // }
    // rt = min<double>(rt, line_distance(line(A[(min_a-1+a)%a], A[min_a]), line(B[(min_b-1+b)%b],B[min_b])));
    // rt = min<double>(rt, line_distance(line(A[(min_a-1+a)%a], A[min_a]), line(B[(min_b+1+b)%b],B[min_b])));
    // rt = min<double>(rt, line_distance(line(A[(min_a+1+a)%a], A[min_a]), line(B[(min_b-1+b)%b],B[min_b])));
    // rt = min<double>(rt, line_distance(line(A[(min_a+1+a)%a], A[min_a]), line(B[(min_b+1+b)%b],B[min_b])));
    
    return rt;
}

inline double getClearance(const vector<point>& A, const vector<point>& B){
    double rt = INF;

    // int a = A.size()-1;
    // int b = B.size()-1;
    // for(int i=0; i<a;i++){
    //     line l1 = line(A[i],A[i+1]);
    //     for(int j=0;j<b;j++){
    //         line l2 = line(B[j],B[j+1]);
    //         rt = min<double>(rt, line_distance(l1,l2));
    //     }
    // }

    int a = A.size();
    int b = B.size();
    int min_a, min_b;
    for(int i = 0 ; i<a;i++){
        for(int j = 0; j<b;j++){
            double d = dist(A[i], B[j]);
            if(rt>d){
                rt = d;
                min_a = i;
                min_b = j;
            }
        }
    }
    rt = min<double>(rt, line_distance(line(A[(min_a-1+a)%a], A[min_a]), line(B[(min_b-1+b)%b],B[min_b])));
    rt = min<double>(rt, line_distance(line(A[(min_a-1+a)%a], A[min_a]), line(B[(min_b+1+b)%b],B[min_b])));
    rt = min<double>(rt, line_distance(line(A[(min_a+1+a)%a], A[min_a]), line(B[(min_b-1+b)%b],B[min_b])));
    rt = min<double>(rt, line_distance(line(A[(min_a+1+a)%a], A[min_a]), line(B[(min_b+1+b)%b],B[min_b])));
    
    return rt;
}

inline double getClearance2(const vector<point>& A, const vector<point>& B){
    double rt = INF;

    // int a = A.size()-1;
    // int b = B.size()-1;
    // for(int i=0; i<a;i++){
    //     line l1 = line(A[i],A[i+1]);
    //     for(int j=0;j<b;j++){
    //         line l2 = line(B[j],B[j+1]);
    //         rt = min<double>(rt, line_distance(l1,l2));
    //     }
    // }

    int a = A.size();
    int b = B.size();
    int min_a, min_b;
    for(int i = 0 ; i<a;i++){
        for(int j = 0; j<b;j++){
            double d = dist2(A[i], B[j]);
            if(rt>d){
                rt = d;
                min_a = i;
                min_b = j;
            }
        }
    }

    rt = sqrt(rt);

    rt = min<double>(rt, line_distance(line(A[(min_a-1+a)%a], A[min_a]), line(B[(min_b-1+b)%b],B[min_b])));
    rt = min<double>(rt, line_distance(line(A[(min_a-1+a)%a], A[min_a]), line(B[(min_b+1+b)%b],B[min_b])));
    rt = min<double>(rt, line_distance(line(A[(min_a+1+a)%a], A[min_a]), line(B[(min_b-1+b)%b],B[min_b])));
    rt = min<double>(rt, line_distance(line(A[(min_a+1+a)%a], A[min_a]), line(B[(min_b+1+b)%b],B[min_b])));
    
    return rt;
}

inline double distance_to_segment(double x, double y, double x1, double y1, double x2, double y2){
    point p = point(x,y);
    line l = line(x1,y1,x2,y2);
    if(valid(p, l.s, l.e)) return 2.0*area(vec(p,l.s),vec(p,l.e))/length(l);
    return min<double>(dist(p,l.s),dist(p,l.e));
}

inline double getTriangleArea(const point &a, const point &b, const point &c){
    return 0.5*abs((a.x-b.x)*(a.y-c.y)-(a.y-b.y)*(a.x-c.x));
}

/*inline double distance_to_segment(double x, double y, double x1, double y1, double x2, double y2){
    double dx2 = x2 - x1;
    double dy2 = y2 - y1;
    double dx = x - x1;
    double dy = y - y1;
    if(dx*dx2 + dy*dy2 <=0) return norm(dx,dy);
    if(dx*dx2 + dy*dy2 >=norm(dx2,dy2)) return norm(dx-dx2,dy-dy2);
    return abs(dx*dy2 - dy*dx2)/norm(dx2,dy2);
}*/

struct ConvexHull{
    public:
    vector<point> p;
    int size;
    bool valid;
    string id;
    point center;
    double radius;

    ConvexHull(){
        size = 0;
        valid = true;
    }
    ConvexHull(vector<point> v){
        valid = true;
        double cx = 0;
        double cy = 0;
        radius = 0.0;
        size = 0;
        if(v.size()<3) {
            p=v;
            center = point(cx, cy);
            ROS_ERROR("ConvexHull Error : few point");
        }
        sort(v.begin(), v.end());
        for(int i=1;i<v.size();i++) v[i].update(v[0]);
        sort(v.begin()+1, v.end());
        stack<int> s;
        s.push(0);
        s.push(1);
        int next = 2;
        while(next<v.size()){
            while(s.size()>=2){
                int first, second;
                first = s.top();
                s.pop();
                second = s.top();
                if(ccw(v[second],v[first],v[next])>0){
                    s.push(first);
                    break;
                }
            }
            s.push(next);
            next++;
        }
        while(!s.empty()){
            p.push_back(v[s.top()]);
            cx += v[s.top()].x;
            cy += v[s.top()].y;
            s.pop();
            size++;
        }
        cx /= size;
        cy /= size;
        center = point(cx,cy);
        for(int i=0;i<size;i++) radius = max<double>(radius, dist(center, p[i]));
    }

    ConvexHull(vector<point> v, string id){
        valid = true;
        double cx = 0;
        double cy = 0;
        radius = 0.0;
        size = 0;
        if(v.size()<3) {
            p=v;
            center = point(cx, cy);
            ROS_ERROR("ConvexHull Error : few point");
        }
        sort(v.begin(), v.end());
        for(int i=1;i<v.size();i++) v[i].update(v[0]);
        sort(v.begin()+1, v.end());
        stack<int> s;
        s.push(0);
        s.push(1);
        int next = 2;
        while(next<v.size()){
            while(s.size()>=2){
                int first, second;
                first = s.top();
                s.pop();
                second = s.top();
                if(ccw(v[second],v[first],v[next])>0){
                    s.push(first);
                    break;
                }
            }
            s.push(next);
            next++;
        }
        while(!s.empty()){
            p.push_back(v[s.top()]);
            cx += v[s.top()].x;
            cy += v[s.top()].y;
            s.pop();
            size++;
        }
        cx /= size;
        cy /= size;
        center = point(cx,cy);
        for(int i=0;i<size;i++) radius = max<double>(radius, dist(center, p[i]));
        
        this->id = id;
    }
    const void print_all() const{
        cout << "ConvexHull" <<endl;
        for(int i=0;i<size;i++){
            cout << p[i].x << " " << p[i].y << endl;
        }
        cout << endl;
        cout << endl;
        cout << endl;
    }

    double getArea(){
        double rt = 0.0;
        for(int i=1;i<p.size()-2;i++){
            rt += getTriangleArea(p[0],p[i],p[i+1]);
        }
        return rt;
    }
};

inline bool checkCollision(const ConvexHull &a, const ConvexHull &b){
    for(int i=0;i<a.size;i++){
        for(int j=0;j<b.size;j++){
            line l1 = line(a.p[(i%a.size)],a.p[((i+1)%a.size)]);
            line l2 = line(b.p[(j%b.size)],b.p[((j+1)%b.size)]);
            if(intersect(l1,l2)) return true;
        }
    }

    bool check = true;
    point p = b.p[0];
    int val = ccw(p,a.p[a.size-1],a.p[0]);
    for(int i=0;i<a.size-1;i++){
        if(ccw(p,a.p[i],a.p[i+1])!=val) {
            check = false;
            break;
        }
    }
    if(check) return true;
    check = true;
    p=a.p[0];
    val = ccw(p,b.p[b.size-1],b.p[0]);
    for(int i=0;i<b.size-1;i++){
        if(ccw(p,b.p[i],b.p[i+1])){
            check = false;
            break;
        }
    }
    if(check) return true;
    return false;
}

inline bool checkCollision_test(const ConvexHull &A, const ConvexHull &B){
    double rt = INF;
    int a = A.size;
    int b = B.size;

    line l1;
    line l2;
    l1 = line(A.p[a-1], A.p[0]);
    int id = -1;
    for(int i=0;i<b;i++){
        l2 = line(B.p[i],B.p[(i+1)%b]);
        double d = line_distance(l1, l2);
        if(rt>d){
            rt =d;
            id = i;
        }
    }
    for(int i=0;i<a-1;i++){
        l1 = line(A.p[i], A.p[i+1]);
        l2 = line(B.p[id], B.p[(id+1)%b]);
        double local_min = line_distance(l1, l2);
        bool inc = true;
        if(line_distance(l1, line(B.p[(id-1+b)%b],B.p[id])) < line_distance(l1, line(B.p[id],B.p[(id+1)%b]))) inc = false;
        if(inc){
            while(1){
                int tmp = (id+1)%b;
                double d = line_distance(l1, line(B.p[tmp],B.p[(tmp+1)%b]));
                if(d> local_min) break;
                local_min = d;
                id = tmp;
            }
        }
        else{
            while(1){
                int tmp = (id-1+b)%b;
                double d = line_distance(l1, line(B.p[tmp], B.p[(tmp+1)%b]));
                if(d>local_min) break;
                local_min =d;
                id = tmp;
            }
        }
        rt = min<double>(rt, local_min);
    }
    if(rt<EPS) return true;
    bool check = true;
    point p = B.p[0];
    int val = ccw(p,A.p[a-1],A.p[0]);
    for(int i=0;i<a-1;i++){
        if(ccw(p,A.p[i],A.p[i+1])!=val) {
            check = false;
            break;
        }
    }
    if(check) return true;
    check = true;
    p=A.p[0];
    val = ccw(p,B.p[b-1],B.p[0]);
    for(int i=0;i<b-1;i++){
        if(ccw(p,B.p[i],B.p[i+1])){
            check = false;
            break;
        }
    }
    if(check) return true;
    return false;
}

inline ConvexHull UnionConvexHull(const ConvexHull &a, const ConvexHull &b){
    vector<point> np;
    for(point p : a.p) np.push_back(p);
    for(point p : b.p) np.push_back(p);

    return ConvexHull(np);
}

inline ConvexHull UnionConvexHull(const vector<ConvexHull>& v){
    vector<point> p;
    for(ConvexHull cvh : v) p.insert(p.end(), cvh.p.begin(), cvh.p.end());
    return ConvexHull(p);
}

inline bool isIncludeConvexHull(const ConvexHull &c, const point &v){
    bool rt = true;
    if(dist(c.center, v)>c.radius) false;
    int val = ccw(v,c.p[c.size-1],c.p[0]);
    for(int i=0;i<c.size-1;i++){
        if(ccw(v,c.p[i],c.p[i+1])!=val) {
            rt = false;
            break;
        }
    }
    return rt;
}

struct PNode{
    int x,y;
    vector<int> segments;
    PNode(){
        x = 0, y= 0;
    }
    PNode(int a, int b, vector<int> v):x(a),y(b),segments(v){}
};

struct NNode{
    int x,y;
    int nearest;
    NNode():NNode(0,0,-1){}
    NNode(int a, int b, int c):x(a),y(b),nearest(c){}
};

enum node_num{
    PREDICTION_TRACKER,
    DECISION_MAKER,
    PATH_PLANNING,
    LIGHT_CONTROL,
};

enum flag{
    START,
    CHECK,
    SET_DEBUG,
};

void time_check(int node_num, int flag, string tag = "", bool debug = false);

// point add(const point & a, const point & b);
// point subtract(const point & a, const point & b);
// point reverse_point(const point & v);
// point perpendicular(const point & v);
// double dotProduct(const point & a, const point & b);
// point multiply_point(double t, const point & v);
double lengthSquared (const point & v);

point tripleProduct (const point & a, const point & b, const point & c);
point averagePoint(const vector<point> & vertices);
int indexOfFurthestPoint(const vector<point> & vertices, const point & d);
point support(const vector<point> & vertices1, const vector<point> & vertices2, const point & d);
point closest_point_to_origin(const point & a);
point closest_point_to_origin(const point & a, const point & b);
point closest_point_to_origin(const point & a, const point & b, const point & c);
double gjk(const vector<point>& vertices1, const vector<point>& vertices2);

#endif
