#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <string>
#include <math.h>
#include <vector>
#include <algorithm>
using namespace std;

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "../include/astar_wrapper.hpp"
#include "../include/path_tracking.hpp"

bool draw_trace = true;

float k = 0.1;
float Lfc = 1.0;
float Kp = 1.0;
float dt = 0.1;
float L = 2.9;

float simErr(void)
{
    float e_out;
    srand((unsigned)time(NULL));
    e_out = rand() / double(RAND_MAX) * 0.3;
    return e_out;
}

State updateState(State& curr, float a, float delta)
{
    curr.x = curr.x + curr.v * cos(curr.yaw) * dt + simErr();
    curr.y = curr.y + curr.v * sin(curr.yaw) * dt + simErr();
    curr.yaw = curr.yaw + curr.v / L *tan(delta) * dt + simErr();
    curr.v = curr.v + a * dt + simErr();
    
    return curr;
}

float PIDControl(float target, float current)
{
    float a;
    a = Kp * (target - current);
    return a;
}

int calc_target_index(State& state, vector<PosPoint> course)
{
    // find the nearset point index
    vector<float> dist;
    float dx,dy,d;
    for(vector<PosPoint>::iterator iter=course.begin(); iter!=course.end(); ++iter)
    {
        dx = state.x - iter->x;
        dy = state.y - iter->y;
        d = fabs(sqrt(pow(dx,2)+pow(dy,2)));
        dist.push_back(d);
    }
    int idx = distance(dist.begin(), min_element(dist.begin(),dist.end()));
    dist.clear();

    // smooth the movement and eliminate concussion
    float L = 0.0;
    float Lf = k * state.v + Lfc;
    // cout << "lf:" << Lf << endl;
    while(Lf > L && (idx + 1) < course.size())
    {
        dx = course[idx+1].x - course[idx].x;
        dy = course[idx+1].y - course[idx].y;
        L += sqrt(pow(dx,2)+pow(dy,2));
        // cout << L << endl;
        idx ++;
    }

    return idx;

}

float purePursuit(State state, vector<PosPoint> course, int& pind)
{
    int ind = calc_target_index(state, course);

    if(pind > ind)
        ind = pind;
    
    float tx, ty;

    // make sure the target is on the course
    if(ind < course.size())
    {
        tx = course[ind].x;
        ty = course[ind].y;
    }
    else
    {
        ind = course.size() - 1;
        tx = course.back().x;
        ty = course.back().y;
    }

    pind = ind;

    float alpha = atan2(ty-state.y, tx-state.x) - state.yaw;

    if(state.v < 0)
    {
        alpha = M_PI - alpha;
    }
    // cout << alpha << endl;
    // steer slowly, the adjustment angle is proportional to the speed
    float Lf = k * state.v + Lfc;
    // cout << "lf " << Lf << endl;
    float delta = atan2(2.0 * L * sin(alpha), Lf);

    return delta;
}

vector<PosPoint> generateTestSample(void)
{
    vector<PosPoint> output;
    PosPoint tmp(0,0);
    for (float cx=0.0; cx <50.0; cx+=0.1)
    {
        float cy = sin(cx / 5.0)*cx / 2.0;
        tmp.x = cx;
        tmp.y = cy;
        output.push_back(tmp);
    }
    return output;
}

bool tracking(State state, vector<float> start, vector<float> end)
{
    MapInfo map_info = getConfig("../config/map.yaml");
    cv::Mat m(map_info.row, map_info.col, CV_8UC3, cv::Scalar(255 ,255 ,255));

    clock_t t_start, t_finish;
    t_start = clock();

    float max_speed = 10.0 / 3.6;

    vector<PosPoint> trace;
    trace = generatePath(start, end);

    vector<PosPoint> downSampled;
    for(unsigned int i=0; i<trace.size(); i++)
    {
        if(i%10 == 0)
            downSampled.push_back(trace[i]);
    
    }
    trace.clear();

    if(draw_trace)
    {
        for(unsigned int i=0; i<downSampled.size(); i++)
        {
            m.at<cv::Vec3b>(map_info.row-1-floor(downSampled[i].y/map_info.res), floor(downSampled[i].x/map_info.res))[0] = 0;
            m.at<cv::Vec3b>(map_info.row-1-floor(downSampled[i].y/map_info.res), floor(downSampled[i].x/map_info.res))[1] = 0;
        }
    }
   

    int lastIdx = downSampled.size() - 1;
    // cout << "last index: " << lastIdx << endl;
    int target_idx = calc_target_index(state, downSampled);

    while(target_idx < lastIdx)
    {
        // cout << " v: " << state.v << endl;
        float ai = PIDControl(max_speed, state.v);
        // cout << " ai: " << ai << endl;
        float di = purePursuit(state, downSampled, target_idx); 
        // cout << " di: " << di << "idx:" << target_idx << endl;
        
        state = updateState(state, ai, di);
        cout << "state: " << state.x << " " << state.y << " " << state.yaw << " " << state.v << endl;
        // cout << "target idx: " << target_idx << endl;
        if(draw_trace)
        { 
            int i = map_info.row-1-floor(state.y/map_info.res);
            if(i < 0)
                i = 0;
            int j = floor(state.x/map_info.res);

            m.at<cv::Vec3b>(i, j)[2] = 0;
        }
        
        // publish linear velocity and angular velocity here

    }

    state.v = 0.0;
    
    while(fabs(state.yaw - end[3]) > 0.05)
    {
        float v_rad = 0.3;
        if(state.yaw < end[3])
            state.yaw += v_rad * dt;
        else
            state.yaw -= v_rad * dt;
        
        cout << "rotating..." << endl;
        cout << "state: " << state.x << " " << state.y << " " << state.yaw << " " << state.v << endl;
        // publish angular velocity here
    }

    t_finish = clock();
    float duration = (float)(t_finish - t_start) / CLOCKS_PER_SEC;
    cout << "execution time: " << duration << endl;
    
    if(draw_trace)
        imwrite("track.jpg", m);

    return true;

}

int main(int argc, char** argv)
{
    vector <float> start(3);
    vector <float> end(3);

    // initialtion
    State state(0.01, 90.0, 0.0, 0.0);
    
    start[0] = 0.1;
    start[1] = 90.05;
    start[2] = 0.0;

    end[0] = 90.05;
    end[1] = 0.35;
    end[2] = 0.0;


    tracking(state, start, end);
    
    return 0;
}
