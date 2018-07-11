#include <iostream>
#include <limits>
#include <algorithm>
#include <vector>
#include <math.h>
#include <time.h>
using namespace std;

#include "yaml-cpp/yaml.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp" 
using namespace cv;

#include "../include/astar.hpp"
#include "../include/astar_wrapper.hpp"

template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}

MapInfo getConfig(string input)
{
    MapInfo info;
    YAML::Node config = YAML::LoadFile(input);

    config["image"] >> info.filePath;
    config["occupied_thresh"] >> info.occ_th;
    config["free_thresh"] >> info.free_th;

    config["origin"][0] >> info.origin_x;
    config["origin"][1] >> info.origin_y;
    config["origin"][1] >> info.origin_yaw;
    config["resolution"] >> info.res;

    Mat Img = imread(info.filePath);
    info.row = Img.rows;
    info.col = Img.cols;

    return info;
}


vector<PosPoint> generatePath(vector<float> start, vector<float> end)
{
    MapInfo info = getConfig("../config/map.yaml");
    
    bool DrawPath = true;
    
    clock_t t_start, t_finish;
    t_start = clock();


    Mat maze = imread(info.filePath);
    int Row = maze.rows;
    int Col = maze.cols;
   
    Mat grid;
    cvtColor(maze, grid, cv::COLOR_BGR2GRAY);
    
    
    float* matrix = new float [Row*Col];
    float INF = numeric_limits<float>::infinity(); 


    // cout << Row*Col << endl;
    for(int i=0;i<Row;++i)
    {
        for(int j=0;j<Col;++j)
        {
            uchar pixel = grid.at<uchar>(i,j);
            unsigned int idx = i*Col+j;

            if (pixel < info.occ_th)
                matrix[idx] = INF;
            else if (pixel > info.free_th)
                matrix[idx] = 1.0;
            // cout << idx << "," << matrix[idx] << "," << (int)pixel << endl;
        }
        
    }

    //************** tf from world coordinate to pixel *************/
    int st_pix_i, st_pix_j, ed_pix_i, ed_pix_j;

    st_pix_i = Row -1 + floor((start[0]-info.origin_x)*sin(info.origin_yaw)/info.res - (start[1]-info.origin_y)*cos(info.origin_yaw)/info.res);
    st_pix_j = floor((start[0]-info.origin_x)*cos(info.origin_yaw)/info.res + (start[1]-info.origin_y)*sin(info.origin_yaw)/info.res);

    ed_pix_i = Row - 1 + floor((end[0]-info.origin_x)*sin(info.origin_yaw)/info.res - (end[1]-info.origin_y)*cos(info.origin_yaw)/info.res);
    ed_pix_j = floor((end[0]-info.origin_x)*cos(info.origin_yaw)/info.res + (end[1]-info.origin_y)*sin(info.origin_yaw)/info.res);

    int start_idx, end_idx;
    start_idx = st_pix_i * Col + st_pix_j;
    end_idx = ed_pix_i * Col + ed_pix_j;

    //******************** use astar to find path ****************/
    int* paths = new int[Row*Col];
    
    astar(matrix, Row, Col, start_idx, end_idx, paths);
    
    // for (unsigned int idx=0; idx<Row*Col; ++idx)
    //      cout << (paths[idx]) << endl;

    delete[] matrix;
    
    int path_idx = end_idx;

    vector<PosPoint> map;
    PosPoint pt(0, 0);

    while(path_idx != start_idx)
    {
        // cout << paths[path_idx] << endl;
        // convert 1D index to 2D index
        int i = path_idx/Col;
        int j = path_idx - (i*Col);
        // cout << i << ", " << j << endl;
        if(DrawPath)
        {    
            maze.at<Vec3b>(i,j)[0] = 0;
            maze.at<Vec3b>(i,j)[1] = 0;
            maze.at<Vec3b>(i,j)[2] = 255;
        }
        //********** tf from pixel to world coordinate ************/
        pt.x = j * info.res;
        pt.y = (Row - i - 1) * info.res;
        map.insert(map.begin(), pt);
        path_idx = paths[path_idx];
    }

    delete[] paths;
    
    if(DrawPath)
        imwrite("sol.png", maze);
    // imshow("path", maze);
    // waitKey(0);

    t_finish = clock();
    float duration = (float)(t_finish - t_start) / CLOCKS_PER_SEC;

    cout << "execution time: " << duration << endl;
    
    return map;
}


// int main(int argc, char** argv)
// {
//     vector<PosPoint> newPath;
//     newPath = generatePath(2, 3234589);
// 
//     newPath.clear();
//     return 0;
// }
