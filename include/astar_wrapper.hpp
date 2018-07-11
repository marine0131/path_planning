#ifndef ASTAR_WRAPPER_HPP
#define ASTAR_WRAPPER_HPP

class PosPoint
{
public:
    float x;
    float y;

    PosPoint(float i, float j) : x(i), y(j){}
};

class MapInfo
{
public:
    string filePath;
    int occ_th;
    int free_th;
    float origin_x;
    float origin_y;
    float origin_yaw;
    float res;
    int col;
    int row;
};

MapInfo getConfig(string input);

std::vector<PosPoint> generatePath(std::vector<float> start, std::vector<float> end);

#endif
