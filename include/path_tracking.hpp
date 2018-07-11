#ifndef PATH_TRACKING_HPP
#define PATH_TRACKING_HPP

// integrate position and velocity status of the moving robot
class State{
public:
    float x;
    float y;
    float yaw;
    float v;

    State(float i, float j, float k, float l) : x(i),y(j),yaw(k),v(l) {}
};
#endif
