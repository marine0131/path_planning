#ifndef ASTAR_HPP
#define ASTAR_HPP

// represents a single pixel                                                                                                                                                                           
class Node {                                                                    
public:                                                                       
    int idx;     // index in the flattened grid                                 
    float cost;  // cost of traversing this pixel                               
                                                                                 
    Node(int i, float c) : idx(i),cost(c) {}                                    
};
          
bool astar(const float* weights, const int height, const int width,
              const int start, const int goal,
                    int* paths);
#endif
