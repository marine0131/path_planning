//#include <queue>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>

#include "../include/astar.hpp"
using namespace std;

bool smallEnd(Node a, Node b)
{
    return (a.cost > b.cost);
}


// bool operator<(const Node &n1, const Node &n2) {
//   return n1.cost > n2.cost;
// }

bool operator==(const Node &n1, const Node &n2) {
  return n1.idx == n2.idx;
}

// manhattan distance: requires each move to cost >= 1
float heuristic(int i0, int j0, int i1, int j1) {
  return std::abs(i0 - i1) + std::abs(j0 - j1);
}

// weights:        flattened h x w grid of costs
// start, goal:    index of start/goal in flattened grid
// paths (output): for each node, stores previous node in path
bool astar(
      const float* weights, const int height, const int width,
      const int start, const int goal,
      int* paths) 
{

  const float INF = std::numeric_limits<float>::infinity();

  Node start_node(start, 0.);
  Node goal_node(goal, 0.);

  float* costs = new float[height * width];
  for (int i = 0; i < height * width; ++i)
    costs[i] = INF;
  costs[start] = 0.;

  vector<Node> nodes_to_visit;
  nodes_to_visit.push_back(start_node);

  int* nbrs = new int[4];

  bool solution_found = false;
  while (!nodes_to_visit.empty()) 
  {
    vector<Node>::iterator tail = nodes_to_visit.end() - 1;
      
    Node cur = *(tail);

    if (cur == goal_node) 
    {
      solution_found = true;
      break;
    }

    nodes_to_visit.erase(tail);

    // check bounds and find up to four neighbors
    // if not exist, the index is -1
    nbrs[0] = (cur.idx / width > 0) ? (cur.idx - width) : -1; //up 
    nbrs[1] = (cur.idx % width > 0) ? (cur.idx - 1) : -1; //left
    nbrs[2] = (cur.idx / width + 1 < height) ? (cur.idx + width) : -1; //down
    nbrs[3] = (cur.idx % width + 1 < width) ? (cur.idx + 1) : -1; //right
    for (int i = 0; i < 4; ++i) 
    {
      if (nbrs[i] >= 0) 
      {
        // the sum of the cost so far and the cost of this move
        float new_cost = costs[cur.idx] + weights[nbrs[i]];
        if (new_cost < costs[nbrs[i]]) 
        {
          costs[nbrs[i]] = new_cost;
          float priority = new_cost + heuristic(nbrs[i] / width,
                                                nbrs[i] % width,
                                                goal / width,
                                                goal % width);
          // paths with lower expected cost are explored first
          nodes_to_visit.push_back(Node(nbrs[i], priority));
          paths[nbrs[i]] = cur.idx;
        }
      }
    }
    sort(nodes_to_visit.begin(),nodes_to_visit.end(),smallEnd);
  }

  delete[] costs;
  delete[] nbrs;

  return solution_found;
}
