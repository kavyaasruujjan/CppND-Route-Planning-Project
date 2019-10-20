#include "route_planner.h"
#include <algorithm>

/* Constructor */
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) 
{
	start_x *= 0.01; //  Since node coordinates are scaled to values between 0 an 1
    start_y *= 0.01;
    end_x   *= 0.01;
    end_y   *= 0.01;
  
	// find the closest nodes to `(start_x, start_y)` and `(end_x, end_y)`. Store pointers to these nodes in the `start_node` and `end_node` class variables.  
    start_node = &model.FindClosestNode(start_x, start_y); 
    end_node = &model.FindClosestNode(end_x, end_y);
}

/* Constructs the final path to be displayed on the map, as A*Search could search through several paths*/
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
  std::vector<RouteModel::Node> path_found {};
  distance = 0;
  while(current_node -> parent != nullptr)
  {
   	path_found.push_back(*current_node);
    distance += current_node->distance(*(current_node -> parent));
    current_node = current_node -> parent;
  }
  //Push back the starting node
  path_found.push_back(*current_node);
  //since node coordinates are scaled down when they are stored in the model. They must be rescaled to get an accurate distance.
  distance *= m_Model.MetricScale();
  return path_found;  
}

/* A*Search */
void RoutePlanner::AStarSearch()
{
  start_node->visited = true;
  open_list.push_back(start_node);
  RouteModel::Node *current_node = nullptr;
  while (open_list.size() > 0)
  {
    current_node = NextNode();
    if (current_node -> distance(*end_node) == 0)
    {
      m_Model.path = ConstructFinalPath(current_node);
      return;
    }
      AddNeighbors(current_node); 
  }
}

/* Calculate H Value. The method will return the distance from the passed argument to the `end_node`*/
float RoutePlanner::CalculateHValue(const RouteModel::Node* node)
{
  return node->distance(*end_node);
}

/* Calculate the next node from the open_list with lowest f value (sum of h and g values)*/
RouteModel::Node* RoutePlanner::NextNode()
{
  std::sort(open_list.begin(), open_list.end(), [] (RouteModel::Node* a, RouteModel::Node* b) {
    float f1 = a->h_value + a->g_value;
    float f2 = b->h_value + b->g_value;
    return f1 > f2;
  });
  RouteModel::Node* node_lowest_fvalue = open_list.back();
  open_list.pop_back();
  return node_lowest_fvalue;
}

/* Add neighbors to the open_list once neighbors are filled with appropriate values */
void RoutePlanner::AddNeighbors(RouteModel::Node* current_node)
{
  current_node->FindNeighbors();
  for (RouteModel::Node* neighbor : current_node->neighbors)
  {
    neighbor -> parent = current_node;
    neighbor -> g_value = current_node -> g_value + current_node -> distance(*neighbor);
    neighbor -> h_value = CalculateHValue(neighbor);
    open_list.push_back(neighbor);
    neighbor -> visited = true;    
  }
}