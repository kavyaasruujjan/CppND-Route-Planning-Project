#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y); //Constructor
    // Add public variables or methods declarations here.
    float GetDistance() const { return distance;}; // used to print out the total distance from `main.cpp`
    void AStarSearch();

  private:
    // Add private variables or methods declarations here.
    RouteModel &m_Model;
    RouteModel::Node* start_node; // Will point to the nodes in the model closest to the input nodes
    RouteModel::Node* end_node; // Will point to the nodes in the model closest to the input nodes
    float distance;// Will hold the total distance from the start node to the end node
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *current_node); // Will construct the final path to be displayed on the map
    float CalculateHValue(const RouteModel::Node* node);
    std::vector<RouteModel::Node*> open_list;
    RouteModel::Node* NextNode();
    void AddNeighbors(RouteModel::Node* current_node);
};

#endif