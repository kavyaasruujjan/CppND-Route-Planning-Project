#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    //Create RouteModel Nodes
    int counter = 0;
	for(Model::Node node : this->Nodes())
    {
      m_Nodes.push_back(Node(counter, this, node));
      counter++;
    }
    CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap()
{
  for(const Model::Road &road: Roads())
  {
    if (road.type != Model::Road::Type::Footway)
    {
      	for(int node_idx : Ways()[road.way].nodes)
        {
          if(node_to_road.find(node_idx) == node_to_road.end())
          {
            //set the value for the `node_idx` key to be an empty vector of `const Model::Road*` objects
            node_to_road[node_idx] = std::vector<const Model::Road *> {};
		  }
          node_to_road[node_idx].push_back(&road);
        }
    }  
  }  
}

RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices)
{
  Node *closest_node = nullptr;
  Node node;
  for(int node_index:node_indices)
  {
    node = parent_model->SNodes()[node_index];
    if(!node.visited && (this->distance(node) !=0))
    {
      if ((closest_node == nullptr) || (this->distance(node) < this->distance(*closest_node)))
        closest_node = &parent_model->SNodes()[node_index];
    }
  }
  return closest_node;
}

void RouteModel::Node::FindNeighbors()
{
  for (auto &road: parent_model->node_to_road[this->index])
  {
    RouteModel::Node* new_neighbor = FindNeighbor(parent_model->Ways()[road->way].nodes);
    if (new_neighbor)
      this->neighbors.push_back(new_neighbor);
  }
}

RouteModel::Node& RouteModel::FindClosestNode(float x, float y)
{
  RouteModel::Node input;
  input.x = x;
  input.y = y;
  float min_dist = std::numeric_limits<float>::max();
  int closest_idx;
  for (const Model::Road &road: Roads())
  {
    if (road.type != Model::Road::Type::Footway)
    {
      for (int node_idx : Ways()[road.way].nodes)
      {
        float dist = input.distance(SNodes()[node_idx]);
        if(dist < min_dist)
        {
          min_dist = dist;
          closest_idx = node_idx;
        }
      }
    }
  }
  return SNodes()[closest_idx];
}