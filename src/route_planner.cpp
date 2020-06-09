#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

 // Find the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);
}


// Calculate the heuristic value from current to the goal node
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{
    return node->distance(*end_node);
}



void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    // Add all unvisited neighbors 
    (*current_node).FindNeighbors();

    for (auto next:current_node->neighbors)
    {
        next->parent = current_node;
        next->g_value = current_node->g_value + current_node->distance(*next);
        next->h_value = CalculateHValue(next);

        // push back all the neighbors in an openlist
        open_list.emplace_back(next);

        // mark visited nodes
        next->visited = true;
    }
    
}


RouteModel::Node *RoutePlanner::NextNode() 
{
    // using sort function to sort openlist in increasing order of f-value
    std::sort(open_list.begin(),open_list.end(),[](const auto &_1st, const auto &_2nd)
    {
        return (_1st->h_value + _1st->g_value < _2nd->h_value + _2nd->g_value);
    });

    RouteModel::Node *lowest_node = open_list.front();

    // once the reference is saved remove it from the open_list and return the reference
    open_list.erase(open_list.begin());
    return lowest_node;


}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node->parent != nullptr)
    {
        path_found.emplace_back(*current_node);
        const RouteModel::Node parent = *(current_node->parent);
        distance += current_node->distance(parent);
        current_node = current_node->parent;
    }
    path_found.emplace_back(*current_node);

    /*As we traversed from goal to start we have to reverse it in order to get proper path from start to 
    goal*/

    std::reverse(path_found.begin(),path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm here.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
    open_list.emplace_back(start_node);

    while (!open_list.empty())
    {
        current_node = NextNode();
        if (current_node->distance(*end_node) == 0)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        else
        {
            AddNeighbors(current_node);
        }

    }

}