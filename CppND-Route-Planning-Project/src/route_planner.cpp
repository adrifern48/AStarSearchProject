#include "route_planner.h"
#include <algorithm>

using namespace std;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    if (node != nullptr && end_node != nullptr) {
        return this->end_node->distance(*node);
    }
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    if (current_node != nullptr) {
        current_node->FindNeighbors();
        for (auto &neighbor : current_node->neighbors) {
            if (neighbor->x < 0 || neighbor->y < 0 || neighbor->visited)
                continue;
            neighbor->parent = current_node;
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
            this->open_list.push_back(neighbor);
            neighbor->visited = true;
        }
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    if (!open_list.empty()) {

        // sorted in descending order
        sort(open_list.begin(), open_list.end(), [](const RouteModel::Node* a, const RouteModel::Node* b) {
            float f1 = a->h_value + a->g_value;
            float f2 = b->h_value + b->g_value;
            return f1 > f2;
        });

        RouteModel::Node *next = open_list.back();
        open_list.pop_back();
        return next;
    }

    return nullptr;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    vector<RouteModel::Node> path_found;

    path_found.push_back(*current_node);

    while (current_node->parent != nullptr && current_node != start_node) {
        path_found.push_back(*current_node->parent);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }

    reverse(path_found.begin(), path_found.end()); // Returning in correct order

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    AddNeighbors(start_node);
    start_node->visited = true;
    
    while (open_list.size() > 0) {
        current_node = NextNode();

        // check if we're done
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        AddNeighbors(current_node);
    }

    // no path found
}