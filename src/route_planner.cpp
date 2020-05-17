#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y)
    : m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
    this->start_node->g_value = 0.0f;
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();
    current_node->visited = true;

    for (RouteModel::Node *neighbor : current_node->neighbors)
    {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*(neighbor->parent));
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}

bool cmp(RouteModel::Node *a, RouteModel::Node *b)
{
    return (a->g_value + a->h_value) > (b->g_value + b->h_value);
}

RouteModel::Node *RoutePlanner::NextNode()
{
    std::sort(open_list.begin(), open_list.end(), cmp);
    RouteModel::Node *nextNode = open_list.back();
    open_list.pop_back();
    return nextNode;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node *temp_node = current_node;
    while (temp_node->parent != nullptr)
    {
        distance += temp_node->distance(*(temp_node->parent));
        path_found.push_back(*temp_node);
        temp_node = temp_node->parent;
    }
    path_found.push_back(*temp_node);
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch()
{
    RouteModel::Node *current_node = nullptr;

    open_list.push_back(start_node);
    while (open_list.size() > 0)
    {
        current_node = NextNode();

        if (current_node->x == end_node->x && current_node->y == end_node->y)
        {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }

        AddNeighbors(current_node);
    }
}