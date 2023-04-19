#include "guidance_planner/graph_search.h"

using namespace GuidancePlanner;

GraphSearch::GraphSearch() {}

void GraphSearch::Search(const Graph &graph, uint max_paths, std::vector<Node *> &L, std::vector<GeometricPath> &T, const Node *goal)
{
  // Stop if the maximum number of paths was reached
  if (T.size() >= max_paths)
    return;

  // Get the last visited node
  Node *l = L.back();

  // Check for the goal
  for (auto &neighbour : l->neighbours_)
  {
    if (neighbour->point_.Time() < l->point_.Time()) // Make edges directed forward in time
      continue;

    if (HasBeenVisited(L, neighbour))
      continue;

    // If we have reached the goal
    if (goal->id_ == neighbour->id_)
    {
      // Add this last node to the path
      L.push_back(neighbour);

      // @todo Check for topology equivalence here?

      // Add this path to the list
      T.emplace_back(L); // Create a new path object
      L.pop_back();
      break;
    }
  }

  // Add nodes to L and recursive calls
  for (auto &neighbour : l->neighbours_)
  {
    if (neighbour->point_.Time() < l->point_.Time()) // Make edges directed forward in time
      continue;

    if (HasBeenVisited(L, neighbour) || goal->id_ == neighbour->id_)
      continue;

    L.push_back(neighbour);
    Search(graph, max_paths, L, T, goal); // Recursive search
    L.pop_back();
  }
}

bool GraphSearch::HasBeenVisited(const std::vector<Node *> &L, const Node *node)
{
  for (auto &node_in_L : L)
  {
    if (node_in_L->id_ == node->id_)
      return true;
  }

  return false;
}