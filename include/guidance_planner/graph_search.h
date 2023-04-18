#ifndef __GRAPH_SEARCH_H__
#define __GRAPH_SEARCH_H__

#include "guidance_planner/paths.h"

namespace GuidancePlanner
{

    class GraphSearch
    {

    public:
        GraphSearch();
        void Search(const Graph &graph, uint max_paths, std::vector<Node *> &L, std::vector<GeometricPath> &T, const Node *goal);

    private:
        bool HasBeenVisited(const std::vector<Node *> &L, const Node *node);
    };
}
#endif // __GRAPH_SEARCH_H__