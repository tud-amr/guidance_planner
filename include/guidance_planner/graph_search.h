#ifndef __GRAPH_SEARCH_H__
#define __GRAPH_SEARCH_H__

#include <vector>

namespace GuidancePlanner
{

    // Used by pointer/reference
    class GeometricPath;
    struct Node;
    class Graph;

    class GraphSearch
    {

    public:
        GraphSearch();
        void Search(const Graph &graph, unsigned int max_paths, std::vector<Node *> &L, std::vector<GeometricPath> &T, const Node *goal);

    private:
        bool HasBeenVisited(const std::vector<Node *> &L, const Node *node);
    };
}
#endif // __GRAPH_SEARCH_H__