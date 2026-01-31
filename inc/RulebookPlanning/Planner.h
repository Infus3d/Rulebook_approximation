#ifndef _PLANNER_H
#define _PLANNER_H

#include "WeightedGraph.h"

template <typename CostType> class Planner {
  public:
    using WEdge = WeightedEdge<CostType>;
    using WEdgePtr = std::shared_ptr<WEdge>;

    Planner(WeightedGraph<CostType> &graph) : graph(graph) {}

    size_t getGraphSize() const { return graph.getNumVertices(); }

    std::vector<WEdgePtr> getPlan(size_t init_vid, size_t goal_vid) const {
        return graph.getPath(init_vid, goal_vid);
    }

    virtual std::vector<WEdgePtr> getOptimalPlan(size_t init_vid,
                                                 size_t goal_vid) const = 0;

    virtual OptimalSet<std::vector<WEdgePtr>, CostType>
    getOptimalPlans(size_t init_vid, size_t goal_vid) const = 0;

  protected:
    WeightedGraph<CostType> &graph;
};

#endif
