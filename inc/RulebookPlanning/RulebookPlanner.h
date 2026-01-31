#ifndef _RULEBOOK_PLANNER_H
#define _RULEBOOK_PLANNER_H

#include "Planner.h"
#include "RulebookCost.h"
#include "Scenario.h"

template <typename State> class RulebookPlanner : public Planner<RulebookCost> {
  public:
    using WGraph = WeightedGraph<RulebookCost>;
    using WEdge = WeightedEdge<RulebookCost>;
    using WEdgePtr = std::shared_ptr<WEdge>;

    RulebookPlanner(Scenario<State> &scenario)
        : Planner(scenario.getGraph()), scenario(scenario),
          rulebook(scenario.getRulebook()), init_vid(scenario.getInit()),
          goal_vid(scenario.getGoal()) {}

    size_t getNumRules() const { return rulebook.getNumRules(); }

    std::vector<WEdgePtr> getOptimalPlan(size_t init_vid,
                                         size_t goal_vid) const override {
        assert(rulebook.isTotallyOrdered());

        bool is_first_rule = true;

        for (auto it = rulebook.begin(); it != rulebook.end(); ++it) {
            for (auto rule_index : *it) {
                const Rule &rule = rulebook.getRule(rule_index);
                if (dynamic_cast<const RuleMax *>(&rule)) {
                    auto getSubCost = [&,
                                       rule_index](const RulebookCost &cost) {
                        return cost.getSubCost<RuleCostMax>(rule_index);
                    };
                    if (is_first_rule) {
                        opt_subgraph =
                            this->graph.extractOptimalSubgraph<RuleCostMax>(
                                init_vid, goal_vid, getSubCost);
                        is_first_rule = false;
                    } else {
                        opt_subgraph.reduceToOptimalSubgraph<RuleCostMax>(
                            init_vid, goal_vid, getSubCost);
                    }
                } else if (dynamic_cast<const RuleSum *>(&rule)) {
                    auto getSubCost = [&,
                                       rule_index](const RulebookCost &cost) {
                        return cost.getSubCost<RuleCostSum>(rule_index);
                    };
                    if (is_first_rule) {
                        opt_subgraph =
                            this->graph.extractOptimalSubgraph<RuleCostSum>(
                                init_vid, goal_vid, getSubCost);
                        is_first_rule = false;
                    } else {
                        opt_subgraph.reduceToOptimalSubgraph<RuleCostSum>(
                            init_vid, goal_vid, getSubCost);
                    }
                } else {
                    throw std::runtime_error("Unsupported rule type at index " +
                                             std::to_string(rule_index));
                }
                if (debug) {
                    std::cout << "Optimal subgraph:" << std::endl;
                    scenario.printGraph(opt_subgraph);
                }
            }
        }

        return opt_subgraph.getPath(init_vid, goal_vid);
    }

    std::vector<WEdgePtr> getDijkstraPlan(size_t init_vid,
                                          size_t goal_vid) const {
        return graph.getDijkstraPath(init_vid, goal_vid);
    }

    OptimalSet<std::vector<WEdgePtr>, RulebookCost> getOptimalPlans() const {
        return getOptimalPlans(init_vid, goal_vid);
    }

    OptimalSet<std::vector<WEdgePtr>, RulebookCost>
    getOptimalPlans(size_t init_vid, size_t goal_vid) const override {
        return graph.getOptimalPaths(init_vid, goal_vid);
    }

    void setDebug(bool d) { debug = d; }

  private:
    const Scenario<State> &scenario;
    const Rulebook &rulebook;
    const size_t init_vid;
    const size_t goal_vid;
    // Make optimal_subgraph a class member to keep it alive
    // so that edges returned by getOptimalPlan will be valid after the call
    mutable WGraph opt_subgraph;
    bool debug = false;
};

#endif
