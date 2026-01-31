#ifndef _SCENARIO_GRID_H
#define _SCENARIO_GRID_H

#include <cmath>

#include "GridWorld.h"
#include "Rule.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "Scenario.h"
#include "WeightedGraph.h"

class ScenarioGrid : public Scenario<DiscreteProductState2D> {
  public:
    using WEdge = WeightedEdge<RulebookCost>;
    using WEdgePtr = std::shared_ptr<WEdge>;

    ScenarioGrid(int xmax = 5, int ymax = 5)
        : Scenario<DiscreteProductState2D>(
              DiscreteProductState2D(0, 0, 0),
              DiscreteProductState2D(xmax - 1, ymax, 1)),
          world(1, xmax, 1, ymax), p1_region(2, xmax - 2, 2, ymax - 3),
          p2_region(xmax - 1, xmax - 1, ymax - 1, ymax - 1),
          obs_region(1, xmax - 2, ymax - 1, ymax - 1), obs_clearance(2),
          qmin(0), qmax(1), sinit(1, 2) {
        assert(init_state.q >= qmin);
        assert(init_state.q <= qmax);
        assert(goal_state.q >= qmin);
        assert(goal_state.q <= qmax);
        assert(world.contain(sinit));
        assert(world.contain(goal_state));
        setup();
    }

    std::vector<DiscreteProductState2D>
    getPStatePath(const std::vector<WEdgePtr> &path) const {
        std::vector<DiscreteProductState2D> pstate_path;
        if (path.empty())
            return pstate_path;

        pstate_path.push_back(vid2state.at(path[0]->from->vid));

        for (const auto &edge : path) {
            pstate_path.push_back(vid2state.at(edge->to->vid));
        }

        return pstate_path;
    }

    void printGraph(const WeightedGraph<RulebookCost> &graph) const override {
        for (const auto &pair : graph.getVertices()) {
            auto ps1 = vid2state.at(pair.first);
            for (size_t i = 0; i < pair.second->out_edges.size(); ++i) {
                std::cout << "  \\draw[->, thick] (v-" << ps1.x << "-" << ps1.y
                          << "-" << ps1.q << ") -- ";
                auto edge = pair.second->out_edges[i];
                auto weighted_edge = std::dynamic_pointer_cast<WEdge>(edge);
                auto ps2 = vid2state.at(weighted_edge->to->vid);
                std::cout << "(v-" << ps2.x << "-" << ps2.y << "-" << ps2.q
                          << ");" << std::endl;
            }
        }
    }

  private:
    // Reserving (x,y) = (0,0) for state marked as init when constructing
    // the product automaton so the world shouldn't contain this.
    const DiscreteRegion2D world;
    const DiscreteRegion2D p1_region;
    const DiscreteRegion2D p2_region;
    const DiscreteRegion2D obs_region;
    const double obs_clearance;
    const int qmin, qmax;
    // Available actions
    const std::vector<DiscreteAction2D> actions = {
        DiscreteAction2D(0, 1), DiscreteAction2D(1, 0), DiscreteAction2D(0, -1),
        DiscreteAction2D(-1, 0)};
    // IDs of the rules
    size_t rid_ltl, rid_obs, rid_len;
    // initial and goal states in the world
    const DiscreteState2D sinit;
    // all vids for product states (sinit, q)
    std::vector<size_t> sinit_vids;
    // all vids for product states (init, q)
    std::vector<size_t> init_vids;

    void buildRulebook() override {
        const RuleSum r0("ltl");
        const RuleMax r1("obs");
        const RuleSum r2("len");

        rid_ltl = rulebook.addRule(r0);
        rid_obs = rulebook.addRule(r1);
        rid_len = rulebook.addRule(r2);

        rulebook.addGTRelation(rid_ltl, rid_obs);
        rulebook.addGTRelation(rid_obs, rid_len);

        rulebook.build();

        if (debug)
            displayRulebook();
    }

    void buildGraph() override {
        // init state (0,0)
        for (int q = qmin; q <= qmax; ++q) {
            const DiscreteProductState2D pstate(0, 0, q);
            const size_t vid = addState(pstate);
            init_vids.push_back(vid);
        }

        // (x, y, q) for (x, y) in world
        for (int x = world.xmin; x <= world.xmax; ++x) {
            for (int y = world.ymin; y <= world.ymax; ++y) {
                for (int q = qmin; q <= qmax; ++q) {
                    const DiscreteProductState2D pstate(x, y, q);
                    const size_t vid = addState(pstate);
                    if (x == sinit.x && y == sinit.y)
                        sinit_vids.push_back(vid);
                }
            }
        }

        // Connections from (init, q)
        for (auto vertex1 : init_vids) {
            for (auto vertex2 : sinit_vids) {
                const auto ps1 = vid2state.at(vertex1);
                const auto ps2 = vid2state.at(vertex2);
                graph.addEdge(vertex1, vertex2, getCostFromInit(ps1, ps2));
            }
        }

        // Other connections
        for (int x = world.xmin; x <= world.xmax; ++x) {
            for (int y = world.ymin; y <= world.ymax; ++y) {
                for (int q = qmin; q <= qmax; ++q) {
                    const DiscreteProductState2D ps1(x, y, q);
                    addConnectionFrom(ps1);
                }
            }
        }
    }

    void addConnectionFrom(const DiscreteProductState2D &ps1) {
        size_t vertex1 = state2vid.at(ps1);
        for (const auto &action : actions) {
            DiscreteProductState2D ps2 = ps1 + action;
            if (!world.contain(ps2))
                continue;
            for (int q = 0; q <= 1; ++q) {
                ps2.q = q;
                size_t vertex2 = state2vid.at(ps2);
                graph.addEdge(vertex1, vertex2, getCost(ps1, ps2));
            }
        }
    }

    RulebookCost getCostFromInit(const DiscreteProductState2D &ps1,
                                 const DiscreteProductState2D &ps2) {
        RulebookCost cost;
        cost.setRuleCost(rid_ltl, getLTLCost(ps1, ps2));
        cost.setRuleCost(rid_obs, getObsCost(ps1, ps2));
        cost.setRuleCost(rid_len, 0.0);
        return cost;
    }

    RulebookCost getCost(const DiscreteProductState2D &ps1,
                         const DiscreteProductState2D &ps2) {
        RulebookCost cost;
        cost.setRuleCost(rid_ltl, getLTLCost(ps1, ps2));
        cost.setRuleCost(rid_obs, getObsCost(ps1, ps2));
        cost.setRuleCost(rid_len, getLength(ps1, ps2));
        return cost;
    }

    double getLTLCost(const DiscreteProductState2D &ps1,
                      const DiscreteProductState2D &ps2) {
        // q0 -> q0
        if (ps1.q == 0 && ps2.q == 0) {
            return p1_region.contain(ps2) ? 1.0 : 0.0;
        }
        if (ps1.q == 0 && ps2.q == 1) {
            return p2_region.contain(ps2) ? 0.0 : 1.0;
        }
        if (ps1.q == 1 && ps2.q == 1) {
            return 0.0;
        }
        return 1.0;
    }

    double getObsCost(const DiscreteProductState2D &ps1,
                      const DiscreteProductState2D &ps2) {
        const double min_dist =
            std::min(obs_region.distanceTo(ps1), obs_region.distanceTo(ps2));
        return std::max(0.0, obs_clearance - min_dist);
    }

    double getLength(const DiscreteProductState2D &ps1,
                     const DiscreteProductState2D &ps2) {
        return std::sqrt((ps1.x - ps2.x) * (ps1.x - ps2.x) +
                         (ps1.y - ps2.y) * (ps1.y - ps2.y));
    }
};

#endif
