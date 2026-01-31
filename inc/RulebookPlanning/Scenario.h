#ifndef _SCENARIO_H
#define _SCENARIO_H

#include "OptimalSet.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "WeightedGraph.h"

template <typename State> class Scenario {
  public:
    using WEdge = WeightedEdge<RulebookCost>;
    using WEdgePtr = std::shared_ptr<WEdge>;

    Scenario() = default;

    Scenario(const State &sinit, const State &sgoal)
        : init_state(sinit), goal_state(sgoal) {}

    virtual ~Scenario() =
        default; // Virtual destructor for proper cleanup of derived classes.

    size_t getGraphSize() const { return graph.getNumVertices(); }

    size_t getNumRules() const { return rulebook.getNumRules(); }

    Rulebook &getRulebook() { return rulebook; }
    const Rulebook &getRulebook() const { return rulebook; }

    WeightedGraph<RulebookCost> &getGraph() { return graph; }
    const WeightedGraph<RulebookCost> &getGraph() const { return graph; }

    size_t getInit() const { return init_vid; }
    size_t getGoal() const { return goal_vid; }

    void setDebug(bool d) { debug = d; }

    virtual void printGraph(const WeightedGraph<RulebookCost> &graph) const {
        if (vid2state.size() == 0) {
            graph.display();
            return;
        }

        for (const auto &pair : graph.getVertices()) {
            std::cout << "  " << vid2state.at(pair.first) << " ->";
            for (size_t i = 0; i < pair.second->out_edges.size(); ++i) {
                auto edge = pair.second->out_edges[i];
                auto weighted_edge = std::dynamic_pointer_cast<WEdge>(edge);
                std::cout << " (" << vid2state.at(weighted_edge->to->vid) << ","
                          << weighted_edge->cost << ")";
            }
            std::cout << std::endl;
        }
    }

  protected:
    Rulebook rulebook;
    WeightedGraph<RulebookCost> graph;
    // initial and goal states
    State init_state;
    State goal_state;
    // vertex id of init_state and goal_state
    size_t init_vid;
    size_t goal_vid;
    // map between vertex ID and state
    std::unordered_map<size_t, State> vid2state;
    std::unordered_map<State, size_t> state2vid;
    bool debug = false;

    // Pure virtual function to be implemented by derived classes.
    virtual void buildGraph() = 0;
    virtual void buildRulebook() = 0;

    void setup() {
        buildRulebook();
        RulebookCost::setRulebook(rulebook);
        buildGraph();
    }

    size_t addState(const State &state) {
        const size_t vid = graph.addVertex();
        vid2state.insert({vid, state});
        state2vid.insert({state, vid});
        if (state == init_state)
            init_vid = vid;
        if (state == goal_state)
            goal_vid = vid;
        return vid;
    }

    void displayRulebook() {
        std::cout << "Rulebook:" << std::endl;
        rulebook.display();

        std::cout << "Order: ";
        for (auto it = rulebook.begin(); it != rulebook.end(); ++it) {
            std::cout << "{ ";
            for (auto elem : *it) {
                std::cout << elem << " ";
            }
            std::cout << "} ";
        }
        std::cout << std::endl;

        // Successors
        std::cout << "Successors:" << std::endl;
        for (size_t i = 0; i < 4; ++i) {
            std::cout << "    " << i << ": ";
            const auto successors = rulebook.getSuccessors(i);
            for (auto j : successors) {
                std::cout << j << " ";
            }
            std::cout << std::endl;
        }
    }
};

#endif
