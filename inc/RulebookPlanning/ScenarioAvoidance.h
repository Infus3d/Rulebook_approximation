#ifndef _SCENARIO_AVOIDANCE_H
#define _SCENARIO_AVOIDANCE_H

#include <cmath>

#include "GridWorld.h"
#include "Rule.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "Scenario.h"

class ScenarioAvoidance : public Scenario<DiscreteState2D> {
  public:
    ScenarioAvoidance(size_t refinement_option = 0)
        : Scenario<DiscreteState2D>(DiscreteState2D(1, 1),
                                    DiscreteState2D(7, 1)),
          refinement_option(refinement_option) {
        setup();
    }

  private:
    size_t refinement_option;
    const size_t num_x = 7;
    const size_t num_y = 5;
    const double dx = 2.0;
    const double dy = 2.0;
    // IDs of the rules
    size_t rid_blockage, rid_lane, rid_clearance, rid_length;

    void buildRulebook() override {
        const RuleSum r0("blockage");
        const RuleSum r1("lane");
        const RuleSum r2("clearance");
        const RuleSum r3("length");

        rid_blockage = rulebook.addRule(r0);
        rid_lane = rulebook.addRule(r1);
        rid_clearance = rulebook.addRule(r2);
        rid_length = rulebook.addRule(r3);

        rulebook.setEquivalentClasses(
            {{rid_blockage}, {rid_lane}, {rid_clearance}, {rid_length}});

        rulebook.addGTRelation(rid_blockage, rid_lane);
        rulebook.addGTRelation(rid_blockage, rid_clearance);
        rulebook.addGTRelation(rid_lane, rid_length);
        rulebook.addGTRelation(rid_clearance, rid_length);

        if (refinement_option == 1)
            rulebook.addGTRelation(rid_lane, rid_clearance);

        if (refinement_option == 2)
            rulebook.addGTRelation(rid_clearance, rid_lane);

        rulebook.build();

        if (debug)
            displayRulebook();
    }

    void buildGraph() override {
        size_t obstacle_x_min = floor(static_cast<double>(num_x) / 2) - 1;
        size_t obstacle_x_max = ceil(static_cast<double>(num_x) / 2);
        double diag_length = sqrt(dx * dx + dy * dy);

        for (size_t row = 0; row < num_y; ++row) {
            for (size_t col = 0; col < num_x; ++col) {
                addState(getStateAt(row, col));
            }
        }

        for (size_t row = 0; row < num_y; ++row) {
            for (size_t col = 0; col < num_x; ++col) {
                const size_t vertex1 = state2vid.at(getStateAt(row, col));
                if (col + 1 < num_x) {
                    // Horizontal edge
                    size_t vertex2 = state2vid.at(getStateAt(row, col + 1));
                    const RulebookCost cost1 =
                        getCost(row, col, row, col + 1, obstacle_x_min,
                                obstacle_x_max, dx);
                    graph.addEdge(vertex1, vertex2, cost1, 0);
                    // Diagonal edge up
                    if (row + 1 < num_y) {
                        vertex2 = state2vid.at(getStateAt(row + 1, col + 1));
                        const RulebookCost cost2 =
                            getCost(row, col, row + 1, col + 1, obstacle_x_min,
                                    obstacle_x_max, diag_length);
                        graph.addEdge(vertex1, vertex2, cost2, 1);
                    }
                    // Diagonal edge down
                    if (row > 0) {
                        vertex2 = state2vid.at(getStateAt(row - 1, col + 1));
                        const RulebookCost cost3 =
                            getCost(row, col, row - 1, col + 1, obstacle_x_min,
                                    obstacle_x_max, diag_length);
                        graph.addEdge(vertex1, vertex2, cost3, 2);
                    }
                }
            }
        }
        // Display the graph
        if (debug) {
            std::cout << "Graph: " << std::endl;
            graph.display();
        }
    }

    DiscreteState2D getStateAt(size_t row, size_t col) {
        return DiscreteState2D(col + 1, row + 1);
    }

    bool isInObstacle(size_t row, size_t col, size_t obstacle_x_min,
                      size_t obstacle_x_max) {
        if (row > 0)
            return false;
        return (col >= obstacle_x_min && col <= obstacle_x_max);
    }

    bool isNearObstacle(size_t row, size_t col, size_t obstacle_x_min,
                        size_t obstacle_x_max) {
        if (row > 1)
            return false;
        return (col >= obstacle_x_min && col <= obstacle_x_max);
    }

    bool isNotInLane(size_t row) { return (row > 1); }

    RulebookCost getCost(size_t prev_row, size_t prev_col, size_t next_row,
                         size_t next_col, size_t obstacle_x_min,
                         size_t obstacle_x_max, double length) {
        double blockage_cost =
            isInObstacle(prev_row, prev_col, obstacle_x_min, obstacle_x_max)
                ? 1.0
                : 0.0;
        if (isInObstacle(next_row, next_col, obstacle_x_min, obstacle_x_max))
            blockage_cost = blockage_cost + 1.0;
        double lane_cost = isNotInLane(prev_row) ? 1.0 : 0.0;
        if (isNotInLane(next_row))
            lane_cost = lane_cost + 1.0;
        double clearance_cost =
            isNearObstacle(prev_row, prev_col, obstacle_x_min, obstacle_x_max)
                ? 1.0
                : 0.0;
        if (isNearObstacle(next_row, next_col, obstacle_x_min, obstacle_x_max))
            clearance_cost = clearance_cost + 1.0;
        RulebookCost cost;
        cost.setRuleCost(rid_blockage, blockage_cost);
        cost.setRuleCost(rid_lane, lane_cost);
        cost.setRuleCost(rid_clearance, clearance_cost);
        cost.setRuleCost(rid_length, length);
        return cost;
    }
};

#endif
