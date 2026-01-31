#ifndef _SCENARIO_RANDOM_H
#define _SCENARIO_RANDOM_H

#include <random>

#include "GridWorld.h"
#include "Rule.h"
#include "Rulebook.h"
#include "RulebookCost.h"
#include "Scenario.h"

class ScenarioRandom : public Scenario<DiscreteState2D> {
  public:
    ScenarioRandom() {
        num_x = getRandomInt(5, 15);
        num_y = getRandomInt(5, 15);
        num_rules = getRandomInt(2, 10);
        setRandomInitAndGoal();
        std::cout << "  Graph size: (" << num_x << ", " << num_y << ")"
                  << std::endl;
        setup();
    }

    ScenarioRandom(size_t num_x, size_t num_y, size_t num_rules)
        : num_x(num_x), num_y(num_y), num_rules(num_rules) {
        setRandomInitAndGoal();
        setup();
    }

  private:
    size_t num_x;
    size_t num_y;
    size_t num_rules;

    void setRandomInitAndGoal() {
        init_vid = getRandomInt(0, (num_x * num_y) - 1);
        goal_vid = getRandomInt(0, (num_x * num_y) - 1);
    }

    size_t getRandomInt(size_t min_value, size_t max_value) {
        std::random_device rd;  // Obtain a random number from hardware
        std::mt19937 gen(rd()); // Seed the generator
        std::uniform_int_distribution<size_t> distr(
            min_value, max_value); // Define the range
        return distr(gen);
    }

    double getRandomDouble(double min_value, double max_value) {
        // Random number generator
        std::random_device rd;  // Obtain a random number from hardware
        std::mt19937 gen(rd()); // Seed the generator
        std::uniform_real_distribution<double> distr(
            min_value, max_value); // Define the range

        return distr(gen);
    }

    // Function to generate a random partition of set S = {0, ..., n}
    std::vector<std::unordered_set<size_t>> getRandomSubsets(int n) {
        std::vector<int> S(n);
        std::iota(S.begin(), S.end(),
                  0); // Fill S with values {0, 1, 2, ..., n-1}

        // Random number generator
        std::random_device rd;
        std::mt19937 gen(rd());

        // Shuffle the set S
        std::shuffle(S.begin(), S.end(), gen);

        // Randomly decide the number of subsets (1 to n)
        std::uniform_int_distribution<int> subset_count_distr(1, n);
        int subset_count = subset_count_distr(gen);

        // Create a random partition of the set S
        std::vector<std::unordered_set<size_t>> subsets(subset_count);
        int current_subset = 0;

        for (int i = 0; i < n; ++i) {
            subsets[current_subset].insert(S[i]);
            current_subset = (current_subset + 1) % subset_count;
        }

        return subsets;
    }

    RulebookCost getRandomCost() {
        RulebookCost cost;
        for (size_t rule_index = 0; rule_index < rulebook.getNumRules();
             ++rule_index)
            cost.setRuleCost(rule_index, getRandomDouble(0, 10.0));
        return cost;
    }

    void buildRulebook() override {
        for (size_t i = 0; i < num_rules; ++i) {
            const RuleSum r("r" + std::to_string(i));
            rulebook.addRule(r);
        }

        const std::vector<std::unordered_set<size_t>> equiv_rules_set =
            getRandomSubsets(num_rules);
        std::cout << "  Rules: ";
        for (const auto &set : equiv_rules_set) {
            std::cout << "{";
            for (auto it = set.begin(); it != set.end(); ++it) {
                std::cout << *it;
                if (std::next(it) != set.end())
                    std::cout << ", ";
            }
            std::cout << "},";
        }
        std::cout << std::endl;

        rulebook.setEquivalentClasses(equiv_rules_set);
        const size_t num_equiv_rules = equiv_rules_set.size();

        if (num_equiv_rules >= 2) {
            const size_t num_relations =
                getRandomInt(0, num_equiv_rules * (num_equiv_rules - 1) / 2.0);

            for (size_t rel_ind = 0; rel_ind < num_relations; ++rel_ind) {
                const size_t from = getRandomInt(0, num_equiv_rules - 2);
                const size_t to = getRandomInt(from + 1, num_equiv_rules - 1);
                if (from != to) {
                    auto it1 = equiv_rules_set[from].begin();
                    const size_t from_rule = *it1;
                    auto it2 = equiv_rules_set[to].begin();
                    const size_t to_rule = *it2;
                    rulebook.addGTRelation(from_rule, to_rule);
                }
            }
        }

        rulebook.build();
        if (debug)
            displayRulebook();
    }

    void buildGraph() override {
        for (size_t vid = 0; vid < num_x * num_y; ++vid) {
            graph.addVertex(vid);
        }

        for (size_t row = 0; row < num_y; ++row) {
            for (size_t col = 0; col < num_x; ++col) {
                size_t start_vertex = num_x * row + col;
                if (col + 1 < num_x) {
                    const RulebookCost cost = getRandomCost();
                    graph.addEdge(start_vertex, start_vertex + 1, cost, 0);
                }
                if (row + 1 < num_y) {
                    const RulebookCost cost = getRandomCost();
                    graph.addEdge(start_vertex, start_vertex + num_x, cost, 1);
                }
                if (col >= 1) {
                    const RulebookCost cost = getRandomCost();
                    graph.addEdge(start_vertex, start_vertex - 1, cost, 2);
                }
                if (row >= 1) {
                    const RulebookCost cost = getRandomCost();
                    graph.addEdge(start_vertex, start_vertex - num_x, cost, 3);
                }
            }
        }
        // Display the graph
        if (debug) {
            std::cout << "Graph: " << std::endl;
            graph.display();
        }
    }
};

#endif
