#ifndef _RULEBOOK_H
#define _RULEBOOK_H

#include <iostream>
#include <iterator>
#include <unordered_set>
#include <vector>

#include "Graph.h"
#include "Rule.h"

struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const {
        return std::hash<T1>()(p.first) ^ (std::hash<T2>()(p.second) << 1);
    }
};

class Rulebook {
  public:
    class Iterator {
      public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = std::unordered_set<size_t>;
        using difference_type = std::ptrdiff_t;
        using pointer = const std::unordered_set<size_t> *;
        using reference =
            const std::unordered_set<size_t> &; // Use const reference

        Iterator(const std::vector<size_t> &sorted_indices,
                 const std::vector<std::unordered_set<size_t>> &sets,
                 std::size_t index)
            : sorted_indices(sorted_indices), sets(sets), index(index) {}

        // Iterator operations
        reference operator*() const { return sets[sorted_indices[index]]; }

        pointer operator->() const { return &sets[sorted_indices[index]]; }

        Iterator &operator++() {
            ++index;
            return *this;
        }

        Iterator operator++(int) {
            Iterator temp = *this;
            ++(*this);
            return temp;
        }

        bool operator==(const Iterator &other) const {
            return index == other.index;
        }

        bool operator!=(const Iterator &other) const {
            return !(*this == other);
        }

      private:
        const std::vector<size_t> &sorted_indices;
        const std::vector<std::unordered_set<size_t>> &sets;
        std::size_t index;
    };

  public:
    Rulebook() : is_built(false) {}

    Iterator begin() const {
        return Iterator(sorted_rule_set, quotient_rule_set, 0);
    }

    Iterator end() const {
        return Iterator(sorted_rule_set, quotient_rule_set,
                        sorted_rule_set.size());
    }

    size_t addRule(const Rule &rule) {
        size_t rid = rules.size();
        rules.push_back(rule.clone());
        quotient_rule_set.emplace_back(std::unordered_set<size_t>{rid});
        return rid;
    }

    const Rule &getRule(size_t id) const { return *rules[id]; }

    void setEquivalentClasses(
        const std::vector<std::unordered_set<size_t>> &eq_classes) {
        // TO FIX: If this function is called after addGTRelation, the priority
        // will be broken
        if (!rule_edges.empty()) {
            throw std::logic_error("Cannot set equivalence classes after GT "
                                   "relations have been added.");
        }
        quotient_rule_set = completeQuotientRuleSet(eq_classes);
        is_built = false;
    }

    void addGTRelation(size_t from_rule, size_t to_rule) {
        int from_rule_index = getQuotientIndex(from_rule);
        if (from_rule_index < 0) {
            throw std::invalid_argument("from_rule " +
                                        std::to_string(from_rule) +
                                        " not found in rulebook.");
        }

        int to_rule_index = getQuotientIndex(to_rule);
        if (to_rule_index < 0) {
            throw std::invalid_argument("to_rule " + std::to_string(to_rule) +
                                        " not found in rulebook.");
        }

        if (from_rule_index == to_rule_index) {
            throw std::invalid_argument("rules are of the same rank ");
        }

        std::pair<size_t, size_t> backward_edge = {to_rule_index,
                                                   from_rule_index};

        if (rule_edges.find(backward_edge) != rule_edges.end()) {
            throw std::invalid_argument("Rule " + std::to_string(to_rule) +
                                        " is of higher rank than " +
                                        std::to_string(from_rule));
        }

        std::pair<size_t, size_t> edge =
            std::make_pair((size_t)from_rule_index, (size_t)to_rule_index);
        rule_edges.insert(edge);
        is_built = false;
    }

    size_t getNumRules() const { return rules.size(); }

    bool build() {
        if (!isQuotientRuleSetComplete()) {
            throw std::logic_error("Cannot build because the set of equivalent "
                                   "classes is not complete.");
        }

        rule_graph.clear();
        for (size_t vid = 0; vid < quotient_rule_set.size(); ++vid) {
            rule_graph.addVertex(vid);
        }
        for (const auto &edge : rule_edges) {
            rule_graph.addEdge(edge.first, edge.second);
        }
        sorted_rule_set = rule_graph.topologicalSort();

        for (size_t vid = 0; vid < quotient_rule_set.size(); ++vid) {
            const auto vertex = rule_graph.getVertex(vid);

            // Check that rule_graph is a DAG
            for (auto edge : vertex->out_edges) {
                auto it_vid =
                    find(sorted_rule_set.begin(), sorted_rule_set.end(), vid);
                auto it_suc = find(sorted_rule_set.begin(),
                                   sorted_rule_set.end(), edge->to->vid);
                // If parent vertex does not appear first, then there is a cycle
                if (it_suc - it_vid < 0) {
                    return false;
                }
            }

            // Construct rule_successors
            const auto successors = rule_graph.findSuccessors(vid);
            for (const auto &rule : quotient_rule_set[vid]) {
                for (const auto &suc_vid : successors) {
                    rule_successors[rule].insert(
                        quotient_rule_set[suc_vid].begin(),
                        quotient_rule_set[suc_vid].end());
                }
            }
        }
        is_built = true;

        return true;
    }

    std::unordered_set<size_t> getSuccessors(size_t rule) const {
        assert(is_built);
        auto it = rule_successors.find(rule);
        if (it != rule_successors.end()) {
            return it->second;
        } else {
            return {}; // Return an empty unordered_map
        }
    }

    bool isTotallyOrdered() const {
        assert(is_built);
        if (!isAllSingletons())
            return false;
        return rule_graph.isTotallyOrdered();
    }

    bool isAllSingletons() const {
        for (const auto &eq_class : quotient_rule_set) {
            if (eq_class.size() != 1)
                return false;
        }
        return true;
    }

    // Display the rulebook
    virtual void display() const {
        std::cout << "Rules: ";
        for (size_t id = 0; id < this->getNumRules(); ++id) {
            if (id > 0)
                std::cout << ", ";
            std::cout << "(" << id << ", " << *rules[id] << ")";
        }
        std::cout << std::endl;
        std::cout << "Equivalent class: ";
        for (const auto &rule_set : quotient_rule_set) {
            std::cout << "{ ";
            for (const auto rule : rule_set) {
                std::cout << rule << " ";
            }
            std::cout << "} ";
        }
        std::cout << std::endl;
        std::cout << "Edges: ";
        for (const auto &edge : rule_edges) {
            std::cout << "(" << edge.first << ", " << edge.second << ") ";
        }
        std::cout << std::endl;
        if (is_built) {
            std::cout << "Graph:" << std::endl;
            rule_graph.display();
        }
    }

  private:
    int getQuotientIndex(size_t rule_index) const {
        for (size_t i = 0; i < quotient_rule_set.size(); ++i) {
            if (quotient_rule_set[i].find(rule_index) !=
                quotient_rule_set[i].end())
                return i;
        }
        return -1;
    }

    bool isQuotientRuleSetComplete() const {
        std::unordered_set<size_t> seen;

        // Collect all rule indices that appear in quotient_rule_set
        for (const auto &eq_class : quotient_rule_set) {
            for (size_t id : eq_class) {
                if (id >= rules.size()) {
                    std::cerr << "Invalid rule ID: " << id << " (out of bounds)"
                              << std::endl;
                    return false;
                }
                if (!seen.insert(id).second) {
                    std::cerr
                        << "Duplicate rule ID in quotient_rule_set: " << id
                        << std::endl;
                    return false;
                }
            }
        }

        // Check that all rules are present
        if (seen.size() != rules.size()) {
            std::cerr << "Missing rules in quotient_rule_set." << std::endl;
            return false;
        }

        return true;
    }

    std::vector<std::unordered_set<size_t>> completeQuotientRuleSet(
        const std::vector<std::unordered_set<size_t>> &eq_classes) {
        std::unordered_set<size_t> seen_rids;

        // Collect seen rule IDs
        for (const auto &eq_class : eq_classes) {
            for (size_t rid : eq_class) {
                if (rid >= this->getNumRules()) {
                    throw std::invalid_argument("Rule ID " +
                                                std::to_string(rid) +
                                                " not found in rulebook.");
                }
                if (!seen_rids.insert(rid).second) {
                    throw std::invalid_argument("Duplicate rule ID " +
                                                std::to_string(rid) +
                                                " in equivalence classes.");
                }
            }
        }

        // Add missing rule IDs as singleton equivalence classes
        std::vector<std::unordered_set<size_t>> completed_classes = eq_classes;
        for (size_t rid = 0; rid < this->getNumRules(); ++rid) {
            if (seen_rids.find(rid) == seen_rids.end()) {
                completed_classes.emplace_back(std::unordered_set<size_t>{rid});
            }
        }

        return completed_classes;
    }

    std::vector<std::shared_ptr<Rule>> rules;
    std::vector<std::unordered_set<size_t>> quotient_rule_set;
    std::unordered_set<std::pair<size_t, size_t>, PairHash> rule_edges;

    Graph rule_graph;
    std::vector<size_t> sorted_rule_set;
    std::unordered_map<size_t, std::unordered_set<size_t>> rule_successors;
    bool is_built;
};

#endif
