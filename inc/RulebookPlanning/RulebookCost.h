#ifndef _RULEBOOKCOST_H
#define _RULEBOOKCOST_H

#include "RuleCost.h"
#include "Rulebook.h"
#include <iostream>
#include <stdexcept>

class RulebookCost {
  public:
    RulebookCost(double v = 0.0) {
        if (!rulebook) {
            throw std::runtime_error(
                "Rulebook must be initialized before RulebookCost.");
        }

        const size_t n = rulebook->getNumRules();
        cost_vector.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            cost_vector.push_back(rulebook->getRule(i).makeCost(v));
        }
    }

    void setRuleCost(size_t rule_index, double cost) {
        cost_vector[rule_index]->setValue(cost);
    }

    void setRuleEps(size_t rule_index, double eps) {
        cost_vector[rule_index]->setEps(eps);
    }

    std::shared_ptr<RuleCost> &operator[](size_t index) {
        return cost_vector[index];
    }

    const std::shared_ptr<RuleCost> &operator[](size_t index) const {
        return cost_vector[index];
    }

    size_t getNumRules() const { return cost_vector.size(); }

    const std::vector<std::shared_ptr<RuleCost>> &getCosts() const {
        return cost_vector;
    }

    static void setRulebook(const Rulebook &rb) {
        rulebook = std::make_unique<Rulebook>(rb);
    }

    RulebookCost getZero() {
        RulebookCost result;
        for (int i=0; i<(int)cost_vector.size(); ++i) {
            result.setRuleEps(i, cost_vector[i]->getEps());
        }
        return result;
    }

    template <typename SubCostType>
    SubCostType getSubCost(size_t rule_index) const {
        auto cost_ptr =
            std::dynamic_pointer_cast<SubCostType>(cost_vector[rule_index]);
        if (!cost_ptr) {
            throw std::runtime_error("SubCost type mismatch at rule index " +
                                     std::to_string(rule_index));
        }
        return *cost_ptr;
    }

    // Overloading the stream insertion operator for easy printing
    friend std::ostream &operator<<(std::ostream &os, const RulebookCost &c) {
        os << "[";

        for (auto it = c.cost_vector.begin(); it != c.cost_vector.end(); ++it) {
            os << (*it)->getValue();
            if (std::next(it) != c.cost_vector.end()) {
                os << ",";
            }
        }
        os << "]";
        return os;
    }

    RulebookCost operator+(const RulebookCost &other) const {
        if (cost_vector.size() != other.cost_vector.size()) {
            std::cerr << "This cost: " << cost_vector.size()
                      << " other: " << other.cost_vector.size() << std::endl;
            throw std::invalid_argument(
                "Cost vectors must be of the same size");
        }

        RulebookCost result;
        result.cost_vector.resize(cost_vector.size());

        for (size_t i = 0; i < cost_vector.size(); ++i) {
            result.cost_vector[i] = *cost_vector[i] + *other.cost_vector[i];
        }

        return result;
    }

    bool operator<=(const RulebookCost &other) const {
        std::unordered_set<size_t> overridden_rules;
        for (auto it = rulebook->begin(); it != rulebook->end(); ++it) {
            for (auto rule_index : *it) {
                if (overridden_rules.find(rule_index) != overridden_rules.end())
                    continue;

                const RuleCost &rule_cost = *cost_vector[rule_index];
                const RuleCost &other_cost = *other[rule_index];

                if (rule_cost > other_cost)
                    return false;
                if (rule_cost < other_cost) {
                    const auto successors = rulebook->getSuccessors(rule_index);
                    overridden_rules.insert(successors.begin(),
                                            successors.end());
                }
            }
        }

        return true;
    }

    bool operator>=(const RulebookCost &other) const { return other <= *this; }

    // Overloading comparison operators
    bool operator<(const RulebookCost &other) const {
        return (*this <= other) && !(other <= *this);
    }

    bool operator>(const RulebookCost &other) const {
        return (other <= *this) && !(*this <= other);
    }

    bool operator==(const RulebookCost &other) const {
        return (other <= *this) && (*this <= other);
    }

    bool operator<<(const RulebookCost &other) const {
        std::unordered_set<size_t> overridden_rules;
        for (auto it = rulebook->begin(); it != rulebook->end(); ++it) {
            for (auto rule_index : *it) {
                if (overridden_rules.find(rule_index) != overridden_rules.end())
                    continue;

                const RuleCost &rule_cost = *cost_vector[rule_index];
                const RuleCost &other_cost = *other[rule_index];

                if (rule_cost >> other_cost)
                    return false;
                if (rule_cost << other_cost) {
                    const auto successors = rulebook->getSuccessors(rule_index);
                    overridden_rules.insert(successors.begin(),
                                            successors.end());
                }
            }
        }

        return true;
    }

    bool operator>>(const RulebookCost &other) const {
        return other << *this;
    }

  private:
    inline static std::unique_ptr<Rulebook> rulebook = std::make_unique<Rulebook>();
    std::vector<std::shared_ptr<RuleCost>> cost_vector;
    double value;
};

template <> class std::numeric_limits<RulebookCost> {
  public:
    static constexpr bool is_specialized = true;

    static RulebookCost max() noexcept {
        return RulebookCost(std::numeric_limits<double>::max());
    }

    static RulebookCost min() noexcept {
        return RulebookCost(std::numeric_limits<double>::lowest());
    }

    static RulebookCost lowest() noexcept {
        return RulebookCost(std::numeric_limits<double>::lowest());
    }

    static constexpr bool is_signed = true;
    static constexpr bool is_integer = false;
    static constexpr bool is_exact = false;
    static constexpr bool has_infinity = false;
    static constexpr bool has_quiet_NaN = false;
    static constexpr bool has_signaling_NaN = false;
    static constexpr std::float_denorm_style has_denorm = std::denorm_absent;
    static constexpr bool has_denorm_loss = false;
    static constexpr std::float_round_style round_style = std::round_to_nearest;
    static constexpr bool is_iec559 = false;
    static constexpr bool is_bounded = true;
    static constexpr bool is_modulo = false;
};

// Make RulebookCost hashable
namespace std {
template <> struct hash<RulebookCost> {
    std::size_t operator()(const RulebookCost &cost) const {
        std::size_t seed = 0;
        for (const auto &rule_cost : cost.getCosts()) {
            // Hash using getValue() (same logic for RuleCostMax/RuleCostSum)
            std::size_t h = std::hash<double>()(rule_cost->getValue());

            // Combine hashes (boost-like)
            seed ^= h + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};
} // namespace std

#endif
