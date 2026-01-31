#ifndef _RULE_H
#define _RULE_H

#include "RuleCost.h"
#include <iostream>

class Rule {
  public:
    Rule(const std::string &name) : name(name) {}

    virtual ~Rule() = default;

    // This is to allow allow polymorphic copying to
    // allow Rulebook to store its derived classes in a single container.
    virtual std::shared_ptr<Rule> clone() const = 0;

    // This is to allow RulebookCost to have a mix of RuleCostMax and
    // RuleCostSum cost
    virtual std::shared_ptr<RuleCost> makeCost(double value) const = 0;

    // Overloading the stream insertion operator for easy printing
    friend std::ostream &operator<<(std::ostream &os, const Rule &rule) {
        os << rule.name;
        return os;
    }

    /*
    // Overloading comparison operators
    bool operator<(const Rule &other) const { return value < other.value; }

    bool operator>(const Rule &other) const { return value > other.value; }

    bool operator<=(const Rule &other) const { return value <= other.value; }

    bool operator>=(const Rule &other) const { return value >= other.value; }

    bool operator==(const Rule &other) const { return value == other.value; }
    */

    const std::string &getName() const { return name; }

  protected:
    std::string name;
};

class RuleMax : public Rule {
  public:
    RuleMax(const std::string &name) : Rule(name) {}

    std::shared_ptr<Rule> clone() const override {
        return std::make_shared<RuleMax>(*this);
    }

    std::shared_ptr<RuleCost> makeCost(double value) const override {
        return std::make_shared<RuleCostMax>(value);
    }

    /*
    RuleMax operator+(const RuleMax& other) const {
        return RuleMax(name, std::max(value, other.value));
    }
    */
};

class RuleSum : public Rule {
  public:
    RuleSum(const std::string &name) : Rule(name) {}

    std::shared_ptr<Rule> clone() const override {
        return std::make_shared<RuleSum>(*this);
    }

    std::shared_ptr<RuleCost> makeCost(double value) const override {
        return std::make_shared<RuleCostSum>(value);
    }

    /*
    RuleSum operator+(const RuleSum& other) const {
        return RuleSum(name, value + other.value);
    }
    */
};

#endif
