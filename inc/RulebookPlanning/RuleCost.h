#ifndef _RULE_COST_H
#define _RULE_COST_H

#include <iostream>
#include <limits>
#include <memory>

class RuleCost {
  public:
    RuleCost(double v = 0, double eps = 0) : value(v), eps(eps) {}

    virtual ~RuleCost() = default;

    // This is to allow allow polymorphic copying to
    // allow RulebookCost to store its derived classes in a single container.
    virtual std::shared_ptr<RuleCost> clone() const = 0;

    virtual std::shared_ptr<RuleCost>
    operator+(const RuleCost &other) const = 0;

    // Overloading the stream insertion operator for easy printing
    friend std::ostream &operator<<(std::ostream &os, const RuleCost &rc) {
        os << rc.value;
        return os;
    }

    // Overloading comparison operators
    bool operator<(const RuleCost &other) const { return value < other.value; }

    bool operator>(const RuleCost &other) const { return value > other.value; }

    bool operator<<(const RuleCost &other) const { return value < (1 + eps) * other.value; }

    bool operator>>(const RuleCost &other) const { return value > (1 + eps) * other.value; }

    bool operator<=(const RuleCost &other) const {
        return value <= other.value;
    }

    bool operator>=(const RuleCost &other) const {
        return value >= other.value;
    }

    bool operator==(const RuleCost &other) const {
        return value == other.value;
    }

    void setValue(double v) { value = v; }

    void setEps(double e) { eps = e; }

    double getValue() const { return value; }

    double getEps() const { return eps; }

  protected:
    double value;
    double eps;
};

class RuleCostMax : public RuleCost {
  public:
    RuleCostMax(double v = 0, double eps = 0) : RuleCost(v, eps) {}

    std::shared_ptr<RuleCost> clone() const override {
        return std::make_shared<RuleCostMax>(*this);
    }

    std::shared_ptr<RuleCost> operator+(const RuleCost &other) const override {
        return std::make_shared<RuleCostMax>(std::max(value, other.getValue()), eps);
    }

    /*
    std::shared_ptr<RuleCostMax> operator+(const RuleCostMax& other) const {
        return std::make_shared<RuleCostMax>(std::max(value, other.value));
    }
    */

    RuleCostMax operator+(const RuleCostMax &other) const {
        return RuleCostMax(std::max(value, other.value), eps);
    }

    RuleCostMax operator+(double other) const {
        return RuleCostMax(std::max(value, other), eps);
    }
};

class RuleCostSum : public RuleCost {
  public:
    RuleCostSum(double v = 0, double eps = 0) : RuleCost(v, eps) {}

    std::shared_ptr<RuleCost> clone() const override {
        return std::make_shared<RuleCostSum>(*this);
    }

    std::shared_ptr<RuleCost> operator+(const RuleCost &other) const override {
        return std::make_shared<RuleCostSum>(value + other.getValue(), eps);
    }

    /*
    std::shared_ptr<RuleCostSum> operator+(const RuleCostSum& other) const {
        return std::make_shared<RuleCostSum>(value + other.value);
    }
    */

    RuleCostSum operator+(const RuleCostSum &other) const {
        return RuleCostSum(value + other.value, eps);
    }

    RuleCostSum operator+(double other) const {
        return RuleCostSum(value + other, eps);
    }
};

template <> class std::numeric_limits<RuleCostMax> {
  public:
    static constexpr bool is_specialized = true;

    static RuleCostMax max() noexcept {
        return RuleCostMax(std::numeric_limits<double>::max());
    }

    static RuleCostMax min() noexcept {
        return RuleCostMax(std::numeric_limits<double>::lowest());
    }

    static RuleCostMax lowest() noexcept {
        return RuleCostMax(std::numeric_limits<double>::lowest());
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

template <> class std::numeric_limits<RuleCostSum> {
  public:
    static constexpr bool is_specialized = true;

    static RuleCostSum max() noexcept {
        return RuleCostSum(std::numeric_limits<double>::max());
    }

    static RuleCostSum min() noexcept {
        return RuleCostSum(std::numeric_limits<double>::lowest());
    }

    static RuleCostSum lowest() noexcept {
        return RuleCostSum(std::numeric_limits<double>::lowest());
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

// Make RuleCostMax and RuleCostSum hashable
namespace std {
template <> struct hash<RuleCostMax> {
    std::size_t operator()(const RuleCostMax &cost) const {
        return std::hash<double>()(cost.getValue());
    }
};

template <> struct hash<RuleCostSum> {
    std::size_t operator()(const RuleCostSum &cost) const {
        return std::hash<double>()(cost.getValue());
    }
};
} // namespace std

#endif
