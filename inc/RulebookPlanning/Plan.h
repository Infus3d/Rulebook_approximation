#ifndef _PLAN_H
#define _PLAN_H

class Plan {
  public:
    Plan() = default;

    Plan(const std::vector<int> &actions) : actions(actions) {}

    void add(int u) { actions.push_back(u); }

    bool operator==(const Plan &other) const {
        return actions == other.actions;
    }

    // Overloading the stream insertion operator for easy printing
    friend std::ostream &operator<<(std::ostream &os, const Plan &pi) {
        os << "< ";
        for (auto u : pi.actions)
            os << u << " ";
        os << ">";
        return os;
    }

  private:
    std::vector<int> actions;
};

#endif
