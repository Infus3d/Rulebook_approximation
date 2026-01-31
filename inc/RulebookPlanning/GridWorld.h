#ifndef _GRID_WORLD_H
#define _GRID_WORLD_H

#include <cmath>

class DiscreteProductState2D; // forward declaration

class DiscreteAction2D {
  public:
    int x, y;

    DiscreteAction2D(int ux, int uy) : x(ux), y(uy) {}
};

class DiscreteState2D {
  public:
    int x, y;

    DiscreteState2D() : x(0), y(0) {}

    DiscreteState2D(int x_, int y_) : x(x_), y(y_) {}

    bool operator==(const DiscreteState2D &other) const {
        return x == other.x && y == other.y;
    }

    bool operator==(const DiscreteProductState2D &other) const;

    DiscreteState2D operator+(const DiscreteAction2D &u) const {
        return DiscreteState2D(this->x + u.x, this->y + u.y);
    }

    friend std::ostream &operator<<(std::ostream &os,
                                    const DiscreteState2D &s) {
        os << "(" << s.x << "," << s.y << ")";
        return os;
    }
};

class DiscreteProductState2D {
  public:
    int x, y, q;

    DiscreteProductState2D() : x(0), y(0), q(0) {}

    DiscreteProductState2D(int x_, int y_, int q_) : x(x_), y(y_), q(q_) {}

    bool operator==(const DiscreteState2D &other) const {
        return x == other.x && y == other.y;
    }

    bool operator==(const DiscreteProductState2D &other) const {
        return x == other.x && y == other.y && q == other.q;
    }

    DiscreteProductState2D operator+(const DiscreteAction2D &u) const {
        return DiscreteProductState2D(this->x + u.x, this->y + u.y, q);
    }

    friend std::ostream &operator<<(std::ostream &os,
                                    const DiscreteProductState2D &s) {
        os << "(" << s.x << "," << s.y << "," << s.q << ")";
        return os;
    }
};

// Now that DiscreteProductState2D is defined, define the missing method
inline bool
DiscreteState2D::operator==(const DiscreteProductState2D &other) const {
    return x == other.x && y == other.y;
}

class DiscreteRegion2D {
  public:
    int xmin, xmax, ymin, ymax;

    DiscreteRegion2D(int xmin_, int xmax_, int ymin_, int ymax_)
        : xmin(xmin_), xmax(xmax_), ymin(ymin_), ymax(ymax_) {}

    bool contain(const DiscreteState2D &s) const {
        return s.x >= xmin && s.x <= xmax && s.y >= ymin && s.y <= ymax;
    }

    bool contain(const DiscreteProductState2D &ps) const {
        return ps.x >= xmin && ps.x <= xmax && ps.y >= ymin && ps.y <= ymax;
    }

    double distanceTo(const DiscreteProductState2D &ps) const {
        if (contain(ps))
            return 0.0;
        return std::min(abs(ps.x - xmin), abs(ps.x - xmax)) +
               std::min(abs(ps.y - ymin), abs(ps.y - ymax));
    }
};

// Hash function
namespace std {
template <> struct hash<DiscreteProductState2D> {
    size_t operator()(const DiscreteProductState2D &s) const {
        return ((std::hash<int>()(s.x) ^ (std::hash<int>()(s.y) << 1)) >> 1) ^
               (std::hash<int>()(s.q) << 1);
    }
};

template <> struct hash<DiscreteState2D> {
    std::size_t operator()(const DiscreteState2D &s) const noexcept {
        // Combine x and y with a simple hash combiner
        return std::hash<int>()(s.x) ^ (std::hash<int>()(s.y) << 1);
    }
};
} // namespace std

#endif
