#ifndef _WEIGHTEDGRAPH_H
#define _WEIGHTEDGRAPH_H

#include "Graph.h"
#include "OptimalSet.h"
#include <functional>
#include <limits>
#include <memory>
#include <unordered_set>
#include <vector>

template <typename CostType> class WeightedEdge : public Eedge {
  public:
    CostType cost;

    WeightedEdge(const std::shared_ptr<Vertex> &v1,
                 const std::shared_ptr<Vertex> &v2, const CostType &cost,
                 size_t eid)
        : Eedge(v1, v2, eid), cost(cost) {}

    void setCost(CostType cost) { cost = cost; }
};

template <typename CostType>
std::ostream &
operator<<(std::ostream &os,
           const std::vector<std::shared_ptr<WeightedEdge<CostType>>> &edges) {
    os << "[";
    for (size_t i = 0; i < edges.size(); ++i) {
        const auto &edge = edges[i];
        if (edge) {
            os << *edge;
        } else {
            os << "null";
        }
        if (i + 1 != edges.size()) {
            os << ", ";
        }
    }
    os << "]";
    return os;
}

template <typename CostType> class WeightedGraph : public Graph {
  public:
    using WEdge = WeightedEdge<CostType>;
    using VertexPtr = std::shared_ptr<Vertex>;
    using WEdgePtr = std::shared_ptr<WEdge>;

    // Default constructor
    WeightedGraph() = default;

    // Default destructor to just call Graph::~Graph()
    ~WeightedGraph() = default;

    // Copy constructor (deep copy)
    WeightedGraph(const WeightedGraph &other) {
        // Copy vertices
        for (const auto &[vid, vertex] : other.vertices)
            addVertex(vid);

        // Copy edges and rebuild connections
        for (const auto &edge : other.edges) {
            auto wedge = std::dynamic_pointer_cast<WEdge>(edge);
            if (!wedge)
                throw std::runtime_error(
                    "Cannot copy: Edge is not of correct type.");
            addEdge(wedge->from->vid, wedge->to->vid, wedge->cost, wedge->eid,
                    true);
        }
    }

    // Copy assignment operator (deep copy)
    WeightedGraph &operator=(const WeightedGraph &other) {
        if (this == &other)
            return *this;

        clear();
        WeightedGraph<CostType> tmp(other);
        *this = std::move(tmp);
        return *this;
    }

    // Move constructor
    WeightedGraph(WeightedGraph &&other) noexcept : Graph(std::move(other)) {
        // All base members (vertices and edges) are moved via Graph
    }

    // Move assignment operator
    WeightedGraph &operator=(WeightedGraph &&other) noexcept {
        if (this == &other)
            return *this;

        Graph::operator=(std::move(other));
        return *this;
    }

    // Add an edge with weight to the graph
    size_t addEdge(size_t from, size_t to, const CostType &cost) {
        size_t eid = edges.size();
        addEdge(from, to, cost, eid, true);
        return eid;
    }

    void addEdge(size_t from, size_t to, const CostType &cost, size_t eid,
                 bool allow_duplicate = true) {
        auto v1 = getVertex(from);
        auto v2 = getVertex(to);

        if (!v1)
            std::cout << "Can't find from vertex " << from << std::endl;
        if (!v2)
            std::cout << "Can't find to vertex " << to << std::endl;

        assert(v1 && v2);

        auto edge = std::make_shared<WEdge>(v1, v2, cost, eid);
        addEdgeToGraph(v1, v2, edge, allow_duplicate);
    }

    // Display the graph with weights
    void display() const override {
        for (const auto &pair : vertices) {
            std::cout << "  \"" << pair.first << "\": [";
            for (size_t i = 0; i < pair.second->out_edges.size(); ++i) {
                auto edge = pair.second->out_edges[i];
                auto weighted_edge = std::dynamic_pointer_cast<WEdge>(edge);
                if (weighted_edge) {
                    std::cout << "[\"" << weighted_edge->to->vid << "\", "
                              << weighted_edge->cost << "]";
                } else {
                    std::cout << "[\"" << edge->to->vid << "\"]";
                }
                if (i != pair.second->out_edges.size() - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << "]" << std::endl;
        }
    }

    std::vector<WEdgePtr> getPath(size_t from, size_t to) {
        // Call base Graph's method
        const auto base_path = Graph::getPath(from, to);

        // Convert edges to WeightedEdge
        std::vector<WEdgePtr> weighted_path;
        weighted_path.reserve(base_path.size());

        for (const auto &edge : base_path) {
            auto wedge = std::dynamic_pointer_cast<WEdge>(edge);
            if (!wedge) {
                throw std::runtime_error(
                    "Edge in path is not of type WeightedEdge<CostType>.");
            }
            weighted_path.push_back(wedge);
        }

        return weighted_path;
    }

    std::vector<WEdgePtr> getDijkstraPath(size_t from, size_t to) const {
        auto vI = getVertex(from);
        auto vG = getVertex(to);
        if (!vI || !vG)
            throw std::invalid_argument("Invalid initial or goal vertex");

        const auto cost_to_come = this->getCostToComeFrom<CostType>(
            vI, false, [](const CostType &c) { return c; });

        std::vector<WEdgePtr> path;
        std::shared_ptr<Vertex> current = vG;

        while (current != vI) {
            bool found = false;
            for (const auto &edge : current->in_edges) {
                auto wedge = std::dynamic_pointer_cast<WEdge>(edge);
                auto u = wedge->from;
                if (cost_to_come.count(u->vid) &&
                    cost_to_come.at(current->vid) ==
                        cost_to_come.at(u->vid) + wedge->cost) {
                    path.push_back(wedge);
                    current = u;
                    found = true;
                    break;
                }
            }
            if (!found) {
                throw std::runtime_error("No valid predecessor found.");
            }
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    CostType reduceToOptimalSubgraph(size_t from, size_t to) {
        return reduceToOptimalSubgraph<CostType>(
            from, to, [](const CostType &c) { return c; });
    }

    template <
        typename SubCostType = CostType,
        typename GetSubCost = std::function<SubCostType(const CostType &)>>
    SubCostType reduceToOptimalSubgraph(size_t from, size_t to,
                                        const GetSubCost &getSubCost) {
        auto [cost_to_come, cost_to_go, optimal_cost] =
            computeOptimalCostInfo<SubCostType>(from, to, getSubCost);
        auto is_edge_optimal = makeOptimalityChecker(cost_to_come, cost_to_go,
                                                     optimal_cost, getSubCost);

        // Remove edges
        auto it = std::remove_if(
            edges.begin(), edges.end(), [&](const std::shared_ptr<Eedge> &edge) {
                const auto wedge = std::dynamic_pointer_cast<WEdge>(edge);
                if (!wedge)
                    return false;
                if (is_edge_optimal(wedge))
                    return false;

                // Remove from vertices as a side effect
                removeEdgeFromVertices(
                    edge); // removes from from->out_edges and to->in_edges
                edge->clear();
                return true; // mark for removal from top-level `edges`
            });

        edges.erase(it, edges.end());
        return optimal_cost;
    }

    WeightedGraph<CostType> extractOptimalSubgraph(size_t from,
                                                   size_t to) const {
        return extractOptimalSubgraph<CostType>(
            from, to, [](const CostType &c) { return c; });
    }

    template <
        typename SubCostType = CostType,
        typename GetSubCost = std::function<SubCostType(const CostType &)>>
    WeightedGraph<CostType>
    extractOptimalSubgraph(size_t from, size_t to,
                           const GetSubCost &getSubCost) const {
        auto [cost_to_come, cost_to_go, optimal_cost] =
            computeOptimalCostInfo<SubCostType>(from, to, getSubCost);
        auto is_edge_optimal = makeOptimalityChecker(cost_to_come, cost_to_go,
                                                     optimal_cost, getSubCost);

        WeightedGraph<CostType> optimal_graph;
        for (const auto &[vid, _] : vertices)
            optimal_graph.addVertex(vid);

        for (const auto &edge : edges) {
            auto wedge = std::dynamic_pointer_cast<WEdge>(edge);
            if (wedge && is_edge_optimal(wedge)) {
                optimal_graph.addEdge(wedge->from->vid, wedge->to->vid,
                                      wedge->cost, wedge->eid, true);
            }
        }

        return optimal_graph;
    }

    OptimalSet<std::vector<WEdgePtr>, CostType>
    getOptimalPaths(size_t from, size_t to, bool globalCheck = false) const {
        const auto vI = getVertex(from);
        const auto vG = getVertex(to);
        if (!vI || !vG || edges.empty())
            return OptimalSet<std::vector<WEdgePtr>, CostType>();

        size_t n = 0;

        // Initialize optimal_paths
        const auto wedge0 = std::dynamic_pointer_cast<WEdge>(edges[0]);
        const CostType zero_cost = wedge0->cost.getZero();

        std::unordered_map<VertexPtr,
                           OptimalSet<std::vector<WEdgePtr>, CostType>>
            optimal_paths;
        for (const auto &pair : vertices) {
            optimal_paths[pair.second] =
                OptimalSet<std::vector<WEdgePtr>, CostType>();
            if (pair.second == vI)
                optimal_paths[pair.second].insert(std::vector<WEdgePtr>(),
                                                  zero_cost, n);
        }

        // Initialize maps
        std::unordered_map<size_t, VertexPtr> vertices_t2g;

        Graph ST;
        ST.addVertex(n);
        vertices_t2g[n] = vI;
        std::queue<size_t> Q;
        Q.push(n);

        while (!Q.empty()) {
            const size_t vtid = Q.front();
            Q.pop();
            const auto current_vertex = vertices_t2g[vtid];
            const auto current_path_with_cost =
                optimal_paths[current_vertex].getElement(vtid);

            for (const auto &edge : current_vertex->out_edges) {
                const auto wedge = std::dynamic_pointer_cast<WEdge>(edge);
                const auto next_vertex = edge->to;
                std::vector<WEdgePtr> next_path =
                    current_path_with_cost.element;
                next_path.push_back(wedge);
                const auto next_cost =
                    current_path_with_cost.cost + wedge->cost;

                // global dominance check
                bool dominated = false;
                for (const auto &e : optimal_paths[vG].getAllElementIDs()) {
                    if (optimal_paths[vG].getElement(e).cost << next_cost) {
                        dominated = true;
                        break;
                    }
                }

                if (dominated && globalCheck) {
                    continue;
                }

                const std::unordered_set<size_t> removed_vtids =
                    optimal_paths[next_vertex].insert(next_path, next_cost,
                                                      n + 1);
                removeElementsFromQueue(Q, removed_vtids);
                if (optimal_paths[next_vertex].isIn(n + 1)) {
                    ++n;
                    ST.addVertex(n);
                    ST.addEdge(vtid, n, edge->eid);
                    vertices_t2g[n] = next_vertex;
                    Q.push(n);
                }
            }
        }

        return optimal_paths[vG];
    }

  private:
    template <typename SubCostType, typename GetSubCost = std::function<
                                        SubCostType(const CostType &)>>
    std::function<bool(const WEdgePtr &)> makeOptimalityChecker(
        const std::unordered_map<size_t, SubCostType> &cost_to_come,
        const std::unordered_map<size_t, SubCostType> &cost_to_go,
        const SubCostType &optimal_cost, const GetSubCost &getSubCost) const {
        return [=](const WEdgePtr &wedge) {
            size_t from_id = wedge->from->vid;
            size_t to_id = wedge->to->vid;
            SubCostType total = cost_to_come.at(from_id) +
                                getSubCost(wedge->cost) + cost_to_go.at(to_id);
            return total <= optimal_cost + SubCostType(1e-9);
        };
    }

    template <typename SubCostType, typename GetSubCost = std::function<
                                        SubCostType(const CostType &)>>
    std::tuple<std::unordered_map<size_t, SubCostType>,
               std::unordered_map<size_t, SubCostType>, SubCostType>
    computeOptimalCostInfo(size_t from, size_t to,
                           const GetSubCost &getSubCost) const {
        const auto vI = getVertex(from);
        const auto vG = getVertex(to);
        if (!vI || !vG)
            throw std::invalid_argument("Invalid initial or goal vertex");

        const auto cost_to_come =
            this->getCostToComeFrom<SubCostType>(vI, false, getSubCost);
        const auto cost_to_go =
            this->getCostToComeFrom<SubCostType>(vG, true, getSubCost);
        const auto optimal_cost = cost_to_come.at(to);

        return {cost_to_come, cost_to_go, optimal_cost};
    }

    void
    removeElementsFromQueue(std::queue<size_t> &Q,
                            const std::unordered_set<size_t> &to_remove) const {
        std::queue<size_t> newQueue;

        while (!Q.empty()) {
            size_t element = Q.front();
            Q.pop();

            // Check if the element is in the to_remove set
            if (to_remove.find(element) == to_remove.end()) {
                newQueue.push(element);
            }
        }

        // Swap the new queue with the original queue
        Q.swap(newQueue);
    }

    template <typename SubCostType, typename GetSubCost = std::function<
                                        SubCostType(const CostType &)>>
    std::unordered_map<size_t, SubCostType>
    getCostToComeFrom(const VertexPtr vI, bool reverse_graph,
                      const GetSubCost &getSubCost) const {
        std::unordered_map<size_t, SubCostType> cost_to_come;

        // Initialize all costs to "infinity"
        SubCostType INF = std::numeric_limits<SubCostType>::max();

        for (const auto &pair : vertices) {
            cost_to_come[pair.first] = INF;
        }

        // Dijkstra queue: (cost, vertex ID)
        using QElem = std::pair<SubCostType, VertexPtr>;
        std::priority_queue<QElem, std::vector<QElem>, std::greater<>> pq;

        cost_to_come[vI->vid] = SubCostType{}; // assume SubCostType{} = zero
        pq.emplace(SubCostType{}, vI);

        while (!pq.empty()) {
            auto [current_cost, current_vertex] = pq.top();
            pq.pop();

            if (current_cost > cost_to_come[current_vertex->vid])
                continue;

            const auto &vertex_edges = reverse_graph
                                           ? current_vertex->in_edges
                                           : current_vertex->out_edges;

            for (const auto &edge : vertex_edges) {
                const auto wedge = std::dynamic_pointer_cast<WEdge>(edge);
                const auto next_vertex = reverse_graph ? edge->from : edge->to;
                SubCostType edge_cost = getSubCost(wedge->cost);
                SubCostType new_cost = current_cost + edge_cost;

                if (new_cost < cost_to_come[next_vertex->vid]) {
                    cost_to_come[next_vertex->vid] = new_cost;
                    pq.emplace(new_cost, next_vertex);
                }
            }
        }

        return cost_to_come;
    }
};

#endif
