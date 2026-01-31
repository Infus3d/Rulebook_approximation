#ifndef _GRAPH_H
#define _GRAPH_H

#include <queue>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Vertex.h"

class Graph {
  public:
    Graph() = default;

    virtual ~Graph() {
        clear(); // Ensure proper cleanup
    }

    // Copy constructor (deep copy)
    Graph(const Graph &other) {
        // Copy vertices
        for (const auto &[vid, vertex] : other.vertices)
            addVertex(vid);

        // Copy edges and rebuild connections
        for (const auto &edge : other.edges)
            addEdge(edge->from->vid, edge->to->vid, edge->eid, true);
    }

    // Copy assignment operator (deep copy)
    Graph &operator=(const Graph &other) {
        if (this == &other)
            return *this;

        clear();
        Graph tmp(other);       // Use copy constructor
        *this = std::move(tmp); // Use move assignment
        return *this;
    }

    // Move constructor
    Graph(Graph &&other) noexcept
        : vertices(std::move(other.vertices)), edges(std::move(other.edges)) {}

    // Move assignment operator
    Graph &operator=(Graph &&other) noexcept {
        if (this == &other)
            return *this;

        clear();
        vertices = std::move(other.vertices);
        edges = std::move(other.edges);
        return *this;
    }

    // Add a vertex to the graph
    size_t addVertex() {
        size_t vid = vertices.size();
        while (vertices.find(vid) != vertices.end())
            ++vid;
        addVertex(vid);
        return vid;
    }

    void addVertex(size_t vid) {
        assert(vertices.find(vid) == vertices.end());
        vertices[vid] = std::make_shared<Vertex>(vid);
    }

    // Add an edge to the graph
    size_t addEdge(size_t from, size_t to) {
        size_t eid = edges.size();
        addEdge(from, to, eid, true);
        return eid;
    }

    void addEdge(size_t from, size_t to, size_t eid,
                 bool allow_duplicate = true) {
        auto v1 = getVertex(from);
        auto v2 = getVertex(to);

        assert(v1 && v2);

        auto edge = std::make_shared<Eedge>(v1, v2, eid);
        addEdgeToGraph(v1, v2, edge, allow_duplicate);
    }

    void removeEdge(const std::shared_ptr<Eedge> &edge) {
        // Remove from top-level edges
        removeEdgeFromVertices(edge);
        edges.erase(std::remove(edges.begin(), edges.end(), edge), edges.end());
    }

    void removeVertex(size_t vertex_id) {
        auto it = vertices.find(vertex_id);
        if (it == vertices.end()) {
            throw std::invalid_argument(
                "Vertex ID " + std::to_string(vertex_id) + " not found.");
        }

        const auto &vertex = it->second;

        // Collect all incident edges (to avoid mutating while iterating)
        std::vector<std::shared_ptr<Eedge>> edges_to_remove;
        for (const auto &edge : vertex->out_edges) {
            edges_to_remove.push_back(edge);
        }
        for (const auto &edge : vertex->in_edges) {
            edges_to_remove.push_back(edge);
        }

        // Remove all incident edges from graph and from connected vertices
        for (const auto &edge : edges_to_remove) {
            removeEdge(edge);
        }

        // Finally, remove the vertex from the vertex map
        vertices.erase(it);
    }

    [[nodiscard]] const std::shared_ptr<Vertex> getVertex(size_t vid) const {
        auto it = vertices.find(vid);
        return it != vertices.end() ? it->second : nullptr;
    }

    [[nodiscard]] const std::shared_ptr<Eedge> getEdge(size_t eid) const {
        for (const auto &edge : edges) {
            if (edge->eid == eid)
                return edge;
        }
        return nullptr;
    }

    [[nodiscard]] const std::unordered_map<size_t, std::shared_ptr<Vertex>> &
    getVertices() const {
        return vertices;
    }

    [[nodiscard]] const std::vector<std::shared_ptr<Eedge>> &getEdges() const {
        return edges;
    }

    void clear() {
        for (auto &edge : edges) {
            edge->clear(); // breaks edge → vertex links
        }

        for (auto &[_, vertex] : vertices) {
            vertex->clear(); // breaks vertex → edge links
        }

        // Then clear the top-level containers
        edges.clear();
        vertices.clear();
    }

    [[nodiscard]] size_t getNumVertices() const { return vertices.size(); }

    [[nodiscard]] std::unordered_set<size_t>
    findSuccessors(size_t start_id) const {
        std::unordered_set<size_t> successors;
        std::queue<size_t> to_visit;

        auto it = vertices.find(start_id);
        if (it == vertices.end()) {
            // Start vertex not found
            return successors;
        }

        to_visit.push(start_id);

        while (!to_visit.empty()) {
            size_t current_id = to_visit.front();
            to_visit.pop();

            auto current_vertex = vertices.at(current_id);
            for (const auto &edge : current_vertex->out_edges) {
                size_t neighbor_id = edge->to->vid;
                if (successors.find(neighbor_id) == successors.end()) {
                    successors.insert(neighbor_id);
                    to_visit.push(neighbor_id);
                }
            }
        }

        return successors;
    }

    std::vector<size_t> topologicalSort() const {
        std::unordered_map<std::shared_ptr<Vertex>, bool> visited, on_stack;
        std::unordered_map<size_t, std::shared_ptr<Eedge>> parent_edge;
        std::stack<size_t> topo_stack;

        for (const auto &[_, vertex] : vertices) {
            if (!visited[vertex]) {
                if (!dfsVisit(vertex, visited, on_stack, parent_edge,
                              &topo_stack))
                    throw std::runtime_error("Graph contains a cycle; "
                                             "topological sort not possible.");
            }
        }

        std::vector<size_t> result;
        while (!topo_stack.empty()) {
            result.push_back(topo_stack.top());
            topo_stack.pop();
        }
        return result;
    }

    // Compute a path using DFS
    [[nodiscard]] std::vector<std::shared_ptr<Eedge>> getPath(size_t from,
                                                             size_t to) const {
        auto vI = getVertex(from);
        auto vG = getVertex(to);
        if (!vI || !vG)
            throw std::invalid_argument("Invalid initial or goal vertex");

        std::unordered_map<std::shared_ptr<Vertex>, bool> visited, on_stack;
        std::unordered_map<size_t, std::shared_ptr<Eedge>> parent_edge;

        // Run DFS from vI
        dfsVisit(vI, visited, on_stack, parent_edge);

        // Reconstruct path from vG to vI using parent_edge map
        std::vector<std::shared_ptr<Eedge>> path;
        size_t current = to;

        while (current != from) {
            if (parent_edge.find(current) == parent_edge.end())
                return {}; // no path
            auto edge = parent_edge[current];
            path.push_back(edge);
            current = edge->from->vid;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    // Determine whether the graph is DAG
    bool isDAG() const {
        std::unordered_map<std::shared_ptr<Vertex>, bool> visited, on_stack;
        std::unordered_map<size_t, std::shared_ptr<Eedge>> parent_edge;
        bool has_cycle = false;

        for (const auto &[_, vertex] : vertices) {
            if (!visited[vertex]) {
                dfsVisit(vertex, visited, on_stack, parent_edge, nullptr,
                         &has_cycle);
                if (has_cycle)
                    return false;
            }
        }
        return true;
    }

    // Determine whether the graph is totally ordered
    bool isTotallyOrdered() const {
        // Must be a DAG first
        if (!isDAG())
            return false;

        // Get the topological ordering
        auto topo = topologicalSort();

        // Ensure there is an edge from each vertex to the next one in topo
        // order
        for (size_t i = 0; i + 1 < topo.size(); ++i) {
            auto u = getVertex(topo[i]);
            auto v = getVertex(topo[i + 1]);
            bool edge_found = false;

            for (const auto &e : u->out_edges) {
                if (e->to == v) {
                    edge_found = true;
                    break;
                }
            }

            if (!edge_found)
                return false;
        }

        return true;
    }

    // Display the graph
    virtual void display() const {
        for (const auto &pair : vertices) {
            std::cout << "  " << pair.first << ": [";
            for (size_t i = 0; i < pair.second->out_edges.size(); ++i) {
                auto edge = pair.second->out_edges[i];
                std::cout << edge->to->vid;
                if (i != pair.second->out_edges.size() - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << "]" << std::endl;
        }
    }

    // Check that the edges and those contained in vertices are consistent
    [[nodiscard]] bool is_consistent() const {
        // Check that all edges in graph.edges are also in from->out_edges and
        // to->in_edges
        for (const auto &edge : edges) {
            auto from = edge->from;
            auto to = edge->to;

            if (!from || !to) {
                std::cerr << "Edge has null endpoint.\n";
                return false;
            }

            // Check if edge is in from->out_edges
            const auto &out_edges = from->out_edges;
            if (std::find(out_edges.begin(), out_edges.end(), edge) ==
                out_edges.end()) {
                std::cerr << "Edge from " << from->vid << " to " << to->vid
                          << " missing from from->out_edges.\n";
                return false;
            }

            // Check if edge is in to->in_edges
            const auto &in_edges = to->in_edges;
            if (std::find(in_edges.begin(), in_edges.end(), edge) ==
                in_edges.end()) {
                std::cerr << "Edge from " << from->vid << " to " << to->vid
                          << " missing from to->in_edges.\n";
                return false;
            }
        }

        // Check that all vertex->in_edges and out_edges exist in graph.edges
        for (const auto &[vid, vertex] : vertices) {
            for (const auto &edge : vertex->out_edges) {
                if (std::find(edges.begin(), edges.end(), edge) ==
                    edges.end()) {
                    std::cerr << "Edge in vertex " << vid
                              << " out_edges not found in graph.edges.\n";
                    return false;
                }
            }

            for (const auto &edge : vertex->in_edges) {
                if (std::find(edges.begin(), edges.end(), edge) ==
                    edges.end()) {
                    std::cerr << "Edge in vertex " << vid
                              << " in_edges not found in graph.edges.\n";
                    return false;
                }
            }
        }
        return true;
    }

  protected:
    std::unordered_map<size_t, std::shared_ptr<Vertex>> vertices;
    std::vector<std::shared_ptr<Eedge>> edges;

    bool addEdgeToGraph(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2,
                        std::shared_ptr<Eedge> edge, bool allow_duplicate) {
        if (!allow_duplicate) {
            for (const auto &e : v1->out_edges) {
                if (e->to == v2) {
                    edge->clear();
                    std::cout << "WARNING: adding duplicate edge from " << *v1
                              << " to " << *v2 << std::endl;
                    return false;
                }
            }
        }
        v1->addOutEdge(edge);
        v2->addInEdge(edge);
        edges.push_back(edge);
        return true;
    }

    void removeEdgeFromVertices(const std::shared_ptr<Eedge> &edge) {
        if (edge->from) {
            auto &out_vec = edge->from->out_edges;
            out_vec.erase(std::remove(out_vec.begin(), out_vec.end(), edge),
                          out_vec.end());
        }

        if (edge->to) {
            auto &in_vec = edge->to->in_edges;
            in_vec.erase(std::remove(in_vec.begin(), in_vec.end(), edge),
                         in_vec.end());
        }
        edge->clear();
    }

    // Shared DFS utility for building a parent map, topological order, and
    // detecting cycles
    bool
    dfsVisit(const std::shared_ptr<Vertex> &vertex,
             std::unordered_map<std::shared_ptr<Vertex>, bool> &visited,
             std::unordered_map<std::shared_ptr<Vertex>, bool> &on_stack,
             std::unordered_map<size_t, std::shared_ptr<Eedge>> &parent_edge,
             std::stack<size_t> *topo_stack = nullptr,
             bool *cycle_detected = nullptr) const {
        visited[vertex] = true;
        on_stack[vertex] = true;

        for (const auto &edge : vertex->out_edges) {
            auto next = edge->to;
            if (!visited[next]) {
                parent_edge[next->vid] = edge;
                if (!dfsVisit(next, visited, on_stack, parent_edge, topo_stack,
                              cycle_detected))
                    return false;
            } else if (on_stack[next]) {
                if (cycle_detected)
                    *cycle_detected = true;
                return false;
            }
        }

        on_stack[vertex] = false;
        if (topo_stack)
            topo_stack->push(vertex->vid);
        return true;
    }
};

#endif
