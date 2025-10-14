
#ifndef MULTI_OBJECTIVE_SEARCH_RULEBOOKGRAPH_H
#define MULTI_OBJECTIVE_SEARCH_RULEBOOKGRAPH_H
#include <vector>

class RulebookGraph {
private:
    size_t numRules;
    std::vector<std::vector<size_t>> adj;

    std::vector<size_t> ordered_rules;
    std::vector<size_t> ordered_components;

    size_t numQRules;
    std::vector<std::vector<size_t>> qAdj;
    std::vector<std::vector<size_t>> components;
    std::vector<size_t>  comp_roots; // roots of each component, the representing vertex

    // runs depth first search starting at vertex v.
    // each visited vertex is appended to the output vector when dfs leaves it.
    void dfs(size_t v, std::vector<std::vector<size_t>> const& adj, std::vector<size_t> &output, std::vector<bool> &visited);

    // input: adj -- adjacency list of G
    // output: components -- the strongly connected components in G
    // output: adj_cond -- adjacency list of G^SCC (by root vertices)
    void strongly_connected_components();

    void dfs(size_t v, std::vector<bool> &visited, std::vector<size_t> &output);

    void topological_sort();
    void recursively_exclude(size_t comp, std::vector<bool>& excluded) const;

public:
    RulebookGraph(size_t numRules) : numRules(numRules), adj(numRules, std::vector<size_t>()) {}

    void add_relationship(size_t fromR, size_t toR);

    void calculate_quotient_graph();

    const std::vector<size_t>& get_ordered_rules() const;

    const std::vector<size_t>& get_Q_neighbors(size_t rule) const;

    size_t get_Q_root(size_t rule) const;

    const std::vector<size_t>& get_component_rules(size_t rootId) const;

    bool is_dominated_Tr(const std::vector<size_t>& a, const std::vector<size_t>& b) const;
    bool is_dominated_Sh(const std::vector<size_t>& a, const std::vector<size_t>& b) const;

    bool is_dominated_Tr(const std::vector<size_t>& a, const std::vector<size_t>& b, const std::vector<double> eps) const;
    bool is_dominated_Sh(const std::vector<size_t>& a, const std::vector<size_t>& b, const std::vector<double> eps) const;

    bool dominates(const std::vector<size_t> &a, const std::vector<size_t> &b, const std::vector<double> eps) const;
};

#endif //MULTI_OBJECTIVE_SEARCH_RULEBOOKGRAPH_H