
#ifndef MULTI_OBJECTIVE_SEARCH_RULEBOOKSEARCH_H
#define MULTI_OBJECTIVE_SEARCH_RULEBOOKSEARCH_H

#include "Utils/Definitions.h"
#include "Utils/Logger.h"
#include "Utils/MapQueue.h"
#include"DominanceChecker.h"
#include "AbstractSolver.h"
#include "Utils/RulebookGraph.h"


class RulebookSearch: public AbstractSolver {
protected:
    size_t num_of_objectives;
    MergeStrategy ms=MergeStrategy::RANDOM;
    RulebookGraph rulebook_graph;

    std::unique_ptr<LocalCheckRulebook> local_dom_checker;
    std::unique_ptr<SolutionCheckRulebook> solution_dom_checker;

    virtual void insert(RealizationPairPtr &pp, RPQueue &queue);
    bool is_dominated(RealizationPairPtr ap);
    void merge_to_solutions(const RealizationPairPtr &pp, RealizationSolutionSet &solutions);
    std::vector<std::vector<RealizationPairPtr>> expanded;
    void init_search() override;

public:

    virtual std::string get_solver_name() override;


    void set_merge_strategy(MergeStrategy new_ms){ms = new_ms;}
    void set_rulebook_graph(const RulebookGraph& new_graph) { rulebook_graph = new_graph; }
    RulebookSearch(const AdjacencyMatrix &adj_matrix, EPS eps, const LoggerPtr logger=nullptr);
    virtual void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit=UINT_MAX) override;
};

#endif //MULTI_OBJECTIVE_SEARCH_RULEBOOKSEARCH_H