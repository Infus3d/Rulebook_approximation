#include <iostream>
#include <memory>
#include <time.h>
#include <fstream>
#include <random>

#include "ShortestPathHeuristic.h"
#include "Utils/Definitions.h"
#include "Utils/IOUtils.h"
#include "Utils/Logger.h"
#include "BOAStar.h"
#include "PPA.h"
#include "SingleCriteria.h"
#include "ApexSearch.h"
#include "NAMOA.h"

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

#include "RulebookSearch.h"

#include "RulebookPlanning/Rulebook.h"
#include "RulebookPlanning/RulebookCost.h"
#include "RulebookPlanning/WeightedGraph.h"

using namespace std;
using WEdge = WeightedEdge<RulebookCost>;
using VertexPtr = std::shared_ptr<Vertex>;
using WEdgePtr = std::shared_ptr<WEdge>;

const std::string resource_path = "resources/";
const std::string output_path = "output/";
const MergeStrategy DEFAULT_MERGE_STRATEGY = MergeStrategy::RANDOM;
std::string alg_variant = "";


// Simple example to demonstarte the usage of the algorithm

SolutionSet single_run_map(size_t graph_size, AdjacencyMatrix& graph, AdjacencyMatrix&inv_graph, size_t source, size_t target, std::ofstream& output, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, unsigned int time_limit, RulebookGraph& rgraph) {
    // Compute heuristic
    std::cout << "Start Computing Heuristic" << std::endl;
    ShortestPathHeuristic sp_heuristic(target, graph_size, inv_graph);
    // sp_heuristic.set_all_to_zero();
    std::cout << "Finish Computing Heuristic\n" << std::endl;

    using std::placeholders::_1;
    Heuristic heuristic = std::bind( &ShortestPathHeuristic::operator(), sp_heuristic, _1);

    SolutionSet solutions;
    int num_exp, num_gen;
    auto runtime = std::clock();

    std::unique_ptr<AbstractSolver> solver;
    if (algorithm == "PPA"){
        Pair<double> eps_pair({eps, eps});
        solver = std::make_unique<PPA>(graph, eps_pair, logger);
    }else if (algorithm == "BOA"){
        Pair<double> eps_pair({eps, eps});
        solver = std::make_unique<BOAStar>(graph, eps_pair, logger);
    }else if (algorithm == "NAMOAdr"){
        EPS eps_vec (graph.get_num_of_objectives(), eps);
        solver = std::make_unique<NAMOAdr>(graph, eps_vec, logger);
        // ((ApexSearch*)solver.get())->set_merge_strategy(ms);
    }else if (algorithm == "Apex"){
        EPS eps_vec (graph.get_num_of_objectives(), eps);
        solver = std::make_unique<ApexSearch>(graph, eps_vec, logger);
        ((ApexSearch*)solver.get())->set_merge_strategy(ms);
    }else if (algorithm == "RulebookApex") {
        EPS eps_vec ({eps, eps, eps, eps, eps});
        solver = std::make_unique<RulebookSearch>(graph, eps_vec, logger);
        ((RulebookSearch*)solver.get())->set_merge_strategy(ms);
        ((RulebookSearch*)solver.get())->set_rulebook_graph(rgraph);
    }else{
        std::cerr << "unknown solver name" << std::endl;
        exit(-1);
    }
    auto start =std::clock();
    (*solver)(source, target, heuristic, solutions, time_limit);
    runtime = std::clock() - start;

    std::cout << "Node expansion: " << solver->get_num_expansion() << std::endl;
    std::cout << "Runtime: " <<  ((double) runtime) / CLOCKS_PER_SEC<< std::endl;
    num_exp = solver->get_num_expansion();
    num_gen = solver->get_num_generation();
    for (auto sol: solutions){
        std::cout << *sol << std::endl;
    }


    output << algorithm << "-" << alg_variant << "(" << eps << ")" << "\t"
           << source << "\t" << target << "\t"
           << num_gen << "\t"
           << num_exp << "\t"
           << solutions.size() << "\t"
           << (double) runtime / CLOCKS_PER_SEC
           << std::endl;

    std::cout << "-----End Single Example-----" << std::endl;

    return solutions;
}

void single_run_map(size_t graph_size, std::vector<Edge> & edges, size_t source, size_t target, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit) {

    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);

    // build rulebook for RulebookSearch
    RulebookGraph rgraph(3);
    rgraph.add_relationship(0, 1);
    rgraph.add_relationship(0, 2);
    // rgraph.add_relationship(3, 2);

    assert(graph.get_num_of_objectives() == 3);

    rgraph.calculate_quotient_graph();

    std::ofstream stats;
    stats.open(output_file);

    single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit, rgraph);
    stats.close();
}

OptimalSet<std::vector<WEdgePtr>, RulebookCost> single_run_map_planning(size_t graph_size, std::vector<Edge> &edges, size_t source, size_t target, std::ostream& output, LoggerPtr logger, double eps, size_t time_limit, Rulebook &rulebook, EPS epsV = {}) {
    RulebookCost::setRulebook(rulebook);

    WeightedGraph<RulebookCost> graph;
    unordered_map<size_t, bool> Vs;

    for (auto &e : edges) {
        if (Vs.find(e.source) == Vs.end()) {
            graph.addVertex(e.source);
            Vs.insert({e.source, true});
        }
        if (Vs.find(e.target) == Vs.end()) {
            graph.addVertex(e.target);
            Vs.insert({e.target, true});
        }
        RulebookCost c;
        for (int i=0; i<rulebook.getNumRules(); i++) {
            c.setRuleCost(i, e.cost[i]);
            c.setRuleEps(i, epsV[i]);
        }
        graph.addEdge(e.source, e.target, c);
    }

    auto start =std::clock();
    const auto optimal_set = graph.getOptimalPaths(source, target, (eps > 0.0000001));
    auto runtime = std::clock() - start;

    std::cout << "Graph size: " <<graph_size << std::endl;
    std::cout << "Runtime: " <<  ((double) runtime) / CLOCKS_PER_SEC<< std::endl;

    std::cout << optimal_set << std::endl;

    vector<vector<size_t>> planningSolutionsVector;
    for (size_t eid : optimal_set.getAllElementIDs()) {
        const auto ele = optimal_set.getElement(eid);
        vector<size_t> sol;
        for (const auto it : ele.cost.getCosts()) {
            sol.push_back(it->getValue());
        }
        planningSolutionsVector.push_back(sol);
    }
    std::sort(planningSolutionsVector.begin(), planningSolutionsVector.end());
    planningSolutionsVector.erase(std::unique(planningSolutionsVector.begin(), planningSolutionsVector.end()), planningSolutionsVector.end());

    output << "Planning" << "-" << "(" << eps << ")" << "\t"
           << source << "\t" << target << "\t"
           << planningSolutionsVector.size() << "\t"
           << (double) runtime / CLOCKS_PER_SEC
           << std::endl;

    std::cout << "-----End Single Example-----" << std::endl;

    return optimal_set;
}

void run_query(size_t graph_size, std::vector<Edge> & edges, std::string query_file, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit) {
    std::ofstream stats;
    stats.open(output_file);


    std::vector<std::pair<size_t, size_t>> queries;
    if (load_queries(query_file, queries) == false) {
        std::cout << "Failed to load queries file" << std::endl;
        return;
    }

    // Build graphs
    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);

    EPS epsV{eps, eps, eps, 0};

    // build rulebook for planning
    Rulebook rulebook;

    size_t r0 = rulebook.addRule(RuleSum("r0"));
    size_t r1 = rulebook.addRule(RuleSum("r1"));
    size_t r2 = rulebook.addRule(RuleSum("r2"));
    size_t r3 = rulebook.addRule(RuleSum("r3"));

    // rulebook.setEquivalentClasses({{r0}, {r1}, {r2}, {r3}});
    // rulebook.addGTRelation(r3, r0);
    // rulebook.addGTRelation(r3, r1);
    // rulebook.addGTRelation(r3, r2);

    rulebook.setEquivalentClasses({{r0}, {r1}, {r2}, {r3}});
    rulebook.addGTRelation(r3, r1);
    rulebook.addGTRelation(r3, r2);
    rulebook.addGTRelation(r3, r0);

    rulebook.build();

    // build rulebook for RulebookSearch
    RulebookGraph rgraph(4);
    rgraph.add_relationship(3, 0);
    rgraph.add_relationship(3, 1);
    rgraph.add_relationship(3, 2);
    // rgraph.add_relationship(3, 2);

    assert(graph.get_num_of_objectives() == 4);

    rgraph.calculate_quotient_graph();

    size_t query_count = 0;
    for (auto iter = queries.begin(); iter != queries.end(); ++iter) {

        query_count++;
        std::cout << "Started Query: " << query_count << "/" << queries.size() << std::endl;
        size_t source = iter->first;
        size_t target = iter->second;
        if (algorithm == "Planning") {
            // if (query_count != 4 && query_count != 8 && query_count != 11) continue;
            single_run_map_planning(graph_size, edges, source, target, stats, logger, eps, time_limit, rulebook, epsV);
        } else {
            single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit, rgraph);
        }
    }
    stats.close();
}

void find_all_solutions(AdjacencyMatrix& graph, std::vector<std::vector<double>>& solutions, int u, int goal, std::vector<double> sofar) {
    if (u == goal) {
        solutions.push_back(sofar);
        return;
    }
    for (auto e : graph[u]) {
        int v = e.target;
        std::vector<double> newcost(sofar);
        for (int j=0; j<newcost.size(); j++) {
            newcost[j] += e.cost[j];
        }
        find_all_solutions(graph, solutions, v, goal, newcost);
    }
}

void test_random_graph() {

    // std::vector<Edge> edges;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distr(1, 100000);
    std::uniform_int_distribution<> fifty(1, 2);
    std::uniform_real_distribution<> rigged(0.0, 1.0);
    size_t graph_size = 400;
    double eps = 0.01;

    std::ofstream stats;
    stats.open("RandomGraph_400_eps_0.01_equiv34_Preor3.txt");

    int attemptCount = 0, maxAttempts = 10;

    double rulebookSolSize = 0, rulebookRuntime = 0;
    double planningSolSize = 0, planningRuntime = 0;


    while (attemptCount < maxAttempts) {
        std::cout << "Attempt: " << ++attemptCount << std::endl;
        std::vector<Edge> edges;
        for (int i=1; i<=graph_size; i++) {
            for (int j=i+1; j<=graph_size; j++) {
                if (fifty(gen) == 1) continue;
                vector<size_t> cost;
                // for (int t=0; t<3; t++) {
                //     if (!t) {
                //         cost.push_back(!(rigged(gen) < 0.99));
                //     } else {
                //         cost.push_back(distr(gen));
                //     }
                // }
                cost.push_back(distr(gen));
                cost.push_back(distr(gen));
                cost.push_back(distr(gen));
                cost.push_back(distr(gen));
                // cost.push_back(!(rigged(gen) < 0.80));
                edges.push_back(Edge(i, j, cost));
                // std::cout << i << " " << j << " [" << cost[0] << " " << cost[1] << " " << cost[2] << "] " << std::endl;
            }
        }

        AdjacencyMatrix graph(graph_size, edges);
        AdjacencyMatrix inv_graph(graph_size, edges, true);
        EPS epsV{eps, eps, eps, eps};

        // build rulebook for planning
        Rulebook rulebook;

        size_t r0 = rulebook.addRule(RuleSum("r0"));
        size_t r1 = rulebook.addRule(RuleSum("r1"));
        size_t r2 = rulebook.addRule(RuleSum("r2"));
        size_t r3 = rulebook.addRule(RuleSum("r3"));

        rulebook.setEquivalentClasses({{r0}, {r1},  {r2, r3}});
        rulebook.addGTRelation(r2, r0);
        rulebook.addGTRelation(r3, r1);

        // rulebook.setEquivalentClasses({{r0}, {r1}, {r2}, {r3}});
        // rulebook.addGTRelation(r3, r0);
        // rulebook.addGTRelation(r3, r1);
        // rulebook.addGTRelation(r3, r2);

        rulebook.build();

        // build rulebook for RulebookSearch
        RulebookGraph rgraph(4);
        rgraph.add_relationship(2, 0);
        rgraph.add_relationship(3, 1);
        rgraph.add_relationship(3, 2);
        rgraph.add_relationship(2, 3);
        // rgraph.add_relationship(3, 2);

        assert(graph.get_num_of_objectives() == 4);

        rgraph.calculate_quotient_graph();

        LoggerPtr logger = nullptr;

        size_t source = 1;
        size_t target = graph_size;
        string algorithm = "";

        MergeStrategy ms = MergeStrategy::RANDOM;

        size_t time_limit = 300;

        // std::vector<std::vector<double>> solutions;
        // solutions.clear();
        // find_all_solutions(graph, solutions, source, target, std::vector<double>{0, 0, 0});
        //
        // cout << "Printing all solutions:" << std::endl;
        // for (auto sol : solutions) {
        //     cout << "[";
        //     for (auto i : sol) {
        //         cout << i << ", ";
        //     }
        //     cout << "]" << std::endl;
        // }

        algorithm = "Apex";
        cout << "\n\nApex:" << std::endl;
        single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit, rgraph);

        algorithm = "Rulebook";
        cout << "\n\nRulebook:" << std::endl;
        // SolutionSet rulebookSolutions =
        //     single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, 0, time_limit, rgraph);

        SolutionSet rulebookSolutionsEps =
            single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit, rgraph);

        // vector<vector<size_t>> rulebookSolutionsVector;
        // for (auto sol : rulebookSolutions) {
        //     rulebookSolutionsVector.push_back(sol->f);
        // }


        algorithm = "Planning";
        cout << "\n\nPlanning:" << std::endl;

        OptimalSet<std::vector<WEdgePtr>, RulebookCost> planningSolutions =
            single_run_map_planning(graph_size, edges, source, target, stats, logger, 0, time_limit, rulebook, epsV);

        OptimalSet<std::vector<WEdgePtr>, RulebookCost> planningSolutionsEps =
            single_run_map_planning(graph_size, edges, source, target, stats, logger, eps, time_limit, rulebook, epsV);

        // vector<vector<size_t>> planningSolutionsVector;
        // for (size_t eid : planningSolutions.getAllElementIDs()) {
        //     const auto ele = planningSolutions.getElement(eid);
        //     vector<size_t> sol;
        //     for (const auto it : ele.cost.getCosts()) {
        //         sol.push_back(it->getValue());
        //     }
        //     planningSolutionsVector.push_back(sol);
        // }
        //
        // vector<vector<size_t>> planningSolutionsEpsVector;
        // for (size_t eid : planningSolutionsEps.getAllElementIDs()) {
        //     const auto ele = planningSolutionsEps.getElement(eid);
        //     vector<size_t> sol;
        //     for (const auto it : ele.cost.getCosts()) {
        //         sol.push_back(it->getValue());
        //     }
        //     planningSolutionsEpsVector.push_back(sol);
        // }
        //
        // std::sort(rulebookSolutionsVector.begin(), rulebookSolutionsVector.end());
        // std::sort(planningSolutionsVector.begin(), planningSolutionsVector.end());
        // planningSolutionsVector.erase(std::unique(planningSolutionsVector.begin(), planningSolutionsVector.end()), planningSolutionsVector.end());
        //
        // //assert(rulebookSolutionsVector.size() == planningSolutionsVector.size());
        // bool good = true;
        // if (rulebookSolutionsVector.size() != planningSolutionsVector.size()) break;
        // for (int i=0; i<(int)rulebookSolutionsVector.size(); i++) {
        //     bool same = true;
        //     const auto a = rulebookSolutionsVector[i];
        //     const auto b = planningSolutionsVector[i];
        //     // assert(a.size() == b.size());
        //     if (a.size() != b.size()) {
        //         good = false;
        //         break;
        //     }
        //     for (int j=0; j<(int)a.size(); j++) {
        //         // assert(a[j] == b[j]);
        //         if (a[j] != b[j]) {
        //             good = false;
        //             break;
        //         }
        //     }
        //     if (!good) break;
        // }
        //
        //
        // if (!good) break;
        //
        // std::cout << "Status: OK, BOTH ARE THE SAME" << std::endl;
        //
        // for (const auto &sol : planningSolutionsVector) {
        //     bool found = false;
        //     for (auto epsSol : rulebookSolutionsEps) {
        //         if (rgraph.dominates(epsSol->f, sol, {eps, eps, eps, eps})) {
        //             found = true;
        //             break;
        //         }
        //     }
        //     if (!found) {
        //         good = false;
        //         break;
        //     }
        // }
        //
        // // assert(rulebookSolutions.size() == rulebookSolutionsEps.size());
        //
        // if (!good) break;
        //
        // std::cout << "Status: OK, RULEBOOK APPROXIMATE SOLUTION SET COVERS ALL SOLUTIONS" << std::endl;
        //
        // for (const auto &sol : planningSolutionsVector) {
        //     bool found = false;
        //     for (const auto &epsSol : planningSolutionsEpsVector) {
        //         if (rgraph.dominates(epsSol, sol, {eps, eps, eps, eps})) {
        //             found = true;
        //             break;
        //         }
        //     }
        //     if (!found) {
        //         good = false;
        //         break;
        //     }
        // }
        //
        // // assert(rulebookSolutions.size() == rulebookSolutionsEps.size());
        //
        // if (!good) break;
        //
        // std::cout << "Status: OK, PLANNING APPROXIMATE SOLUTION SET COVERS ALL SOLUTIONS" << std::endl;
    }

    std::cout << "\n\n************ Found a counter-example: *************\n" << std::endl;
    stats.close();
}

void test_manual_graph() {
    // Test case 1
    // std::vector<Edge> edges;
    // size_t graph_size = 5;
    // edges.push_back(Edge(1, 2, {6, 1, 10}));
    // edges.push_back(Edge(1, 3, {3, 1, 2}));
    // edges.push_back(Edge(3, 2, {3, 1, 2}));
    // edges.push_back(Edge(1, 4, {2, 1, 1}));
    // edges.push_back(Edge(4, 5, {2, 1, 1}));
    // edges.push_back(Edge(5, 2, {2, 1, 1}));
    // edges.push_back(Edge(4, 3, {10, 10, 10}));
    // edges.push_back(Edge(3, 5, {3, 1, 4}));

    // uncomment below for additional edges
    // edges.push_back(Edge(1,5, {4,1,1}));
    // edges.push_back(Edge(2, 4, {1, 1, 1}));


    // Test case 2
    // std::vector<Edge> edges;
    // size_t graph_size = 4;
    // edges.push_back(Edge(1, 2, {5, 3, 6}));
    // edges.push_back(Edge(1, 3, {6, 9, 5}));
    // edges.push_back(Edge(2, 3, {8, 6, 5}));
    // edges.push_back(Edge(2, 4, {9, 2, 8}));
    // edges.push_back(Edge(3, 4, {1, 1, 9}));


    // Test case 3
    // A*pex Assertion Fails on this example?
    // std::vector<Edge> edges;
    // size_t graph_size = 5;
    // edges.push_back(Edge(1, 2, {2, 8, 6}));
    // edges.push_back(Edge(1, 4, {8, 7, 1}));
    // edges.push_back(Edge(2, 4, {5, 4, 1}));
    // edges.push_back(Edge(2, 5, {9, 5, 9}));
    // edges.push_back(Edge(2, 3, {7, 10, 4}));
    // edges.push_back(Edge(1, 3, {3, 1, 8}));
    // edges.push_back(Edge(3, 5, {1, 5, 8}));
    // EPS eps({0.0, 0.0, 0.0});


    // Test case 4
    // std::vector<Edge> edges;
    // size_t graph_size = 5;
    // edges.push_back(Edge(1, 2, {8, 3, 5}));
    // edges.push_back(Edge(2, 3, {9, 10, 1}));
    // edges.push_back(Edge(2, 4, {6, 7, 6}));
    // edges.push_back(Edge(2, 5, {10, 7, 2}));
    // edges.push_back(Edge(3, 5, {3, 5, 4}));
    // edges.push_back(Edge(4, 5, {2, 7, 10}));
    // edges.push_back(Edge(3, 4, {6, 2, 4}));


    // Test case 5
    // std::vector<Edge> edges;
    // size_t graph_size = 5;
    // edges.push_back(Edge(1, 3, {7, 1, 7}));
    // edges.push_back(Edge(1, 4, {3, 5, 2}));
    // edges.push_back(Edge(3, 4, {1, 8, 8}));
    // edges.push_back(Edge(3, 5, {3, 9, 5}));
    // edges.push_back(Edge(4, 5, {8, 9, 6}));

    // 1 3 [0 104 838]
    // 1 4 [0 893 547]
    // 1 5 [0 966 201]
    // 2 3 [0 431 774]
    // 2 4 [0 814 147]
    // 3 4 [0 140 772]
    // 3 5 [1 436 308]
    // 4 5 [0 659 549]

    // Test case 6
    std::vector<Edge> edges;
    size_t graph_size = 5;
    edges.push_back(Edge(1, 3, {0, 10, 80}));
    edges.push_back(Edge(1, 4, {0, 90, 50}));
    edges.push_back(Edge(1, 5, {0, 90, 20}));
    edges.push_back(Edge(3, 4, {0, 10, 70}));
    edges.push_back(Edge(3, 5, {1, 40, 30}));
    edges.push_back(Edge(4, 5, {0, 60, 50}));

    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);

    // build rulebook for planning
    Rulebook rulebook;

    size_t r0 = rulebook.addRule(RuleSum("r0"));
    size_t r1 = rulebook.addRule(RuleSum("r1"));
    size_t r2 = rulebook.addRule(RuleSum("r2"));
    // size_t r3 = rulebook.addRule(RuleSum("r3"));

    // rulebook.setEquivalentClasses({{r0}, {r1}, {r2}, {r3}});
    // rulebook.addGTRelation(r3, r0);
    // rulebook.addGTRelation(r3, r1);
    // rulebook.addGTRelation(r3, r2);

    rulebook.setEquivalentClasses({{r0}, {r1}, {r2}});
    rulebook.addGTRelation(r0, r1);
    rulebook.addGTRelation(r0, r2);

    rulebook.build();

    // build rulebook for RulebookSearch
    RulebookGraph rgraph(3);
    rgraph.add_relationship(0, 1);
    rgraph.add_relationship(0, 2);
    // rgraph.add_relationship(3, 2);

    assert(graph.get_num_of_objectives() == 3);

    rgraph.calculate_quotient_graph();

    std::ofstream stats;
    stats.open(output_path + "output.txt", std::fstream::app);

    LoggerPtr logger = nullptr;

    size_t source = 1;
    size_t target = graph_size;
    string algorithm = "";

    MergeStrategy ms = MergeStrategy::RANDOM;
    double eps = 0.0;

    size_t time_limit = 300;

    algorithm = "Apex";
    cout << "\n\nApex:" << std::endl;
    single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit, rgraph);

    algorithm = "RulebookApex";
    cout << "\n\nRulebookApex:" << std::endl;
    SolutionSet rulebookSolutions =
        single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit, rgraph);

    algorithm = "Planning";
    cout << "\n\nPlanning:" << std::endl;

    OptimalSet<std::vector<WEdgePtr>, RulebookCost> planningSolutions =
        single_run_map_planning(graph_size, edges, source, target, stats, logger, eps, time_limit, rulebook);
}

int main(int argc, char** argv){
    namespace po = boost::program_options;

    std::vector<string> objective_files;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("start,s", po::value<int>()->default_value(-1), "start location")
        ("goal,g", po::value<int>()->default_value(-1), "goal location")
        ("query,q", po::value<std::string>()->default_value(""), "number of agents")
        ("map,m",po::value< std::vector<string> >(&objective_files)->multitoken(), "files for edge weight")
        ("eps,e", po::value<double>()->default_value(0), "approximation factor")
        ("merge", po::value<std::string>()->default_value(""), "strategy for merging apex node pair: SMALLER_G2, RANDOM or MORE_SLACK")
        ("algorithm,a", po::value<std::string>()->default_value("RulebookApex"), "solvers (BOA, PPA, Apex, Planning or RulebookApex search)")
        ("cutoffTime,t", po::value<int>()->default_value(300), "cutoff time (seconds)")
        ("output,o", po::value<std::string>()->required(), "Name of the output file")
        ("logging_file", po::value<std::string>()->default_value(""), "logging file" )
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);
    srand((int)time(0));

    if (vm["query"].as<std::string>() != ""){
        if (vm["start"].as<int>() != -1 || vm["goal"].as<int>() != -1){
            std::cerr << "query file and start/goal cannot be given at the same time !" << std::endl;
            return -1;
        }
    }

    LoggerPtr logger = nullptr;

    if (vm["logging_file"].as<std::string>() != ""){
        logger = new Logger(vm["logging_file"].as<std::string>());
    }

    // Load files
    size_t graph_size;
    std::vector<Edge> edges;

    for (auto file:objective_files){
        std::cout << file << std::endl;
    }


    if (load_gr_files(objective_files, edges, graph_size) == false) {
        std::cout << "Failed to load gr files" << std::endl;
        return -1;
    }

    std::cout << "Graph Size: " << graph_size << std::endl;

    // Build graphs
    MergeStrategy ms = DEFAULT_MERGE_STRATEGY;
    alg_variant = vm["merge"].as<std::string>();

    if (vm["merge"].as<std::string>() != "" && vm["algorithm"].as<std::string>()!= "Apex"){
        alg_variant = "";
        std::cout << "WARNING: merge strategy with non-apex search" << std::endl;
    }else if(vm["merge"].as<std::string>() == "SMALLER_G2"){
        ms = MergeStrategy::SMALLER_G2;
    }else if(vm["merge"].as<std::string>() == "SMALLER_G2_FIRST"){
        ms = MergeStrategy::SMALLER_G2_FIRST;
    }else if(vm["merge"].as<std::string>() == "RANDOM"){
        ms = MergeStrategy::RANDOM;
    }else if(vm["merge"].as<std::string>() == "MORE_SLACK"){
        ms = MergeStrategy::MORE_SLACK;
    }else if(vm["merge"].as<std::string>() == "REVERSE_LEX"){
        ms = MergeStrategy::REVERSE_LEX;
    }else{
        std::cerr << "unknown merge strategy" << std::endl;
    }


    if (vm["query"].as<std::string>() != ""){
        run_query(graph_size, edges, vm["query"].as<std::string>(), vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>());
    } else{
        single_run_map(graph_size, edges, vm["start"].as<int>(), vm["goal"].as<int>(), vm["output"].as<std::string>(), vm["algorithm"].as<std::string>(), ms, logger, vm["eps"].as<double>(), vm["cutoffTime"].as<int>());
    }

    delete(logger);


    // test_manual_graph();

    // test_random_graph();

    return 0;
}
/*
1 2 [0 55 69]
1 4 [0 55 54]
1 8 [0 21 100]
1 9 [0 48 13]
1 11 [0 88 94]
1 13 [0 34 45]
1 14 [0 91 32]
2 5 [0 68 92]
2 7 [0 20 68]
2 13 [0 4 28]
2 14 [0 15 80]
3 4 [0 22 2]
3 5 [0 20 68]
3 6 [0 68 69]
3 7 [0 37 92]
3 9 [0 39 58]
3 11 [0 58 73]
3 12 [0 75 19]
3 15 [0 4 100]
4 5 [0 91 54]
4 6 [0 70 67]
4 9 [0 76 45]
4 12 [0 100 96]
4 13 [0 45 59]
5 6 [0 21 3]
5 11 [0 47 43]
6 7 [0 62 3]
6 8 [0 37 100]
6 9 [0 82 3]
6 10 [0 4 1]
6 12 [0 93 55]
6 13 [0 84 99]
6 14 [0 66 95]
7 9 [0 30 38]
7 10 [0 53 2]
7 12 [0 45 22]
7 14 [0 73 22]
7 15 [0 46 70]
8 9 [0 81 80]
8 10 [0 72 74]
8 12 [0 52 68]
8 15 [1 82 48]
9 10 [0 34 74]
9 12 [0 84 41]
9 13 [0 4 90]
10 11 [0 92 16]
10 15 [0 85 62]
11 12 [0 3 20]
12 14 [0 19 100]
12 15 [0 90 50]
13 14 [0 70 92]
 */

/*
1 3 [0 104 838]
1 4 [0 893 547]
1 5 [0 966 201]
2 3 [0 431 774]
2 4 [0 814 147]
3 4 [0 140 772]
3 5 [1 436 308]
4 5 [0 659 549]
 */