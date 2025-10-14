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

using namespace std;

const std::string resource_path = "resources/";
const std::string output_path = "output/";
const MergeStrategy DEFAULT_MERGE_STRATEGY = MergeStrategy::RANDOM;
std::string alg_variant = "";


// Simple example to demonstarte the usage of the algorithm

void single_run_map(size_t graph_size, AdjacencyMatrix& graph, AdjacencyMatrix&inv_graph, size_t source, size_t target, std::ofstream& output, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, unsigned int time_limit) {
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
    }else if (algorithm == "Rulebook") {
        RulebookGraph rgraph(3);
        rgraph.add_relationship(0, 1);
        rgraph.add_relationship(0, 2);

        assert(graph.get_num_of_objectives() == 3);

        rgraph.calculate_quotient_graph();

        EPS eps_vec ({0.0, 0.01, 0.01});
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


    output << algorithm << "-" << alg_variant << " (" << eps << ")" << "\t"
           << source << "\t" << target << "\t"
           << num_gen << "\t"
           << num_exp << "\t"
           << solutions.size() << "\t"
           << (double) runtime / CLOCKS_PER_SEC
           << std::endl;

    std::cout << "-----End Single Example-----" << std::endl;
}

void single_run_map(size_t graph_size, std::vector<Edge> & edges, size_t source, size_t target, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit) {

    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);
    std::ofstream stats;
    stats.open(output_path + output_file, std::fstream::app);

    single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit);
    stats.close();
}

void run_query(size_t graph_size, std::vector<Edge> & edges, std::string query_file, std::string output_file, std::string algorithm, MergeStrategy ms, LoggerPtr logger, double eps, int time_limit) {
    std::ofstream stats;
    stats.open(output_path + output_file, std::fstream::app);


    std::vector<std::pair<size_t, size_t>> queries;
    if (load_queries(query_file, queries) == false) {
        std::cout << "Failed to load queries file" << std::endl;
        return;
    }

    // Build graphs
    AdjacencyMatrix graph(graph_size, edges);
    AdjacencyMatrix inv_graph(graph_size, edges, true);

    size_t query_count = 0;
    for (auto iter = queries.begin(); iter != queries.end(); ++iter) {

        query_count++;
        std::cout << "Started Query: " << query_count << "/" << queries.size() << std::endl;
        size_t source = iter->first;
        size_t target = iter->second;

        single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit);
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
        ("algorithm,a", po::value<std::string>()->default_value("Apex"), "solvers (BOA, PPA or Apex search)")
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

    // std::vector<Edge> edges;
    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_int_distribution<> distr(1, 10);
    // std::uniform_int_distribution<> fifty(1, 2);
    // size_t graph_size = 20;
    //
    // for (int i=1; i<=graph_size; i++) {
    //     for (int j=i+1; j<=graph_size; j++) {
    //         if (fifty(gen) == 1) continue;
    //         vector<size_t> cost;
    //         for (int t=0; t<3; t++) {
    //             cost.push_back(distr(gen));
    //         }
    //         edges.push_back(Edge(i, j, cost));
    //         std::cout << i << " " << j << " [" << cost[0] << " " << cost[1] << " " << cost[2] << "] " << std::endl;
    //     }
    // }
    //
    // AdjacencyMatrix graph(graph_size, edges);
    // AdjacencyMatrix inv_graph(graph_size, edges, true);
    //
    // std::ofstream stats;
    // stats.open(output_path + "output.txt", std::fstream::app);
    //
    // LoggerPtr logger = nullptr;
    //
    // size_t source = 1;
    // size_t target = 2;
    // string algorithm = "Rulebook";
    //
    // MergeStrategy ms = MergeStrategy::RANDOM;
    // double eps = 0.0;
    //
    // size_t time_limit = 300;
    //
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
    //
    // cout << "Rulebook:" << std::endl;
    // single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit);
    //
    // algorithm = "Apex";
    // cout << "\n\nApex:" << std::endl;
    // single_run_map(graph_size, graph, inv_graph, source, target, stats, algorithm, ms, logger, eps, time_limit);

    return 0;
}
