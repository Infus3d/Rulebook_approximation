#pragma once

#include <vector>
#include "Utils/Definitions.h"
#include "Utils/Logger.h"
#include "AbstractSolver.h"


class NAMOAdr: public AbstractSolver {
protected:

public:

    NAMOAdr(const AdjacencyMatrix &adj_matrix, EPS eps, const LoggerPtr logger=nullptr):     AbstractSolver(adj_matrix, eps, logger) {}

    virtual std::string get_solver_name() {return "NAMOAdr"; }

    void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit=UINT_MAX) override;

};


