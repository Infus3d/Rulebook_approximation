#include "DominanceChecker.h"


bool SolutionCheck::is_dominated(ApexPathPairPtr node){
    if (last_solution == nullptr){
        return false;
    }
    if (is_bounded(node->apex, last_solution->path_node, eps)){
        assert(last_solution->update_apex_by_merge_if_bounded(node->apex, eps));
        // std::cout << "solution dom" << std::endl;
        return true;
    }
    return false;
}


bool LocalCheck::is_dominated(ApexPathPairPtr node){
    return (node->apex->g[1] >= min_g2[node->id]);
}

void LocalCheck::add_node(ApexPathPairPtr ap){
    auto id = ap->id;
    assert(min_g2[ap->id] > ap->apex->g[1]);
    min_g2[ap->id] = ap->apex->g[1];
}

bool LocalCheckLinear::is_dominated(ApexPathPairPtr node){
    for (auto ap:min_g2[node->id]) {
        if (is_dominated_dr(node->apex, ap->apex)){
            assert(node->apex->f[0] >= ap->apex->f[0]);
            return true;
        }
    }
    return false;
}

void LocalCheckLinear::add_node(ApexPathPairPtr ap){
    auto id = ap->id;
    for (auto it = min_g2[id].begin(); it != min_g2[id].end(); ){
        // TODO remove it for performance
        assert(! is_dominated_dr(ap->apex, (*it)->apex  ));
        if (is_dominated_dr((*it)->apex, ap->apex)){
            it = min_g2[id].erase(it);
        } else {
            it ++;
        }
    }

    min_g2[ap->id].push_front(ap);
}

bool SolutionCheckLinear::is_dominated(ApexPathPairPtr node){
    for (auto ap: solutions){
        // if (is_bounded(node->apex, ap->path_node, eps)){
        if (ap->update_apex_by_merge_if_bounded(node->apex, eps)){
            // assert(ap->update_apex_by_merge_if_bounded(node->apex, eps));
            return true;
        }
    }
    return false;
}

void SolutionCheckLinear::add_node(ApexPathPairPtr ap){
    for (auto it = solutions.begin(); it != solutions.end(); ){
        if (is_dominated_dr((*it)->path_node, ap->path_node)){
            it = solutions.erase(it);
        } else {
            it ++;
        }
    }
    solutions.push_front(ap);
}

/*************** Rulebook dominance checks ***************/

void LocalCheckRulebook::add_node(RealizationPairPtr rp) {
    auto id = rp->id;
    const RulebookGraph& rulebook_graph = rp->rulebook_graph;
    if (!transfer_equal_to_less_than(rp)) {
        for (auto it = G_equal[id].begin(); it != G_equal[id].end(); ) {
            if (rulebook_graph.is_dominated_equal((*it)->rule_apex->f, rp->rule_apex->f)) {
                it = G_equal[id].erase(it);
            } else {
                it ++;
            }
        }
    }

    G_equal[id].push_front(rp);
}


bool LocalCheckRulebook::is_dominated(RealizationPairPtr rp, bool transferFlag = false) {
    auto id = rp->id;
    const RulebookGraph& rulebook_graph = rp->rulebook_graph;

    if (!transferFlag || !transfer_equal_to_less_than(rp)) {
        for (auto ap : G_equal[id]) {
            if (rulebook_graph.is_dominated_equal(rp->rule_apex->f, ap->rule_apex->f)) {
                return true;
            }
        }
    }

    for (auto ap : G_less_than[id]) {
        if (rulebook_graph.is_dominated_less_than(rp->rule_apex->f, ap->rule_apex->f)) {
            return true;
        }
    }

    return false;
}

bool LocalCheckRulebook::transfer_equal_to_less_than(RealizationPairPtr rp) {
    auto id = rp->id;
    const RulebookGraph& rulebook_graph = rp->rulebook_graph;
    auto ordered_rules = rp->rulebook_graph.get_ordered_rules();
    auto topmost_rule = ordered_rules[0];

    if (G_equal[id].size() > 0 && (*G_equal[id].begin())->rule_apex->f[topmost_rule] < rp->rule_apex->f[topmost_rule]) {
        for (auto tr : G_equal[id]) {
            for (auto it = G_less_than[id].begin(); it != G_less_than[id].end(); ){
                if (rulebook_graph.is_dominated_equal((*it)->rule_apex->f, tr->rule_apex->f)) {
                    it = G_less_than[id].erase(it);
                } else {
                    it ++;
                }
            }
            G_less_than[id].push_front(tr);
        }
        G_equal[id].clear();
        return true;
    }
    assert(G_equal[id].size() == 0 || (*G_equal[id].begin())->rule_apex->f[topmost_rule] == rp->rule_apex->f[topmost_rule]);
    return false;
}


bool SolutionCheckRulebook::is_dominated(RealizationPairPtr rp, bool transferFlag = false) {
    auto id = rp->id;
    const RulebookGraph& rulebook_graph = rp->rulebook_graph;

    if (!transferFlag || !transfer_equal_to_less_than(rp)) {
        for (auto ap : solutions_equal) {
            if (ap->update_rule_apex_by_merge_if_bounded(rp->rule_apex, eps)) {
                return true;
            }
        }
    }

    for (auto ap : solutions_less_than) {
        if (ap->update_rule_apex_by_merge_if_bounded(rp->rule_apex, eps)) {
            return true;
        }
    }

    return false;
}

bool SolutionCheckRulebook::transfer_equal_to_less_than(RealizationPairPtr rp) {
    auto id = rp->id;
    const RulebookGraph& rulebook_graph = rp->rulebook_graph;
    auto ordered_rules = rp->rulebook_graph.get_ordered_rules();
    auto topmost_rule = ordered_rules[0];

    if (solutions_equal.size() > 0 && (*solutions_equal.begin())->rule_apex->f[topmost_rule] < rp->rule_apex->f[topmost_rule]) {
        for (auto tr : solutions_equal) {
            for (auto it = solutions_less_than.begin(); it != solutions_less_than.end(); ){
                if (rulebook_graph.is_dominated_equal((*it)->rule_apex->f, tr->rule_apex->f)) {
                    it = solutions_less_than.erase(it);
                } else {
                    it ++;
                }
            }
            solutions_less_than.push_front(tr);
        }
        solutions_equal.clear();
        return true;
    }
    assert(solutions_equal.size() == 0 || (*solutions_equal.begin())->rule_apex->f[topmost_rule] == rp->rule_apex->f[topmost_rule]);
    return false;
}

void SolutionCheckRulebook::add_node(RealizationPairPtr node) {
    auto id = node->id;
    const RulebookGraph& rulebook_graph = node->rulebook_graph;
    if (!transfer_equal_to_less_than(node)) {
        for (auto it = solutions_equal.begin(); it != solutions_equal.end(); ) {
            if (rulebook_graph.is_dominated_equal((*it)->rule_apex->f, node->rule_apex->f)) {
                it = solutions_equal.erase(it);
            } else {
                it ++;
            }
        }
    }

    solutions_equal.push_front(node);
}


bool LocalCheckRulebookBasic::is_dominated(RealizationPairPtr rp, bool transferFlag = false) {
    auto id = rp->id;
    const RulebookGraph& rulebook_graph = rp->rulebook_graph;
    for (auto ap : G[id]) {
        if (rulebook_graph.dominates(ap->rule_apex->f, rp->rule_apex->f)) {
            return true;
        }
    }
    return false;
}

void LocalCheckRulebookBasic::add_node(RealizationPairPtr node) {
    auto id = node->id;
    const RulebookGraph& rulebook_graph = node->rulebook_graph;
    for (auto it = G[id].begin(); it != G[id].end(); ) {
        if (rulebook_graph.dominates(node->rule_apex->f, (*it)->rule_apex->f)) {
            it = G[id].erase(it);
        } else {
            it ++;
        }
    }
    G[id].push_front(node);
}


bool SolutionCheckRulebookBasic::is_dominated(RealizationPairPtr rp, bool transferFlag = false) {
    auto id = rp->id;
    const RulebookGraph& rulebook_graph = rp->rulebook_graph;

    for (auto ap : solutions) {
        if (ap->update_rule_apex_by_merge_if_bounded(rp->rule_apex, eps)) {
            return true;
        }
    }
    return false;
}
void SolutionCheckRulebookBasic::add_node(RealizationPairPtr node) {
    auto id = node->id;
    const RulebookGraph& rulebook_graph = node->rulebook_graph;
    for (auto it = solutions.begin(); it != solutions.end(); ) {
        if (rulebook_graph.dominates(node->rule_apex->f, (*it)->rule_apex->f)) {
            it = solutions.erase(it);
        } else {
            it ++;
        }
    }
    solutions.push_front(node);
}