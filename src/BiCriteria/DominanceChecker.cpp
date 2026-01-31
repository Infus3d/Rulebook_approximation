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

void LocalCheckRulebook::add_node(RealizationPairPtr ap) {
    auto id = ap->id;
    const RulebookGraph& rulebook_graph = ap->rulebook_graph;
    if (!transfer_Tr_to_Sh(ap)) {
        for (auto it = G_tr[id].begin(); it != G_tr[id].end(); ) {
            if (rulebook_graph.is_dominated_Tr((*it)->pseudoR->f, ap->pseudoR->f)) {
                it = G_tr[id].erase(it);
            } else {
                it ++;
            }
        }
    }

    G_tr[id].push_front(ap);
}


bool LocalCheckRulebook::is_dominated(RealizationPairPtr node, bool transferFlag = false) {
    auto id = node->id;
    const RulebookGraph& rulebook_graph = node->rulebook_graph;

    if (!transferFlag || !transfer_Tr_to_Sh(node)) {
        for (auto ap : G_tr[id]) {
            if (rulebook_graph.is_dominated_Tr(node->pseudoR->f, ap->pseudoR->f)) {
                return true;
            }
        }
    }

    for (auto ap : G_sh[id]) {
        if (rulebook_graph.is_dominated_Sh(node->pseudoR->f, ap->pseudoR->f)) {
            return true;
        }
    }

    return false;
}

bool LocalCheckRulebook::transfer_Tr_to_Sh(RealizationPairPtr node) {
    auto id = node->id;
    const RulebookGraph& rulebook_graph = node->rulebook_graph;
    auto ordered_rules = node->rulebook_graph.get_ordered_rules();
    auto topmost_rule = ordered_rules[0];

    if (G_tr[id].size() > 0 && (*G_tr[id].begin())->pseudoR->f[topmost_rule] < node->pseudoR->f[topmost_rule]) {
        for (auto tr : G_tr[id]) {
            for (auto it = G_sh[id].begin(); it != G_sh[id].end(); ){
                if (rulebook_graph.is_dominated_Tr((*it)->pseudoR->f, tr->pseudoR->f)) {
                    it = G_sh[id].erase(it);
                } else {
                    it ++;
                }
            }
            G_sh[id].push_front(tr);
        }
        G_tr[id].clear();
        return true;
    }
    assert(G_tr[id].size() == 0 || (*G_tr[id].begin())->pseudoR->f[topmost_rule] == node->pseudoR->f[topmost_rule]);
    return false;
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

bool SolutionCheckRulebook::is_dominated(RealizationPairPtr node, bool transferFlag = false) {
    auto id = node->id;
    const RulebookGraph& rulebook_graph = node->rulebook_graph;

    if (!transferFlag || !transfer_Tr_to_Sh(node)) {
        for (auto ap : solutions_tr) {
            if (ap->update_pseudoR_by_merge_if_bounded(node->pseudoR, eps)) {
                return true;
            }
        }
    }

    for (auto ap : solutions_sh) {
        if (ap->update_pseudoR_by_merge_if_bounded(node->pseudoR, eps)) {
            return true;
        }
    }

    return false;
}

bool SolutionCheckRulebook::transfer_Tr_to_Sh(RealizationPairPtr node) {
    auto id = node->id;
    const RulebookGraph& rulebook_graph = node->rulebook_graph;
    auto ordered_rules = node->rulebook_graph.get_ordered_rules();
    auto topmost_rule = ordered_rules[0];

    if (solutions_tr.size() > 0 && (*solutions_tr.begin())->pseudoR->f[topmost_rule] < node->pseudoR->f[topmost_rule]) {
        for (auto tr : solutions_tr) {
            for (auto it = solutions_sh.begin(); it != solutions_sh.end(); ){
                if (rulebook_graph.is_dominated_Tr((*it)->pseudoR->f, tr->pseudoR->f)) {
                    it = solutions_sh.erase(it);
                } else {
                    it ++;
                }
            }
            solutions_sh.push_front(tr);
        }
        solutions_tr.clear();
        return true;
    }
    assert(solutions_tr.size() == 0 || (*solutions_tr.begin())->pseudoR->f[topmost_rule] == node->pseudoR->f[topmost_rule]);
    return false;
}

void SolutionCheckRulebook::add_node(RealizationPairPtr node) {
    auto id = node->id;
    const RulebookGraph& rulebook_graph = node->rulebook_graph;
    if (!transfer_Tr_to_Sh(node)) {
        for (auto it = solutions_tr.begin(); it != solutions_tr.end(); ) {
            if (rulebook_graph.is_dominated_Tr((*it)->pseudoR->f, node->pseudoR->f)) {
                it = solutions_tr.erase(it);
            } else {
                it ++;
            }
        }
    }

    solutions_tr.push_front(node);
}