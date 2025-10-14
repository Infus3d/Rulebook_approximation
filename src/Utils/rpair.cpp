
#include "Utils/Definitions.h"

// return true if node dom ape x
// bool is_dominated_dr(NodePtr pseudoR, NodePtr node){
//     for (int i = 1; i < pseudoR->f.size(); i ++ ){
//       if (node->f[i] > pseudoR->f[i]){
//         return false;
//       }
//     }
//   // TODO MORE
//   return true;
// }
//
// // (dominatee ,dominator)
// bool is_dominated_dr(NodePtr pseudoR, NodePtr node, const EPS eps){
//     for (int i = 1; i < pseudoR->f.size(); i ++ ){
//         if (node->f[i] > (1 + eps[i]) * pseudoR->f[i]){
//             return false;
//         }
//     }
//     // TODO MORE
//     return true;
// }
//
//
// bool is_bounded(NodePtr pseudoR, NodePtr node,  const EPS eps){
//     for (int i = 0; i < pseudoR->f.size(); i ++ ){
//         if (node->f[i] > (1 + eps[i]) * pseudoR->f[i]){
//             return false;
//         }
//     }
//     return true;
// }

RealizationPair::RealizationPair(const RealizationPairPtr parent, const Edge&  edge): h(parent->h), rulebook_graph(parent->rulebook_graph){
    size_t next_id = edge.target;
    id = next_id;

    std::vector<size_t> new_pseudoR_g(parent->pseudoR->g);
    std::vector<size_t> new_g(parent->realization->g);

    for (int i = 0; i < new_pseudoR_g.size(); i ++){
        new_pseudoR_g[i] += edge.cost[i];
        new_g[i] += edge.cost[i];
    }
    auto new_h = h(next_id);
    this->pseudoR = std::make_shared<Node>(next_id, new_pseudoR_g, new_h, nullptr, parent->pseudoR->rulebook_graph);
    this->realization = std::make_shared<Node>(next_id, new_g, new_h, nullptr, parent->realization->rulebook_graph);
}

bool RealizationPair::update_nodes_by_merge_if_bounded(const RealizationPairPtr &other, const std::vector<double> eps, MergeStrategy s){
  // Returns true on successful merge and false if it failure
  if (this->id != other->id) {
    return false;
  }

  NodePtr new_pseudoR = std::make_shared<Node>(this->pseudoR->id, this->pseudoR->g, this->pseudoR->h);
  NodePtr new_realization = nullptr;

  // Merge pseudoR
  for (int i = 0; i < other->pseudoR->g.size(); i ++){
    if (other->pseudoR->g[i] < new_pseudoR->g[i]){
      new_pseudoR->g[i] = other->pseudoR->g[i];
      new_pseudoR->f[i] = other->pseudoR->f[i];
    }
  }

  // choose a path node
  if (s == MergeStrategy::RANDOM){
    if (this->rulebook_graph.dominates(this->realization->f,new_pseudoR->f, eps)){
      new_realization = this->realization;
    }
    if (this->rulebook_graph.dominates(other->realization->f, new_pseudoR->f, eps)){
      if (new_realization == nullptr){
        new_realization = other->realization;
      }else{
        if (rand() % 2 == 1){
          new_realization = other->realization;
        }
      }
    }
    if (new_realization == nullptr){
      return false;
    }
  } else if (s == MergeStrategy::REVERSE_LEX){
    new_realization = this->realization;
    for (int i = 0; i < new_pseudoR->g.size(); i++){
      int i_r = new_pseudoR->g.size() - 1 - i;
      if (this->realization->g[i_r] != other->realization->g[i_r]){
        new_realization = this->realization->g[i_r] < other->realization->g[i_r] ?this->realization: other->realization;
        break;
      }
    }
    if (! is_bounded(new_pseudoR, new_realization, eps)){
      return false;
    }
  }else{
    std::cerr << "merge strategy not known" << std::endl;
    exit(-1);
  }

    // Check if path pair is bounded after merge - if not the merge is illegal
  // if ((((1+eps[0])*new_top_left->g[0]) < new_bottom_right->g[0]) ||
  //     (((1+eps[1])*new_bottom_right->g[1]) < new_top_left->g[1])) {
  //   return false;
  // }

  this->pseudoR = new_pseudoR;
  this->realization = new_realization;
  return true;
}

bool RealizationPair::update_pseudoR_by_merge_if_bounded(const NodePtr &other_pseudoR, const std::vector<double> eps){
  // if (!is_bounded(other_pseudoR, path_node, eps)){
  //   return false;
  // }
  NodePtr new_pseudoR = std::make_shared<Node>(this->pseudoR->id, this->pseudoR->g, this->pseudoR->h);
  bool update_flag = false;
  // Merge pseudoR
  for (int i = 0; i < other_pseudoR->g.size(); i ++){
    if (other_pseudoR->f[i] < new_pseudoR->f[i]){
      new_pseudoR->g[i] = other_pseudoR->f[i];
      new_pseudoR->f[i] = other_pseudoR->f[i];
      update_flag = true;
    }
  }
  if (!this->rulebook_graph.dominates(realization->f, new_pseudoR->f, eps)) {
    return false;
  }
  if (update_flag){
    pseudoR = new_pseudoR;
  }
  return true;
}



bool RealizationPair::more_than_full_cost::operator()(const RealizationPairPtr &a, const RealizationPairPtr &b) const {
  std::vector<size_t> rules = a->rulebook_graph.get_ordered_rules();
  for (size_t i = 0; i + 1 < rules.size(); i ++) {
    if (a->pseudoR->f[rules[i]] != b->pseudoR->f[rules[i]]) {
      return (a->pseudoR->f[rules[i]] > b->pseudoR->f[rules[i]]);
    }
  }
  return (a->pseudoR->f[rules.back()] > b->pseudoR->f[rules.back()]);
}


std::ostream& operator<<(std::ostream &stream, const RealizationPair &ap) {
  // Printed in JSON format
  stream << "{" << *(ap.pseudoR) << ", " << *(ap.realization) << "}";
  return stream;
}