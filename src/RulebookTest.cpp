
#include <iostream>
#include <ostream>

#include "Utils/RulebookGraph.h"

int main() {
    RulebookGraph rgraph(5);
    rgraph.add_relationship(0, 1);
    rgraph.add_relationship(1, 2);
    rgraph.add_relationship(2, 0);


    rgraph.add_relationship(3, 4);
    rgraph.add_relationship(4, 3);

    std::vector<size_t> ordered_rules = rgraph.get_ordered_rules();
    for (const auto &rule : ordered_rules) {
        std::cout << "Rule: " << rule << std::endl;
    }
    return 0;
}