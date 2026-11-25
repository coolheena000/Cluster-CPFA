#ifndef CPFA_PHYSICAL_PHEROMONE_SELECTOR_H
#define CPFA_PHYSICAL_PHEROMONE_SELECTOR_H

#include "CPFA_PhysicalPheromone.h"
#include <vector>
#include <cstddef>

class CPFAPhysicalPheromoneSelector {
public:
    static int select(
        const std::vector<CPFAPhysicalPheromone>& pheromones,
        double randomUnit
    ) {
        double totalWeight = 0.0;

        for (const auto& p : pheromones) {
            if (p.isActive()) {
                totalWeight += p.getWeight();
            }
        }

        if (totalWeight <= 0.0 ||
            randomUnit < 0.0 || randomUnit >= 1.0) {
            return -1;
        }

        double randomWeight = randomUnit * totalWeight;

        for (std::size_t i = 0; i < pheromones.size(); ++i) {
            if (!pheromones[i].isActive()) continue;

            if (randomWeight < pheromones[i].getWeight()) {
                return static_cast<int>(i);
            }

            randomWeight -= pheromones[i].getWeight();
        }

        return -1;
    }
};

#endif