#ifndef CPFA_PHYSICAL_PHEROMONE_H
#define CPFA_PHYSICAL_PHEROMONE_H

#include "CPFA_PhysicalNavigation.h"
#include <vector>
#include <cmath>

class CPFAPhysicalPheromone {
public:
    CPFATarget location;
    std::vector<CPFATarget> trail;

    double lastUpdated;
    double decayRate;
    double weight = 1.0;
    double threshold = 0.001;

    CPFAPhysicalPheromone(
        CPFATarget newLocation,
        std::vector<CPFATarget> newTrail,
        double newTime,
        double newDecayRate
    )
        : location(newLocation),
          trail(newTrail),
          lastUpdated(newTime),
          decayRate(newDecayRate) {}

    void update(double time) {
        weight *= std::exp(-decayRate * (time - lastUpdated));
        lastUpdated = time;
    }

    void deactivate() {
        weight = 0.0;
    }

    bool isActive() const {
        return weight > threshold;
    }

    double getWeight() const {
        return weight;
    }
};

#endif