#ifndef CPFA_PHYSICAL_CORE_H
#define CPFA_PHYSICAL_CORE_H

#include "CPFA_PhysicalNavigation.h"
#include <cmath>
#include <cstddef>
#include "CPFA_PhysicalPheromoneSelector.h"
#include <vector>
class CPFAPhysicalCore {
public:
    enum class State {
        DEPARTING = 0,
        SEARCHING = 1,
        RETURNING = 2,
        SURVEYING = 3
    };

    State state = State::DEPARTING;

    CPFATarget target{};
    CPFATarget nest{};

    bool isInformed = false;
    bool isHoldingFood = false;
    bool isUsingSiteFidelity = false;
    bool isGivingUpSearch = false;

        double probabilityOfSwitchingToSearching = 0.0;
    double searchStepSize = 0.16;

    void departing(const CPFAPose& pose,
               double randomNumber,
               double gaussianAngle,
               bool halfSecondTick,
               double randomX,
               double randomY) {

        if (!pose.valid) return;

        const double distance = std::hypot(
            target.x - pose.x,
            target.y - pose.y
        );

        if (!isInformed && halfSecondTick) {
            // Carlos: probabilistic transition to SEARCHING.
            if (randomNumber < probabilityOfSwitchingToSearching &&
                (target.x != 0.0 || target.y != 0.0)) {

                state = State::SEARCHING;

                const double angle = pose.heading + gaussianAngle;
                target.x = pose.x + searchStepSize * std::cos(angle);
                target.y = pose.y + searchStepSize * std::sin(angle);
            }

                        // Carlos: choose a new random search location.
            else if (distance < 0.01) {
                setRandomSearchLocation(
    randomNumber,
    randomX,
    randomY
);
            }
        }

        if (isInformed && distance < 0.01) {
            state = State::SEARCHING;

            if (isUsingSiteFidelity) {
                isUsingSiteFidelity = false;
                // Original SetFidelityList() pending.
            }
        }
    } 
        double probabilityOfReturningToNest = 0.0;
    double uninformedSearchVariation = 0.0;
    double rateOfInformedSearchDecay = 0.0;
    unsigned int searchTime = 0;

    void searching(const CPFAPose& pose,
               double randomNumber,
               double gaussianSample,
               bool foodDetected) {if (!pose.valid || state != State::SEARCHING) return;

        // Food detection is supplied by a separate verified system.
        if (foodDetected) {
            isHoldingFood = true;
            state = State::RETURNING;
            target = nest;
            return;
        }

        if (isHoldingFood) return;

        const double dx = target.x - pose.x;
        const double dy = target.y - pose.y;

        // Preserve Carlos's original squared-distance comparison.
        if (dx * dx + dy * dy < 0.01) {

            if (randomNumber < probabilityOfReturningToNest) {
                isGivingUpSearch = true;
                isUsingSiteFidelity = false;
                state = State::RETURNING;
                target = nest;
                return;
            }

            double rotation = gaussianSample;

            if (isInformed) {
                // Informed search decay requires the original
                // GetExponentialDecay() implementation.
                // Do not substitute uninformed behavior.
                return;
            }

            const double angle = pose.heading + rotation;

            target.x = pose.x + searchStepSize * std::cos(angle);
            target.y = pose.y + searchStepSize * std::sin(angle);
        }
    } 
        static double getPoissonCDF(double k, double lambda) {
        double sumAccumulator = 1.0;
        double factorialAccumulator = 1.0;

        for (size_t i = 1; i <= std::floor(k); i++) {
            factorialAccumulator *= i;
            sumAccumulator += std::pow(lambda, i) /
                              factorialAccumulator;
        }

        return std::exp(-lambda) * sumAccumulator;
    } 
    // Per-robot state. Shared pheromones are passed by reference.
bool updateFidelity = false;
CPFATarget siteFidelityPosition{};
std::vector<CPFATarget> trailToShare;
std::vector<CPFATarget> trailToFollow;

double resourceDensity = 0.0;
double rateOfLayingPheromone = 0.0;
double rateOfSiteFidelity = 0.0;
double rateOfPheromoneDecay = 0.0;

unsigned int numTargetsCollected = 0;

// Carlos CPFA: select a random forage boundary
double forageMinX = -1.0;
double forageMaxX = 1.0;
double forageMinY = -1.0;
double forageMaxY = 1.0;

void setRandomSearchLocation(
    double randomWall,
    double randomX,
    double randomY
) {
    if (randomWall < 0.25) {
        target = {
            forageMinX + randomX * (forageMaxX - forageMinX),
            forageMaxY
        };
    } else if (randomWall < 0.50) {
        target = {
            forageMinX + randomX * (forageMaxX - forageMinX),
            forageMinY
        };
    } else if (randomWall < 0.75) {
        target = {
            forageMaxX,
            forageMinY + randomY * (forageMaxY - forageMinY)
        };
    } else {
        target = {
            forageMinX,
            forageMinY + randomY * (forageMaxY - forageMinY)
        };
    }
}
void returning(
    const CPFAPose& pose,
    bool inNest,
    double randomLay,
    double randomFidelity,
    double randomPheromone,
    CPFATarget randomSearchTarget,
    double timeSeconds,
    std::vector<CPFAPhysicalPheromone>& sharedPheromones
) {
    if (!pose.valid || state != State::RETURNING) return;

    if (!inNest) {
        target = nest;
        return;
    }

    const double layProbability =
        getPoissonCDF(resourceDensity, rateOfLayingPheromone);

    const double fidelityProbability =
        getPoissonCDF(resourceDensity, rateOfSiteFidelity);

    if (isHoldingFood) {
        ++numTargetsCollected;

        if (updateFidelity && randomLay < layProbability) {
            trailToShare.push_back(nest);

            sharedPheromones.emplace_back(
                siteFidelityPosition,
                trailToShare,
                timeSeconds,
                rateOfPheromoneDecay
            );
        }

        trailToShare.clear();
    }

    if (updateFidelity && randomFidelity < fidelityProbability) {
        target = siteFidelityPosition;
        isInformed = true;
    } else {
        const int index = CPFAPhysicalPheromoneSelector::select(
            sharedPheromones, randomPheromone
        );

        if (index >= 0) {
            target = sharedPheromones[index].location;
            trailToFollow = sharedPheromones[index].trail;
            isInformed = true;
            isUsingSiteFidelity = false;
        } else {
            target = randomSearchTarget;
            isInformed = false;
            isUsingSiteFidelity = false;
        }
    }

    isGivingUpSearch = false;
    isHoldingFood = false;
    state = State::DEPARTING;
} 
// Carlos CPFA: surveying four directions
unsigned int surveyCount = 0;

void surveying(
    const CPFAPose& pose,
    double targetAngleTolerance = 0.1
) {
    if (!pose.valid || state != State::SURVEYING) return;

    if (surveyCount >= 4) {
        surveyCount = 0;
        state = State::RETURNING;
        target = nest;
        return;
    }

    constexpr double pi = 3.141592653589793;
    const double rotation = surveyCount * pi / 2.0;

    double headingDifference = rotation - pose.heading;

    while (headingDifference > pi) {
        headingDifference -= 2.0 * pi;
    }

    while (headingDifference < -pi) {
        headingDifference += 2.0 * pi;
    }

    if (std::abs(headingDifference) < targetAngleTolerance) {
        ++surveyCount;
    }

    target.x = pose.x + searchStepSize * std::cos(rotation);
    target.y = pose.y + searchStepSize * std::sin(rotation);
}
bool headingToNest() const {
        return state == State::RETURNING;
    }

    bool isAtTarget(const CPFAPose& pose) const {
        if (!pose.valid) return false;

        const double dx = target.x - pose.x;
        const double dy = target.y - pose.y;

        // Carlos's original distance tolerance.
        return std::hypot(dx, dy) < 0.01;
    }
};

#endif