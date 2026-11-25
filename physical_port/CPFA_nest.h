#ifndef CPFA_NEST_H
#define CPFA_NEST_H

#include "Pheromone.h"
#include <map>
#include <string>

// Shared data at the nest (Carlos: source/CPFA/CPFA_loop_functions).
// It has no food list: a robot must detect food itself.
class CPFA_nest {
	public:
		// Values from sim_check/cpfa_baseline_clustered.xml.
		double ProbabilityOfSwitchingToSearching = 0.504;
		double ProbabilityOfReturningToNest      = 0.001;
		double UninformedSearchVariation         = 7.0 * M_PI / 180.0;
		double RateOfInformedSearchDecay         = 0.28;
		double RateOfSiteFidelity                = 4.27;
		double RateOfLayingPheromone             = 3.75;
		double RateOfPheromoneDecay              = 0.03;

		Vector2 NestPosition;
		double  NestRadiusSquared = 0.25 * 0.25;

		std::vector<Pheromone>         PheromoneList;
		std::map<std::string, Vector2> FidelityList;
		unsigned int                   currNumCollectedFood = 0;

		// Call every 0.5 s.
		void UpdatePheromoneList(double timeInSeconds);
};

#endif
