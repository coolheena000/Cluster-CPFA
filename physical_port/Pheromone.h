#ifndef PHEROMONE_H
#define PHEROMONE_H

#include "Vector2.h"
#include <vector>

// Carlos: source/Base/Pheromone.h
class Pheromone {
	public:
		Pheromone(Vector2 newLocation, std::vector<Vector2> newTrail, double newTime, double newDecayRate);

		void                 Update(double time);
		Vector2              GetLocation() const;
		std::vector<Vector2> GetTrail() const;
		double               GetWeight() const;
		bool                 IsActive() const;

	private:
		Vector2              location;
		std::vector<Vector2> trail;
		double lastUpdated;
		double decayRate;
		double weight;
		double threshold;
};

#endif
