#include "Pheromone.h"

Pheromone::Pheromone(Vector2 newLocation, std::vector<Vector2> newTrail, double newTime, double newDecayRate) :
	location(newLocation),
	trail(newTrail),
	lastUpdated(newTime),
	decayRate(newDecayRate),
	weight(1.0),
	threshold(0.001)
{
}

void Pheromone::Update(double time) {
	weight *= std::exp(-decayRate * (time - lastUpdated));
	lastUpdated = time;
}

Vector2 Pheromone::GetLocation() const { return location; }

std::vector<Vector2> Pheromone::GetTrail() const { return trail; }

double Pheromone::GetWeight() const { return weight; }

bool Pheromone::IsActive() const { return weight > threshold; }
