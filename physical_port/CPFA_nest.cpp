#include "CPFA_nest.h"

void CPFA_nest::UpdatePheromoneList(double timeInSeconds) {
	std::vector<Pheromone> new_p_list;

	for(size_t i = 0; i < PheromoneList.size(); i++) {
		PheromoneList[i].Update(timeInSeconds);
		if(PheromoneList[i].IsActive()) new_p_list.push_back(PheromoneList[i]);
	}

	PheromoneList = new_p_list;
}
