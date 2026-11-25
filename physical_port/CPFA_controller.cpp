#include "CPFA_controller.h"

// Line numbers are Carlos's source/CPFA/CPFA_controller.cpp unless noted.

static double SignedNormalize(double a) {
	while(a > M_PI) a -= 2.0 * M_PI;
	while(a < -M_PI) a += 2.0 * M_PI;
	return a;
}

static Vector2 Polar(double length, double angle) {
	return {length * std::cos(angle), length * std::sin(angle)};
}

CPFA_controller::CPFA_controller(std::string id, CPFA_nest& nest, RobotInterface& robot, RandomInterface& rng,
                                 Vector2 forageMin, Vector2 forageMax, size_t ticksPerSecond) :
	controllerID(id), Nest(nest), Robot(robot), RNG(rng),
	ForageMin(forageMin), ForageMax(forageMax), TicksPerSecond(ticksPerSecond)
{
}

// line 131
void CPFA_controller::CPFA(size_t tick) {
	Tick = tick;

	switch(CPFA_state) {
		case DEPARTING:
			SetIsHeadingToNest(false);
			Departing();
			break;
		case SEARCHING:
			SetIsHeadingToNest(false);
			if(Tick % (TicksPerSecond / 2) == 0) Searching();
			break;
		case RETURNING:
			SetIsHeadingToNest(true);
			Returning();
			break;
		case SURVEYING:
			SetIsHeadingToNest(false);
			Surveying();
			break;
	}
}

// line 162
bool CPFA_controller::IsInTheNest() {
	return (Robot.GetPosition() - Nest.NestPosition).SquareLength() < Nest.NestRadiusSquared;
}

// line 237
void CPFA_controller::Departing() {
	double distanceToTarget = (Robot.GetPosition() - GetTarget()).Length();
	double randomNumber     = RNG.Uniform(0.0, 1.0);

	// When not informed, travel until randomly switching to searching.
	if(Tick % (TicksPerSecond / 2) == 0) {
		if(!isInformed) {
			bool targetIsOrigin = GetTarget().x == 0 && GetTarget().y == 0;
			if(randomNumber < Nest.ProbabilityOfSwitchingToSearching && !targetIsOrigin) {
				Robot.Stop();
				SearchTime = 0;
				CPFA_state = SEARCHING;
				double rotation = RNG.Gaussian(Nest.UninformedSearchVariation);
				SetIsHeadingToNest(false);
				SetTarget(Polar(SearchStepSize, rotation + Robot.GetHeading()) + Robot.GetPosition());
			}
			else if(distanceToTarget < TargetDistanceTolerance) {
				SetRandomSearchLocation();
			}
		}
	}

	// Informed: using site fidelity or a pheromone.
	if(isInformed && distanceToTarget < TargetDistanceTolerance) {
		SearchTime = 0;
		CPFA_state = SEARCHING;

		if(isUsingSiteFidelity) {
			isUsingSiteFidelity = false;
			SetFidelityList();
		}
	}
}

// line 293
void CPFA_controller::Searching() {
	SetHoldingFood();

	if(!isHoldingFood) {
		Vector2 distance = Robot.GetPosition() - GetTarget();
		double  random   = RNG.Uniform(0.0, 1.0);

		// Carlos compares the squared distance (m^2) with the tolerance (m).
		if(distance.SquareLength() < TargetDistanceTolerance) {

			// randomly give up searching
			if(random < Nest.ProbabilityOfReturningToNest) {
				SetFidelityList();
				TrailToShare.clear();
				SetIsHeadingToNest(true);
				SetTarget(Nest.NestPosition);
				isGivingUpSearch    = true;
				isUsingSiteFidelity = false;
				updateFidelity      = false;
				CPFA_state = RETURNING;
				return;
			}

			// uninformed search
			if(!isInformed) {
				double rotation = RNG.Gaussian(Nest.UninformedSearchVariation);
				SetIsHeadingToNest(false);
				SetTarget(Polar(SearchStepSize, rotation + Robot.GetHeading()) + Robot.GetPosition());
			}
			// informed search
			else {
				SetIsHeadingToNest(false);
				if(IsAtTarget()) {
					size_t t           = SearchTime++;
					double correlation = GetExponentialDecay(4.0 * M_PI - Nest.UninformedSearchVariation, t, Nest.RateOfInformedSearchDecay);
					double rotation    = GetBound(RNG.Gaussian(correlation + Nest.UninformedSearchVariation), -M_PI, M_PI);
					SetTarget(Polar(SearchStepSize, Robot.GetHeading() + rotation) + Robot.GetPosition());
				}
			}
		}
	}
}

// line 418
void CPFA_controller::Surveying() {
	if(survey_count <= 4) {
		double rotation = survey_count * 3.14 / 2;

		SetIsHeadingToNest(true);
		SetTarget(Polar(SearchStepSize, rotation) + Robot.GetPosition());

		if(std::fabs(SignedNormalize(Robot.GetHeading() - rotation)) < TargetAngleTolerance) survey_count++;
	}
	else {
		SetIsHeadingToNest(true);
		SetTarget(Nest.NestPosition);
		CPFA_state   = RETURNING;
		survey_count = 0;
	}
}

// line 450
void CPFA_controller::Returning() {
	if(!IsInTheNest()) {
		SetIsHeadingToNest(true);
		SetTarget(Nest.NestPosition);
		return;
	}

	double poissonCDF_pLayRate    = GetPoissonCDF(ResourceDensity, Nest.RateOfLayingPheromone);
	double poissonCDF_sFollowRate = GetPoissonCDF(ResourceDensity, Nest.RateOfSiteFidelity);
	double r1 = RNG.Uniform(0.0, 1.0);
	double r2 = RNG.Uniform(0.0, 1.0);

	if(isHoldingFood) {
		Nest.currNumCollectedFood++;
		if(poissonCDF_pLayRate > r1 && updateFidelity) {
			TrailToShare.push_back(Nest.NestPosition);
			double timeInSeconds = Tick / TicksPerSecond;
			Nest.PheromoneList.push_back(Pheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, Nest.RateOfPheromoneDecay));
		}
		TrailToShare.clear();
	}

	// use site fidelity
	if(updateFidelity && poissonCDF_sFollowRate > r2) {
		SetIsHeadingToNest(false);
		SetTarget(SiteFidelityPosition);
		isInformed = true;
	}
	// use pheromone waypoints
	else if(SetTargetPheromone()) {
		isInformed          = true;
		isUsingSiteFidelity = false;
	}
	// use random search
	else {
		SetRandomSearchLocation();
		isInformed          = false;
		isUsingSiteFidelity = false;
	}

	isGivingUpSearch = false;
	CPFA_state       = DEPARTING;
	isHoldingFood    = false;
}

// line 519
void CPFA_controller::SetRandomSearchLocation() {
	double random_wall = RNG.Uniform(0.0, 1.0);
	double x = 0.0, y = 0.0;

	if(random_wall < 0.25) {        // north wall
		x = RNG.Uniform(ForageMin.x, ForageMax.x);
		y = ForageMax.y;
	}
	else if(random_wall < 0.5) {    // south wall
		x = RNG.Uniform(ForageMin.x, ForageMax.x);
		y = ForageMin.y;
	}
	else if(random_wall < 0.75) {   // east wall
		x = ForageMax.x;
		y = RNG.Uniform(ForageMin.y, ForageMax.y);
	}
	else {                          // west wall
		x = ForageMin.x;
		y = RNG.Uniform(ForageMin.y, ForageMax.y);
	}

	SetIsHeadingToNest(true);
	SetTarget({x, y});
}

// line 553. The robot's sensor takes the place of the simulator's food list.
// Carlos also records trail points here for drawing; that part is left out.
void CPFA_controller::SetHoldingFood() {
	if(!isHoldingFood && Robot.FoodDetected()) {
		isHoldingFood = true;
		CPFA_state    = SURVEYING;
		SetIsHeadingToNest(true);
		SetTarget(Nest.NestPosition);
		SetLocalResourceDensity();
	}
}

// line 620
void CPFA_controller::SetLocalResourceDensity() {
	ResourceDensity      = Robot.LocalResourceDensity();
	SiteFidelityPosition = Robot.GetPosition();
	isUsingSiteFidelity  = true;
	updateFidelity       = true;
	TrailToShare.push_back(SiteFidelityPosition);
	Nest.FidelityList[controllerID] = SiteFidelityPosition;
}

// line 681
void CPFA_controller::SetFidelityList() {
	Nest.FidelityList.erase(controllerID);
}

// line 701
bool CPFA_controller::SetTargetPheromone() {
	double maxStrength = 0.0;
	if(Nest.PheromoneList.empty()) return false;

	SetIsHeadingToNest(true);
	SetTarget(Nest.NestPosition);

	for(const Pheromone& p : Nest.PheromoneList) {
		if(p.IsActive()) maxStrength += p.GetWeight();
	}

	double randomWeight = RNG.Uniform(0.0, maxStrength);

	for(const Pheromone& p : Nest.PheromoneList) {
		if(randomWeight < p.GetWeight()) {
			SetIsHeadingToNest(false);
			SetTarget(p.GetLocation());
			TrailToFollow = p.GetTrail();
			return true;
		}
		randomWeight -= p.GetWeight();
	}
	return false;
}

// BaseController.cpp line 79
void CPFA_controller::SetTarget(Vector2 t) {
	if(!heading_to_nest) {
		double distanceToTarget = (TargetPosition - Robot.GetPosition()).Length();
		t.x += RNG.Gaussian(DestinationNoiseStdev * distanceToTarget);
		t.y += RNG.Gaussian(DestinationNoiseStdev * distanceToTarget);
	}

	if(t.y > ForageMax.y || t.y < ForageMin.y || t.x > ForageMax.x || t.x < ForageMin.x) {
		t = Robot.GetPosition();
		Robot.SetRightTurn(37.5);
	}

	TargetPosition = t;
}

// BaseController.cpp line 448 (the tolerance is a float there too)
bool CPFA_controller::IsAtTarget() {
	float tolerance = heading_to_nest ? NestDistanceTolerance : TargetDistanceTolerance;
	return (TargetPosition - Robot.GetPosition()).Length() < tolerance;
}

// line 751
double CPFA_controller::GetExponentialDecay(double value, double time, double lambda) {
	return value * std::exp(-lambda * time);
}

// line 764
double CPFA_controller::GetBound(double value, double min, double max) {
	double offset = std::abs(min) + std::abs(max);
	while(value < min) value += offset;
	while(value > max) value -= offset;
	return value;
}

// line 785
double CPFA_controller::GetPoissonCDF(double k, double lambda) {
	double sumAccumulator       = 1.0;
	double factorialAccumulator = 1.0;

	for(size_t i = 1; i <= std::floor(k); i++) {
		factorialAccumulator *= i;
		sumAccumulator += std::pow(lambda, i) / factorialAccumulator;
	}

	return std::exp(-lambda) * sumAccumulator;
}
