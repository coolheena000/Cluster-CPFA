#ifndef CPFA_CONTROLLER_H
#define CPFA_CONTROLLER_H

#include "CPFA_nest.h"
#include "RobotInterface.h"

// Port of Carlos's source/CPFA/CPFA_controller.h. Function names are the same.
class CPFA_controller {
	public:
		enum State { DEPARTING = 0, SEARCHING = 1, RETURNING = 2, SURVEYING = 3 };

		CPFA_controller(std::string id, CPFA_nest& nest, RobotInterface& robot, RandomInterface& rng,
		                Vector2 forageMin, Vector2 forageMax, size_t ticksPerSecond);

		// One control tick. Afterwards GetTarget() is where the robot should drive.
		void CPFA(size_t tick);

		State   GetState() const            { return CPFA_state; }
		Vector2 GetTarget() const           { return TargetPosition; }
		bool    IsHoldingFood() const       { return isHoldingFood; }
		bool    IsInformed() const          { return isInformed; }
		bool    IsUsingSiteFidelity() const { return isUsingSiteFidelity; }
		bool    IsInTheNest();

	private:
		void Departing();
		void Searching();
		void Surveying();
		void Returning();

		void SetRandomSearchLocation();
		void SetHoldingFood();
		void SetLocalResourceDensity();
		void SetFidelityList();
		bool SetTargetPheromone();

		// Carlos: BaseController
		void SetTarget(Vector2 t);
		bool IsAtTarget();
		void SetIsHeadingToNest(bool n) { heading_to_nest = n; }

		double GetExponentialDecay(double value, double time, double lambda);
		double GetBound(double value, double min, double max);
		double GetPoissonCDF(double k, double lambda);

		static constexpr double SearchStepSize          = 0.08;
		static constexpr double TargetDistanceTolerance = 0.05;
		static constexpr double NestDistanceTolerance   = 0.05;
		static constexpr double TargetAngleTolerance    = 0.1;
		static constexpr double DestinationNoiseStdev   = 0.0;

		std::string      controllerID;
		CPFA_nest&       Nest;
		RobotInterface&  Robot;
		RandomInterface& RNG;
		Vector2          ForageMin, ForageMax;
		size_t           TicksPerSecond;
		size_t           Tick = 0;

		State   CPFA_state = DEPARTING;
		Vector2 TargetPosition;
		Vector2 SiteFidelityPosition{1000, 1000};
		std::vector<Vector2> TrailToShare;
		std::vector<Vector2> TrailToFollow;

		bool heading_to_nest     = true;
		bool updateFidelity      = false;
		bool isInformed          = false;
		bool isHoldingFood       = false;
		bool isUsingSiteFidelity = false;
		bool isGivingUpSearch    = false;

		size_t       ResourceDensity = 0;
		size_t       SearchTime      = 0;
		unsigned int survey_count    = 0;
};

#endif
