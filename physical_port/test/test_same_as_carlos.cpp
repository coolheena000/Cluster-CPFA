// Offline test: runs Carlos's own CPFA functions (copied by extract_carlos.sh)
// and the port side by side with the same random numbers, positions and food,
// and compares them after every tick. No robot or ARGoS needed.

#include "../CPFA_controller.h"

#include <cstdio>
#include <cstdlib>
#include <random>

struct Random : RandomInterface {
	std::mt19937_64 engine;
	size_t draws = 0;
	double Unit() { draws++; return (engine() >> 11) * (1.0 / 9007199254740992.0); }
	double Uniform(double min, double max) override { return min + Unit() * (max - min); }
	double Gaussian(double s) override {
		double a = 1.0 - Unit();
		double b = Unit();
		return s * std::sqrt(-2.0 * std::log(a)) * std::cos(2.0 * M_PI * b);
	}
};

// Small stand-ins for the ARGoS types Carlos's functions use.
namespace argos {
	typedef double Real;
	struct CRadians {
		double v;
		explicit CRadians(double x = 0) : v(x) {}
		double GetValue() const { return v; }
		CRadians SignedNormalize() const { double a = v; while(a > M_PI) a -= 2*M_PI; while(a < -M_PI) a += 2*M_PI; return CRadians(a); }
		CRadians UnsignedNormalize() const { double a = v; while(a >= 2*M_PI) a -= 2*M_PI; while(a < 0) a += 2*M_PI; return CRadians(a); }
		static const CRadians PI, TWO_PI;
	};
	const CRadians CRadians::PI(M_PI), CRadians::TWO_PI(2*M_PI);
	inline CRadians operator+(CRadians a, CRadians b) { return CRadians(a.v + b.v); }
	inline CRadians operator-(CRadians a, CRadians b) { return CRadians(a.v - b.v); }
	struct CVector2 {
		double x, y;
		CVector2() : x(0), y(0) {}
		CVector2(double a, double b) : x(a), y(b) {}
		CVector2(double len, CRadians a) : x(len * std::cos(a.v)), y(len * std::sin(a.v)) {}
		double GetX() const { return x; }
		double GetY() const { return y; }
		double SquareLength() const { return x*x + y*y; }
		double Length() const { return std::sqrt(SquareLength()); }
		bool operator!=(const CVector2& o) const { return x != o.x || y != o.y; }
	};
	inline CVector2 operator+(CVector2 a, CVector2 b) { return CVector2(a.x+b.x, a.y+b.y); }
	inline CVector2 operator-(CVector2 a, CVector2 b) { return CVector2(a.x-b.x, a.y-b.y); }
	template<class T> struct CRange {
		T lo, hi;
		CRange(T a, T b) : lo(a), hi(b) {}
		T GetMin() const { return lo; }
		T GetMax() const { return hi; }
	};
	struct CColor { static const CColor ORANGE; };
	const CColor CColor::ORANGE = CColor();
}

namespace carlos {
	using namespace std;
	using argos::CRadians;

	struct RngShim {
		Random r;
		double Uniform(argos::CRange<double> c) { return r.Uniform(c.lo, c.hi); }
		double Gaussian(double s) { return r.Gaussian(s); }
	};

	struct Pheromone {  // same decay as source/Base/Pheromone.cpp
		argos::CVector2 location; vector<argos::CVector2> trail; double last, rate, weight = 1.0;
		Pheromone(argos::CVector2 l, vector<argos::CVector2> t, double tm, double d) : location(l), trail(t), last(tm), rate(d) {}
		void Update(double t) { weight *= exp(-rate * (t - last)); last = t; }
		void Deactivate() { weight = 0.0; }
		argos::CVector2 GetLocation() { return location; }
		vector<argos::CVector2> GetTrail() { return trail; }
		double GetWeight() { return weight; }
		bool IsActive() { return weight > 0.001; }
	};

	struct Loop {
		double ProbabilityOfSwitchingToSearching = 0.504, ProbabilityOfReturningToNest = 0.001;
		argos::CRadians UninformedSearchVariation{7.0 * M_PI / 180.0};
		double RateOfInformedSearchDecay = 0.28, RateOfSiteFidelity = 4.27, RateOfLayingPheromone = 3.75, RateOfPheromoneDecay = 0.03;
		argos::CVector2 NestPosition;
		double NestRadiusSquared = 0.25 * 0.25, SearchRadiusSquared = 0.2 * 0.2;
		size_t DrawDensityRate = 4, ResourceDensityDelay = 0, currNumCollectedFood = 0;
		vector<argos::CVector2> FoodList;
		vector<argos::CColor> FoodColoringList;
		vector<Pheromone> PheromoneList;
		map<string, argos::CVector2> FidelityList;
		void setScore(double) {}
	};

	struct BaseController {
		RngShim* RNG = nullptr;
		argos::CVector2 TargetPosition, pos;
		double heading = 0;
		bool heading_to_nest = true;
		double DestinationNoiseStdev = 0, NestDistanceTolerance = 0.05, TargetDistanceTolerance = 0.05;
		argos::CRadians NestAngleTolerance{0.1}, TargetAngleTolerance{0.1};
		argos::CRange<double> ForageRangeX{-5, 5}, ForageRangeY{-5, 5};
		size_t tick = 0, stops = 0, rightTurns = 0;
		argos::CVector2 GetPosition() { return pos; }
		argos::CRadians GetHeading() { return argos::CRadians(heading); }
		size_t SimulationTick() { return tick; }
		size_t SimulationTicksPerSecond() { return 16; }
		void Stop() { stops++; }
		void SetRightTurn(double) { rightTurns++; }
		argos::CVector2 GetTarget();
		void SetTarget(argos::CVector2 t);
		void SetIsHeadingToNest(bool n);
		bool IsAtTarget();
	};

	unsigned int num_targets_collected = 0;

	struct CPFA_controller : BaseController {
		Loop* LoopFunctions = nullptr;
		string controllerID;
		double FoodDistanceTolerance = 0.13 * 0.13, SearchStepSize = 0.08;
		vector<argos::CVector2> TrailToShare, TrailToFollow;
		argos::CVector2 SiteFidelityPosition{1000, 1000};
		bool updateFidelity = false, isInformed = false, isHoldingFood = false, isUsingSiteFidelity = false, isGivingUpSearch = false;
		size_t ResourceDensity = 0, SearchTime = 0;
		unsigned int survey_count = 0;
		enum State { DEPARTING = 0, SEARCHING = 1, RETURNING = 2, SURVEYING = 3 };
		State CPFA_state = DEPARTING;
		bool IsHoldingFood(); bool IsInTheNest();
		void CPFA(); void Departing(); void Searching(); void Surveying(); void Returning();
		void SetRandomSearchLocation(); void SetHoldingFood(); void SetLocalResourceDensity(); void SetFidelityList(); bool SetTargetPheromone();
		argos::Real GetExponentialDecay(argos::Real, argos::Real, argos::Real);
		argos::Real GetBound(argos::Real, argos::Real, argos::Real);
		argos::Real GetPoissonCDF(argos::Real, argos::Real);
	};

	#include "carlos_extract.inc"
}

// The port's food sensor, simulated with the same 0.13 m rule as Carlos.
struct TestRobot : RobotInterface {
	Vector2 pos;
	double heading = 0;
	size_t stops = 0, rightTurns = 0;
	std::vector<argos::CVector2>* food = nullptr;
	Vector2 GetPosition() override { return pos; }
	double GetHeading() override { return heading; }
	bool FoodDetected() override {
		for(size_t i = 0; i < food->size(); i++) {
			Vector2 d = pos - Vector2{(*food)[i].x, (*food)[i].y};
			if(d.SquareLength() < 0.13 * 0.13) { food->erase(food->begin() + i); return true; }
		}
		return false;
	}
	size_t LocalResourceDensity() override {
		size_t n = 1;
		for(auto& f : *food) if((pos - Vector2{f.x, f.y}).SquareLength() < 0.2 * 0.2 * 2) n++;
		return n;
	}
	void Stop() override { stops++; }
	void SetRightTurn(double) override { rightTurns++; }
};

static bool Same(double a, double b) { return std::fabs(a - b) < 1e-9; }

int main() {
	const int kSeeds = 50, kRobots = 4, kTicks = 16 * 1800;
	size_t ticks = 0, delivered = 0, fidelity = 0, informed = 0, pheromones = 0, boundary = 0, gaveUp = 0;
	size_t visits[4] = {0, 0, 0, 0};

	for(int seed = 1; seed <= kSeeds; seed++) {
		std::mt19937_64 world(seed);
		auto u = [&](double a, double b) { return a + (world() >> 11) * (1.0 / 9007199254740992.0) * (b - a); };
		carlos::Loop loop;
		CPFA_nest nest;
		// Half the seeds use a high give-up probability so that branch is tested too.
		if(seed % 2 == 0) loop.ProbabilityOfReturningToNest = nest.ProbabilityOfReturningToNest = 0.02;
		for(int c = 0; c < 8; c++) {
			double x = u(-1.5, 1.5), y = u(-1.5, 1.5);
			for(int i = 0; i < 16; i++) loop.FoodList.push_back(argos::CVector2(x + (i % 4) * 0.1, y + (i / 4) * 0.1));
		}
		loop.FoodColoringList.resize(loop.FoodList.size());
		std::vector<argos::CVector2> portFood = loop.FoodList;

		std::vector<carlos::CPFA_controller> ref(kRobots);
		std::vector<carlos::RngShim> refRng(kRobots);
		std::vector<Random> portRng(kRobots);
		std::vector<TestRobot> body(kRobots);
		std::vector<CPFA_controller> port;
		port.reserve(kRobots);
		for(int r = 0; r < kRobots; r++) {
			refRng[r].r.engine.seed(seed * 100 + r);
			portRng[r].engine.seed(seed * 100 + r);
			std::string id = "r" + std::to_string(r);
			ref[r].RNG = &refRng[r]; ref[r].LoopFunctions = &loop; ref[r].controllerID = id;
			ref[r].ForageRangeX = ref[r].ForageRangeY = argos::CRange<double>(-2, 2);
			body[r].food = &portFood;
			body[r].pos = {u(-0.2, 0.2), u(-0.2, 0.2)};
			port.emplace_back(id, nest, body[r], portRng[r], Vector2{-2, -2}, Vector2{2, 2}, 16);
		}

		for(int t = 0; t < kTicks; t++) {
			for(int r = 0; r < kRobots; r++) {
				carlos::CPFA_controller& a = ref[r];
				CPFA_controller& b = port[r];
				a.pos = argos::CVector2(body[r].pos.x, body[r].pos.y);
				a.heading = body[r].heading;
				a.tick = t;
				int before = a.CPFA_state;
				a.CPFA();
				b.CPFA(t);
				ticks++;

				bool ok = int(a.CPFA_state) == int(b.GetState())
					&& Same(a.TargetPosition.x, b.GetTarget().x) && Same(a.TargetPosition.y, b.GetTarget().y)
					&& a.isHoldingFood == b.IsHoldingFood() && a.isInformed == b.IsInformed()
					&& a.isUsingSiteFidelity == b.IsUsingSiteFidelity()
					&& refRng[r].r.draws == portRng[r].draws && a.stops == body[r].stops && a.rightTurns == body[r].rightTurns
					&& loop.PheromoneList.size() == nest.PheromoneList.size() && loop.FoodList.size() == portFood.size()
					&& loop.FidelityList.size() == nest.FidelityList.size() && loop.currNumCollectedFood == nest.currNumCollectedFood;
				if(!ok) {
					std::printf("FAIL seed %d tick %d robot %d: state %d/%d target (%f,%f)/(%f,%f) draws %zu/%zu\n", seed, t, r,
						int(a.CPFA_state), int(b.GetState()), a.TargetPosition.x, a.TargetPosition.y,
						b.GetTarget().x, b.GetTarget().y, refRng[r].r.draws, portRng[r].draws);
					return 1;
				}

				visits[a.CPFA_state]++;
				if(before == 1 && a.CPFA_state == 2) gaveUp++;
				if(before == 2 && a.CPFA_state == 0) {
					if(a.isUsingSiteFidelity) fidelity++;
					else if(a.isInformed) informed++;
				}

				// Move the shared body: turn toward the target, then drive (16 cm/s).
				Vector2 d = b.GetTarget() - body[r].pos;
				double e = std::atan2(std::sin(std::atan2(d.y, d.x) - body[r].heading), std::cos(std::atan2(d.y, d.x) - body[r].heading));
				if(std::fabs(e) > 0.05) body[r].heading = std::atan2(std::sin(body[r].heading + std::max(-0.15, std::min(0.15, e))), std::cos(body[r].heading + std::max(-0.15, std::min(0.15, e))));
				else if(d.Length() > 0.01) body[r].pos = body[r].pos + Vector2{0.01 * std::cos(body[r].heading), 0.01 * std::sin(body[r].heading)};
			}
			pheromones = std::max(pheromones, nest.PheromoneList.size());
			if(t % 8 == 0) {
				std::vector<carlos::Pheromone> keep;
				for(auto& p : loop.PheromoneList) { p.Update(t / 16); if(p.IsActive()) keep.push_back(p); }
				loop.PheromoneList = keep;
				nest.UpdatePheromoneList(t / 16);
			}
		}
		delivered += nest.currNumCollectedFood;
		for(auto& b : body) boundary += b.rightTurns;
	}
	std::printf("PASS: %zu robot ticks compared, all equal\n", ticks);
	std::printf("covered: departing %zu searching %zu returning %zu surveying %zu ticks\n", visits[0], visits[1], visits[2], visits[3]);
	std::printf("delivered %zu, left nest by site fidelity %zu, informed without fidelity flag %zu, max pheromones %zu, boundary turns %zu, gave up %zu\n",
		delivered, fidelity, informed, pheromones, boundary, gaveUp);
	return 0;
}
