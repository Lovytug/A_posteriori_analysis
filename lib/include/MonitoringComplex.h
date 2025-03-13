#include <Eigen/Dense>
#include <memory>
#include <vector>
#include "Transport.h"
#include "OnBoardSystem.h"
#include "Helicopter.h"
#include "Ship.h"

using Trans_ptr = std::shared_ptr<apa::Transport>;
using OnBoard_ptr = std::shared_ptr<apa::OnBoardSystem>;
using Locator_ptr = std::shared_ptr<apa::Locator>;

namespace apa
{
	class MonitoringComplex
	{
	public:
		MonitoringComplex(const Trans_ptr& obj1, const Trans_ptr& obj2, OnBoard_ptr& OBS, const double& allTime);
		void trackMovementOfGoals();

	private:
		Trans_ptr helicopter;
		Trans_ptr ship;
		Locator_ptr locator;
		double currentTime;
		double durationOfGoalTracking;

		Eigen::VectorXd vectorState_ship;
		Eigen::VectorXd vectorState_helicopter;
		Eigen::VectorXd vectorDelta_trueState;
		Eigen::VectorXd vectorDelta_awesomeState;

		void appendToVector(Vector& vec, const Vector& newVec);
	};
}