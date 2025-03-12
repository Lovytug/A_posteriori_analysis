#include <Eigen/Dense>
#include <memory>
#include <vector>
#include "Transport.h"
#include "Helicopter.h"
#include "Ship.h"

using Trans_ptr = std::shared_ptr<apa::Transport>;

namespace apa
{
	class MonitoringComplex
	{
	public:
		MonitoringComplex(const Trans_ptr& obj1, const Trans_ptr& obj2, const double& allTime);
		void trackMovementOfGoals();

	private:
		Trans_ptr helicopter;
		Trans_ptr ship;
		double currentTime;
		double durationOfGoalTracking;

		Eigen::VectorXd vectorDelta_trueState;
		Eigen::VectorXd vectorDelta_awesomeState;
	};
}