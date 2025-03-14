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
		std::vector<double> getVectorTime();

		std::vector<double> stl_getVectorPositionX_ship();
		std::vector<double> stl_getVectorPositionY_ship();
		std::vector<double> stl_getVectorPositionX_helicopter();
		std::vector<double> stl_getVectorPositionY_helicopter();

		std::vector<double> stl_getVectorDeltaPosX_trueState();
		std::vector<double> stl_getVectorDeltaPosY_trueState();
		std::vector<double> stl_getVectorDeltaVelX_trueState();
		std::vector<double> stl_getVectorDeltaVelY_trueState();

		std::vector<double> stl_getVectorDeltaPosX_awesomeState();
		std::vector<double> stl_getVectorDeltaPosY_awesomeState();
		std::vector<double> stl_getVectorDeltaVelX_awesomeState();
		std::vector<double> stl_getVectorDeltaVelY_awesomeState();

		std::vector<double> stl_getVectorUpperLimit_DeltaPosX_awesomeState();
		std::vector<double> stl_getVectorUpperLimit_DeltaPosY_awesomeState();
		std::vector<double> stl_getVectorUpperLimit_DeltaVelX_awesomeState();
		std::vector<double> stl_getVectorUpperLimit_DeltaVelY_awesomeState();

		std::vector<double> stl_getVectorLowerLimit_DeltaPosX_awesomeState();
		std::vector<double> stl_getVectorLowerLimit_DeltaPosY_awesomeState();
		std::vector<double> stl_getVectorLowerLimit_DeltaVelX_awesomeState();
		std::vector<double> stl_getVectorLowerLimit_DeltaVelY_awesomeState();

	private:
		Trans_ptr helicopter;
		Trans_ptr ship;
		Locator_ptr locator;
		double currentTime;
		double durationOfGoalTracking;
		std::vector<double> vectorTimes;

		Eigen::VectorXd vectorState_ship;
		Eigen::VectorXd vectorState_helicopter;
		Eigen::VectorXd vectorDelta_trueState;
		Eigen::VectorXd vectorDelta_awesomeState;

		Eigen::VectorXd vectorUpperLimit_awesomeState;
		Eigen::VectorXd vectorLowerLimit_awesomeState;

		void appendToVector(Vector& vec, const Vector& newVec);
		void appendBoundaryOfConfidenceIntervalToVector(const Matrix& matrixBorder);
		bool checkIntersection(const Vector& vec1, const Vector& vec2);
		std::vector<double> transformInSTLvector(const Vector& vec);
	};
}