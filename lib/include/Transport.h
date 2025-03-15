#pragma once
#include "RandomnessGenerator.h"
#include <Eigen/Dense>

using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;

namespace apa
{
	class NaturalDisturbances
	{
	public:
		NaturalDisturbances() = default;
		virtual Vector getTrueVelocity(const double& time) = 0;

	};

	class Transport
	{
	public:
		Transport() = default;
		virtual void move(const double& currentTime, const double& dT) = 0;
		virtual Vector getVectorState(const double& time) = 0;
	};
}