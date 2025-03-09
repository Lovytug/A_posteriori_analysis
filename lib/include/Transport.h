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
		virtual Vector getTrueVelocity() = 0;

	};

	class Transport
	{
	public:
		Transport() = default;
		virtual void move() = 0;
		virtual Vector getVectorState() = 0;
	};
}