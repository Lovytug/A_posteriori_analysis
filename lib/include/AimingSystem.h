#pragma once
#include <Eigen/Dense>

using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;

namespace apa
{
	class AimingSystem
	{
	public:
		AimingSystem();
		Vector getVectorVelocityAiming(const Vector& vectorState);

	private:
		Vector vectorVelocityAiming;
		double maxModuleVelocity = 250.0;

		Vector normalize(const Vector& vec);
		double getModuleVectorVelocity(const Vector& vec);
	};
}