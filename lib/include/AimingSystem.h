#pragma once
#include <Eigen/Dense>

using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;

namespace apa
{
	class AimingSystem
	{
	public:
		AimingSystem(const Vector& vec, const double& dT);
		Vector getVectorVelocityAiming();
		Vector getVectorVelocityAiming(const Vector& vectorState, const double& dT);

	private:
		Vector vectorVelocityAiming;
		double maxModuleVelocity = 250.0;

		Vector normalize(const Vector& vec);
		double getModuleVectorVelocity(const Vector& vec);
	};
}