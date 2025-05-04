#pragma once
#include <Eigen/Dense>

namespace apa
{
	using Vec2D = Eigen::Vector2d;
	using Vec4D = Eigen::Vector4d;

	constexpr double maxModuleVelocity = 70.0; // m/sec

	class AimingSystem
	{
	public:
		AimingSystem();
		Vec2D getVectorVelocityAiming(const Vec4D& vectorState, const double dT);

	private:
		Vec2D vectorVelocityAiming;

		Vec2D normalize(const Vec2D& vec);
		double getModuleVectorVelocity(const Vec2D& vec);
	};
}