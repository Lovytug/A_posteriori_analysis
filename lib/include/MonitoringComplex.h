#pragma once
#include <memory>
#include <Eigen/Dense>
#include "Locator.h"

namespace apa
{
	using Vec2D = Eigen::Vector2d;
	using Vec4D = Eigen::Vector4d;
	using Mat2D = Eigen::Matrix2d;
	using Mat4D = Eigen::Matrix4d;

	using Locator_ptr = std::shared_ptr<Locator>;

	class MonitoringComplex
	{
	public:
		MonitoringComplex(Vec4D beginMean, Mat4D beginCov, const double allTimeModeling, const double dt);

		void startSimulation();

		void createTarget_Ship(Vec2D position);
		void createHunter_Helicopter(Vec2D position);
		void setFluctationTarget_Ship(Vec2D mean, Mat2D cov);
		void setFluctationHunter_Helicopter(Vec2D mean, Mat2D cov);


	private:
		Locator_ptr locator;
		double totalSimulationTime;
		double stepSimulation;
	};
}