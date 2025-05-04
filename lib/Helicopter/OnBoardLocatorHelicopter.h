#pragma once

#include <memory>
#include <Eigen/Dense>
#include "Locator.h"
#include "Helicopter.h"

namespace apa
{
	class Locator;
	class Helicopter;

	using Vec2D = Eigen::Vector2d;
	using Mat2D = Eigen::Matrix2d;

	using Locator_ptr = std::shared_ptr<Locator>;
	using Helic_ptr = std::unique_ptr<Helicopter>;

	class OnBoardLocatorHelicopter : public std::enable_shared_from_this<OnBoardLocatorHelicopter>
	{
	public:
		OnBoardLocatorHelicopter(Vec2D positon);

		void establishConnection(Locator_ptr& loc);

		void explorStateCurrentTime_Helicopter(const double time, const double dt);

		void setFluctation(Vec2D mean, Mat2D cov);

		Eigen::Vector4d getMissDataFromTarget(const double time, const double dt);
		Eigen::Vector4d getVectorState(const double time);

	private:
		Helic_ptr helic;
		Locator_ptr locatorBase;
	};
}