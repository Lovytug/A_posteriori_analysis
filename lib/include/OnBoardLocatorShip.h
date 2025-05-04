#pragma once

#include <memory>
#include "Ship.h"

namespace apa
{
	using Vec2D = Eigen::Vector2d;
	using Mat2D = Eigen::Matrix2d;

	using Ship_ptr = std::unique_ptr<Ship>;

	class OnBoardLocatorShip
	{
	public:
		OnBoardLocatorShip(Vec2D postition);

		void explorStateCurrentTime_Ship(const double time, const double dt);

		void setFluctation(Vec2D mean, Mat2D cov);

		Eigen::Vector4d getVectorState(const double time);

	private:
		Ship_ptr ship;
	};
}