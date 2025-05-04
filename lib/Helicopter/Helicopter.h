#pragma once

#include <memory>
#include "RandomnessGenerator.h"
#include "AimingSystem.h"
#include "OnBoardLocatorHelicopter.h"

namespace apa
{
	class OnBoardLocatorHelicopter;

	using Vec2D = Eigen::Vector2d;
	using Mat2D = Eigen::Matrix2d;

	class Wind
	{
	public:
		Wind(Vec2D meanVelocity, Mat2D covMatrix);
		Vec2D getTrueVelocity(const double time);

	private:
		std::shared_ptr<RandomnessGenerator> fluctation;
		Vec2D meanVelocity;
		Mat2D covMatrix;
	};

	class Helicopter
	{
	public:
		Helicopter(Vec2D vecPos);

		void enableConnection(std::shared_ptr<OnBoardLocatorHelicopter>& loc);

		void setFluctation(Vec2D mean, Mat2D cov);

		void move(const double time, const double dT);

		Eigen::Vector4d getVectorState(const double time) ;

	private:
		std::unique_ptr<Wind> wind;
		std::shared_ptr<OnBoardLocatorHelicopter> locator;
		std::unique_ptr<AimingSystem> AS;
		Vec2D radiusVector;
	};
}