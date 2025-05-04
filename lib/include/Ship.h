#pragma once

#include <memory>
#include <Eigen/Dense>
#include "RandomnessGenerator.h"

namespace apa
{
	using Vec2D = Eigen::Vector2d;
	using Mat2D = Eigen::Matrix2d;

	class Wave
	{
	public:
		Wave(Vec2D meanVelocity, Mat2D covMatrix);
		Vec2D getTrueVelocity(const double time);

	private:
		std::unique_ptr<RandomnessGenerator> fluctation;
		Vec2D meanVelocity;
		Mat2D covMatrix;
	};


	class Ship
	{
	public:
		Ship(Vec2D vecPos);

		void setFluctation(Vec2D mean, Mat2D cov);

		void move(const double time, const double dT);

		Eigen::Vector4d getVectorState(const double time);

	private:
		std::unique_ptr<Wave> wave;
		Vec2D radiusVector;
	};
}