#pragma once

#include <memory>
#include <utility>
#include <Eigen/Dense>
#include "KalmanFilter.h"
#include "OnBoardLocatorShip.h"
#include "OnBoardLocatorHelicopter.h"

namespace apa
{
	class OnBoardLocatorShip;
	class OnBoardLocatorHelicopter;
	class SensorNoise;

	using Vec2D = Eigen::Vector2d;
	using Vec4D = Eigen::Vector4d;
	using Mat2D = Eigen::Matrix2d;
	using Mat4D = Eigen::Matrix4d;

	using Kalman_ptr = std::unique_ptr<KalmanFilter>;
	using Sensor_ptr = std::unique_ptr<SensorNoise>;
	using OBSL_ship_ptr = std::shared_ptr<OnBoardLocatorShip>;
	using OBSL_helic_ptr = std::shared_ptr<OnBoardLocatorHelicopter>;


	class SensorNoise
	{
	public:
		SensorNoise();
		Vec2D getSensorNoise(const double time);

	protected:
		std::unique_ptr<RandomnessGenerator> noiseSensor;
		Vec2D vectorMean;
		Mat2D matrixCov;
	};

	class Locator : public std::enable_shared_from_this<Locator>
	{
	public:
		Locator(Vec4D beginMean, Mat4D beginCov, const double dt);

		void explorStateCurrentTime_Hunter(const double time, const double dt);
		void explorStateCurrentTime_Target(const double time, const double dt);

		void createTargetWithPosition(Vec2D position);
		void createHunterWithPosition(Vec2D position);
		void setFluctationForTarget(Vec2D mean, Mat2D cov);
		void setFluctationForHunter(Vec2D mean, Mat2D cov);

		Vec4D get_DeltaEstimatedVector_Targ_Helic(const double time, const double dt);
		Vec4D get_DetltaVector_Targ_Helic(const double time);

	private:
		Sensor_ptr noise;
		Kalman_ptr KF;
		OBSL_ship_ptr locShip;
		OBSL_helic_ptr locHelic;

		Vec4D getVectorStateTarget(const double time);
		Vec4D getVectorStateHunter(const double time);
	};
}