#pragma once
#include "Transport.h"
#include "KalmanFilter.h"
#include <memory>

using Trans_ptr = std::shared_ptr<apa::Transport>;
using Kalman_ptr = std::shared_ptr<apa::KalmanFilter>;

namespace apa
{
	class SensorNoise
	{
	public:
		SensorNoise();
		Vector getSensorNoise(const double& time);

	protected:
		std::shared_ptr<RandomnessGenerator> noiseSensor;
		Vector vectorMean;
		Matrix matrixCov;
	};

	class Locator
	{
	public:
		Locator(Trans_ptr& target, Trans_ptr& hunter, Kalman_ptr& KF);
		void location(const Vector& vec, const double& time);
		Vector getVectorDelta_state(const double& time);
		Vector getVectorDelta_awesomeState(); //KF
		Matrix getBorderOfConfidenceInterval(); //KF

	private:
		Trans_ptr& me;
		Trans_ptr& target;
		Kalman_ptr KF;
		std::shared_ptr<SensorNoise> noise;
		Vector noiseDelta_position;
		Vector noiseDelta_velocityFluct;

		//also for getVectorDelta_state() 
		Vector getMeVectorState(const double& time);
		Vector getTargetVectorState(const double& time);

		void writeRealDataFromObject(const double& time);
		Vector getVisibleVector();

	};
}