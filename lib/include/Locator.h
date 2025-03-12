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
		Vector getSensorNoise();

	protected:
		std::shared_ptr<RandomnessGenerator> noiseSensor;
		Vector vectorMean;
		Matrix matrixCov;
	};

	class Locator
	{
	public:
		Locator(Trans_ptr& hunter, Trans_ptr& target, Kalman_ptr& KF);
		void location(const Vector& vec);
		Vector getVectorDelta_state();
		Vector getVectorDelta_awesomeState(); //KF

	private:
		Trans_ptr& me;
		Trans_ptr& target;
		Kalman_ptr& KF;
		std::shared_ptr<SensorNoise> noise;
		Vector noiseDelta_position;
		Vector noiseDelta_velocityFluct;

		//also for getVectorDelta_state() 
		Vector getMeVectorState();
		Vector getTargetVectorState();

		void writeRealDataFromObject();
		Vector getVisibleVector();

	};
}