#pragma once
#include "Transport.h"
#include "KalmanFilter.h"
#include <memory>

using Trans_ptr = std::shared_ptr<apa::Transport>;

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
		Locator(const Trans_ptr& hunter, const Trans_ptr& target);
		void location();
		Vector getVectorDelta_state();
		Vector getVectorDelta_awesomeState(); //KF

	private:
		Trans_ptr me;
		Trans_ptr target;
		std::shared_ptr<KalmanFilter> KF;
		std::shared_ptr<SensorNoise> noise;
		Vector noiseDelta_position;
		Vector noiseDelta_velocityFluct;

		//also for getVectorDelta_state() 
		Vector getMeVectorState();
		Vector getTargetVectorState();

		void makeNoiseWithReceivedData();
		Vector getVectorNoiseDelta_state();
		Vector transformVectorStateInVecDelta_position();
		Vector transformVectorStateInVecDelta_velocityFluct();

	};
}