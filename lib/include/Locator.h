#pragma once
#include "Transport.h"
#include "KalmanFilter.h"
#include <memory>

using Trans_ptr = std::shared_ptr<apa::Transport>;
using Sensor_ptr = std::shared_ptr<apa::SensorNoise>;

namespace apa
{
	class SensorNoise
	{
	public:
		SensorNoise();

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
		Sensor_ptr noise;
		Trans_ptr me;
		Trans_ptr target;
		Vector noiseDelta_position;
		Vector noiseDelta_velocityFluct;

		//also for getVectorDelta_state() 
		Vector getMeVectorState();
		Vector getTargetVectorState();

		void getDataInNoiseForm();
		Vector transformVectorStateInVecDelta_position();
		Vector transformVectorStateInVecDelta_velocityFluct();

	};
}