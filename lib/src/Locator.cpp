#include "Locator.h"

apa::SensorNoise::SensorNoise()
{
	vectorMean(2);
	matrixCov(2, 2);

	vectorMean << 0.0, 0.0;
	matrixCov << 10.0, 0.0,
				 0.0, 10.0;

	noiseSensor = std::make_shared<RandomnessGenerator>(vectorMean, matrixCov);
}

Vector apa::SensorNoise::getSensorNoise()
{

}


apa::Locator::Locator(const Trans_ptr& hunter, const Trans_ptr& target) : me(hunter), target(target)
{
	noiseDelta_position(2);
	noiseDelta_velocityFluct(2);

	KF = std::make_shared<KalmanFilter>();
	noise = std::make_shared<SensorNoise>();
}

void apa::Locator::location()
{
	makeNoiseWithReceivedData();
	KF->porfomFiltring(getVectorNoiseDelta_state());
}

void apa::Locator::makeNoiseWithReceivedData()
{
	Vector vecotrDelta_state = getVectorDelta_state();

	noiseDelta_position = vecotrDelta_state.segment(0, 2) + noise->getSensorNoise(); // возращать шумы
	noiseDelta_velocityFluct = vecotrDelta_state.segment(2, 2);
}

Vector apa::Locator::getVectorDelta_awesomeState()
{
	return KF->getVectorDelta_awesomeState();
}

Vector apa::Locator::getVectorDelta_state()
{
	Vector vec(4);
	vec << getTargetVectorState() - getMeVectorState();
	return vec;
}
	