#include "Locator.h"

apa::SensorNoise::SensorNoise()
{
	vectorMean.resize(2);
	matrixCov.resize(2, 2);

	vectorMean << 0.0, 0.0;
	matrixCov << 10.0, 0.0,
				 0.0, 10.0;

	noiseSensor = std::make_shared<RandomnessGenerator>(vectorMean, matrixCov);
}

Vector apa::SensorNoise::getSensorNoise()
{
	return noiseSensor->getVectorRejection();
}


apa::Locator::Locator(Trans_ptr& hunter, Trans_ptr& target) : me(hunter), target(target)
{
	noiseDelta_position.resize(2);
	noiseDelta_velocityFluct.resize(2);

	KF = std::make_shared<KalmanFilter>(getVectorDelta_state());
	noise = std::make_shared<SensorNoise>();
}

void apa::Locator::location()
{
	makeNoiseWithReceivedData();
	KF->perfomFiltring(getVectorNoiseDelta_state());
}

void apa::Locator::makeNoiseWithReceivedData()
{
	Vector vecotrDelta_state = getVectorDelta_state();

	noiseDelta_position = vecotrDelta_state.segment(0, 2) + noise->getSensorNoise(); // возращать шумы
	noiseDelta_velocityFluct = vecotrDelta_state.segment(2, 2);
}

Vector apa::Locator::getVectorNoiseDelta_state()
{
	Vector result(4);
	result = noiseDelta_position.segment(0, 2);
	result = noiseDelta_velocityFluct.segment(2, 2);

	return result;
}



Vector apa::Locator::getVectorDelta_awesomeState()
{
	return KF->getVectorDelta_awesomeState();
}

Vector apa::Locator::getMeVectorState()
{
	return me->getVectorState();
}

Vector apa::Locator::getTargetVectorState()
{
	return target->getVectorState();
}

Vector apa::Locator::getVectorDelta_state()
{
	Vector vec(4);
	vec << getTargetVectorState() - getMeVectorState();
	return vec;
}
	