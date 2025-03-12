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


apa::Locator::Locator(Trans_ptr& hunter, Trans_ptr& target, Kalman_ptr& KF) : me(hunter), target(target), KF(KF)
{
	noiseDelta_position.resize(2);
	noiseDelta_velocityFluct.resize(2);

	noise = std::make_shared<SensorNoise>();
}

void apa::Locator::location(const Vector& vecAimVel)
{
	writeRealDataFromObject();
	KF->perfomFiltring(getVisibleVector(), vecAimVel);
}

void apa::Locator::writeRealDataFromObject()
{
	Vector vectorDelta_state = getVectorDelta_state();

	noiseDelta_position = vectorDelta_state.segment(0, 2) + noise->getSensorNoise(); // возращать шумы
	noiseDelta_velocityFluct = vectorDelta_state.segment(2, 2);
}

Vector apa::Locator::getVisibleVector()
{
	Vector result(2);
	result = noiseDelta_position;

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
	