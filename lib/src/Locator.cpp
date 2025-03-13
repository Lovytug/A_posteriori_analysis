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

Vector apa::SensorNoise::getSensorNoise(const double& time)
{
	return noiseSensor->getVectorRejection(time);
}


apa::Locator::Locator(Trans_ptr& target, Trans_ptr& hunter, Kalman_ptr& KF) : me(hunter), target(target), KF(KF)
{
	noiseDelta_position.resize(2);
	noiseDelta_velocityFluct.resize(2);

	noise = std::make_shared<SensorNoise>();
}

void apa::Locator::location(const Vector& vecAimVel, const double& time)
{
	writeRealDataFromObject(time);
	KF->perfomFiltring(getVisibleVector(), vecAimVel);
}

void apa::Locator::writeRealDataFromObject(const double& time)
{
	Vector vectorDelta_state = getVectorDelta_state(time);

	noiseDelta_position = vectorDelta_state.segment(0, 2) + noise->getSensorNoise(time); // ��������� ����
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

Vector apa::Locator::getMeVectorState(const double& time)
{
	return me->getVectorState(time);
}

Vector apa::Locator::getTargetVectorState(const double& time)
{
	return target->getVectorState(time);
}

Vector apa::Locator::getVectorDelta_state(const double& time)
{
	Vector vec(4);
	Vector vec2 = getTargetVectorState(time);
	double r = vec2[0];
	vec << getTargetVectorState(time) - getMeVectorState(time);
	return vec;
}
	