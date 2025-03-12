#include "Helicopter.h"

apa::Wind::Wind(const Vector& mean, const Matrix& cov)
{
	meanVelocity.resize(2);
	covMatrix.resize(2, 2);
	meanVelocity = mean;
	covMatrix = cov;
	fluctation = std::make_shared<RandomnessGenerator>(meanVelocity, covMatrix);
}

Vector apa::Wind::getTrueVelocity()
{
	return meanVelocity + fluctation->getVectorRejection(); // plus vector fluctation
}



apa::Helicopter::Helicopter(const Vector& vecPos, const NatDist_ptr& wind, const std::shared_ptr<OnBoardSystem>& obs) : radiusVector(vecPos)
{
	this->wind = wind;
	OBS = obs;
}

void apa::Helicopter::move()
{
	auto locator = OBS->getLocator();
	auto AS = OBS->getAimingSystem();
	locator->location(AS->getVectorVelocityAiming());

	Vector vectorVelosityHeli = AS->getVectorVelocityAiming(locator->getVectorDelta_awesomeState());
	Vector next_radiusVector = radiusVector + (vectorVelosityHeli + wind->getTrueVelocity());

	radiusVector = next_radiusVector;
}

Vector apa::Helicopter::getVectorState()
{
	Vector vec(4);
	vec.segment(0, 2) = radiusVector;
	vec.segment(2, 2) = wind->getTrueVelocity();
	return vec;
}