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



apa::Helicopter::Helicopter(const Vector& vecPos, const NatDist_ptr& wind, const std::shared_ptr<Locator>& loc) : radiusVector(vecPos)
{
	this->wind = wind;
	AS = std::make_shared<AimingSystem>();
	locator = loc;
}

void apa::Helicopter::move()
{
	locator->location();
	Vector vectorVelosityHeli = AS->getVectorVelocityAiming(locator->getVectorDelta_awesomeState());
	Vector next_radiusVector = radiusVector + (vectorVelosityHeli + wind->getTrueVelocity());
}

Vector apa::Helicopter::getVectorState()
{
	Vector vec(4);
	vec.segment(0, 2) = radiusVector;
	vec.segment(2, 2) = wind->getTrueVelocity();
	return vec;
}