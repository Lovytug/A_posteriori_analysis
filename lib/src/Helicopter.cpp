#include "Helicopter.h"

apa::Wind::Wind(const Vector& mean, const Matrix& cov)
{
	meanVelocity.resize(2);
	covMatrix.resize(2, 2);
	meanVelocity = mean;
	covMatrix = cov;
	fluctation = std::make_shared<RandomnessGenerator>(meanVelocity, covMatrix);
}

Vector apa::Wind::getTrueVelocity(const double& time)
{
	return meanVelocity + fluctation->getVectorRejection(time, 0xBEEFFDDD); // plus vector fluctation
}



apa::Helicopter::Helicopter(const Vector& vecPos, const NatDist_ptr& wind, std::shared_ptr<OnBoardSystem>& obs) : 
	radiusVector(vecPos), wind(wind), OBS(obs) {}

void apa::Helicopter::move(const double& time, const double& dT)
{
	auto locator = OBS->getLocator();
	auto AS = OBS->getAimingSystem();
	locator->location(AS->getVectorVelocityAiming(), time);

	Vector vectorVelosityHeli = AS->getVectorVelocityAiming(locator->getVectorDelta_awesomeState(), dT);
	Vector next_radiusVector = radiusVector + dT * (vectorVelosityHeli + wind->getTrueVelocity(time));

	radiusVector = next_radiusVector;
}

Vector apa::Helicopter::getVectorState(const double& time)
{
	Vector vec(4);
	vec.segment(0, 2) = radiusVector;
	vec.segment(2, 2) = wind->getTrueVelocity(time);
	return vec;
}