#include "Helicopter.h"

apa::Wind::Wind(const Vector& mean, const Matrix& cov) : meanVelocity(mean), covMatrix(mean)
{
	fluctation = std::make_shared<RandomnessGenerator>(meanVelocity, covMatrix);
}

Vector apa::Wind::getTrueVelocity()
{
	return meanVelocity; // plus vector fluctation
}


apa::Helicopter::Helicopter(const Vector& vecPos, std::shared_ptr<Noise> wind, std::shared_ptr<Transport> target) : radiusVector(vecPos)
{
	this->wind = wind;
	locator = std::make_shared<Locator>(this, target);
	AS = std::make_shared<AimingSystem>(locator);
}

void apa::Helicopter::move()
{

}

Vector apa::Helicopter::getVectorState()
{
	Vector vec(4);
	vec.segment(0, 2) = radiusVector;
	vec.segment(2, 2) = wind->getTrueVelocity();
	return vec;
}