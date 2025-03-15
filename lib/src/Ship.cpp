#include "Ship.h"

apa::Waves::Waves(const Vector& mean, const Matrix& cov)
{
	meanVelocity.resize(2);
	covMatrix.resize(2, 2);
	meanVelocity = mean;
	covMatrix = cov;
	fluctation = std::make_shared<RandomnessGenerator>(meanVelocity, covMatrix);
}

Vector apa::Waves::getTrueVelocity(const double& time)
{
	return meanVelocity + fluctation->getVectorRejection(time, 0xDEADBEEF); // plus vector fluctation
}



apa::Ship::Ship(const Vector& vecPos, const NatDist_ptr& wave) : radiusVector(vecPos)
{
	this->wave = wave;
}

void apa::Ship::move(const double& time, const double& dT)
{
	Vector next_radiusVector = radiusVector + dT * wave->getTrueVelocity(time);
	
	radiusVector = next_radiusVector;
}

Vector apa::Ship::getVectorState(const double& time)
{
	Vector vec(4);
	vec.segment(0, 2) = radiusVector;
	vec.segment(2, 2) = wave->getTrueVelocity(time);
	return vec;
}