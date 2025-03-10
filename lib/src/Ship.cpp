#include "Ship.h"

apa::Waves::Waves(const Vector& mean, const Matrix& cov)
{
	meanVelocity(2);
	covMatrix(4);
	meanVelocity = mean;
	covMatrix = cov;
	fluctation = std::make_shared<RandomnessGenerator>(meanVelocity, covMatrix);
}

Vector apa::Waves::getTrueVelocity()
{
	return meanVelocity; // plus vector fluctation
}


apa::Ship::Ship(const Vector& vecPos, const NatDist_ptr& wave) : radiusVector(vecPos)
{
	this->wave = wave;
}

void apa::Ship::move()
{
	Vector next_radiusVector = radiusVector + wave->getTrueVelocity();
	
	radiusVector = next_radiusVector;
}

Vector apa::Ship::getVectorState()
{
	Vector vec(4);
	vec.segment(0, 2) = radiusVector;
	vec.segment(2, 2) = wave->getTrueVelocity();
	return vec;
}