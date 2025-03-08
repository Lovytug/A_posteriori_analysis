#include "Ship.h"

apa::Waves::Waves(const Vector& mean, const Matrix& cov) : meanVelocity(mean), covMatrix(cov)
{
	fluctation = std::make_shared<RandomnessGenerator>(meanVelocity, covMatrix);
}

Vector apa::Waves::getTrueVelocity()
{
	return meanVelocity; // plus vector fluctation
}


apa::Ship::Ship(const Vector& vecPos, std::shared_ptr<Noise> wave) : radiusVector(vecPos)
{
	this->wave = wave;
}

void apa::Ship::move()
{
	Vector next_radiusVector = radiusVector + wave->getTrueVelocity();
}

Vector apa::Ship::getVectorState()
{
	Vector vec(4);
	vec.segment(0, 2) = radiusVector;
	vec.segment(2, 2) = wave->getTrueVelocity();
	return vec;
}