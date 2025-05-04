#include "Ship.h"

apa::Wave::Wave(Vec2D mean, Mat2D cov)
{
	meanVelocity = mean;
	covMatrix = cov;
	fluctation = std::make_unique<RandomnessGenerator>(std::move(mean), std::move(cov));
}

apa::Vec2D apa::Wave::getTrueVelocity(const double time)
{
	return meanVelocity + fluctation->getVectorRejection(time, 0xDEADBEEF); // plus vector fluctation
}


apa::Ship::Ship(Vec2D vecPos) : radiusVector(std::move(vecPos))
{}

void apa::Ship::setFluctation(Vec2D mean, Mat2D cov)
{
	wave = std::make_unique<Wave>(std::move(mean), std::move(cov));
}

void apa::Ship::move(const double time, const double dT)
{
	if (wave != nullptr) radiusVector = radiusVector + dT * wave->getTrueVelocity(time);
	else return void();
}

Eigen::Vector4d apa::Ship::getVectorState(const double time)
{
	if (wave != nullptr) return (Eigen::Vector4d() << radiusVector, wave->getTrueVelocity(time)).finished();
	else return (Eigen::Vector4d() << Vec2D()).finished();
}