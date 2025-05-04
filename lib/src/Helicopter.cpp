#include "Helicopter.h"

apa::Wind::Wind(Vec2D mean, Mat2D cov)
{
	meanVelocity = mean;
	covMatrix = cov;
	fluctation = std::make_shared<RandomnessGenerator>(std::move(mean), std::move(cov));
}

apa::Vec2D apa::Wind::getTrueVelocity(const double time)
{
	return meanVelocity + fluctation->getVectorRejection(time, 0xBEEFFDDD); // plus vector fluctation
}


apa::Helicopter::Helicopter(Vec2D vecPos) : radiusVector(vecPos)
{
	AS = std::make_unique<AimingSystem>();
}

void apa::Helicopter::setFluctation(Vec2D mean, Mat2D cov)
{
	wind = std::make_unique<Wind>(std::move(mean), std::move(cov));
}

void apa::Helicopter::enableConnection(std::shared_ptr<apa::OnBoardLocatorHelicopter>& loc)
{
	locator = loc;
}

void apa::Helicopter::move(const double time, const double dt)
{
	Eigen::Vector4d vector_deltaEstimatedState = locator->getMissDataFromTarget(time, dt);
	Vec2D vectorVelocity = AS->getVectorVelocityAiming(std::move(vector_deltaEstimatedState), dt);
	radiusVector = radiusVector + dt * (vectorVelocity + wind->getTrueVelocity(time));
}

Eigen::Vector4d apa::Helicopter::getVectorState(const double time)
{
	return (Eigen::Vector4d() << radiusVector, wind->getTrueVelocity(time)).finished();
}