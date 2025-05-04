#include "OnBoardLocatorHelicopter.h"

apa::OnBoardLocatorHelicopter::OnBoardLocatorHelicopter(Vec2D position)
{
	helic = std::make_unique<Helicopter>(std::move(position));
}

void apa::OnBoardLocatorHelicopter::establishConnection(Locator_ptr& loc)
{
	helic->enableConnection(shared_from_this());
	locatorBase = loc;
}

void apa::OnBoardLocatorHelicopter::explorStateCurrentTime_Helicopter(const double time, const double dt)
{
	helic->move(time, dt);
}

void apa::OnBoardLocatorHelicopter::setFluctation(Vec2D mean, Mat2D cov)
{
	helic->setFluctation(std::move(mean), std::move(cov));
}

Eigen::Vector4d apa::OnBoardLocatorHelicopter::getMissDataFromTarget(const double time, const double dt)
{
	return locatorBase->filtringANDget_DeltaEstimatedVector_Targ_Helic(time, dt);
}

Eigen::Vector4d apa::OnBoardLocatorHelicopter::getVectorState(const double time)
{
	return helic->getVectorState(time);
}