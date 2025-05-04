#include "OnBoardLocatorShip.h"

apa::OnBoardLocatorShip::OnBoardLocatorShip(Vec2D position)
{
	ship = std::make_unique<Ship>(std::move(position));
}

void apa::OnBoardLocatorShip::explorStateCurrentTime_Ship(const double time, const double dt)
{
	ship->move(time, dt);
}

void apa::OnBoardLocatorShip::setFluctation(Vec2D mean, Mat2D cov)
{
	ship->setFluctation(mean, cov);
}

Eigen::Vector4d apa::OnBoardLocatorShip::getVectorState(const double time)
{
	return ship->getVectorState(time);
}