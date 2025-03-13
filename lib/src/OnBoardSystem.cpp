#include "OnBoardSystem.h"

apa::OnBoardSystem::OnBoardSystem(Trans_ptr& ship, Trans_ptr& heli, const Matrix& P)
{
	Vector vectorDelta_forecastState = ship->getVectorState(0.0) - heli->getVectorState(0.0);
	std::shared_ptr<KalmanFilter> KF = std::make_shared<KalmanFilter>(vectorDelta_forecastState, P);
	locator = std::make_shared<Locator>(ship, heli, KF);
	AS = std::make_shared<AimingSystem>();
}

std::shared_ptr<apa::Locator> apa::OnBoardSystem::getLocator()
{
	return locator;
}

std::shared_ptr<apa::AimingSystem> apa::OnBoardSystem::getAimingSystem()
{
	return AS;
}