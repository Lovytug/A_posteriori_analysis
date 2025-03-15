#include "OnBoardSystem.h"

apa::OnBoardSystem::OnBoardSystem(Trans_ptr& ship, Trans_ptr& heli, const Matrix& P, const double& dT)
{
	Vector vectorDelta_forecastState = ship->getVectorState(0.0) - heli->getVectorState(0.0);
	Vector vecNoise(2);
	vecNoise << 1, 1;
	Vector vectorMeas = vectorDelta_forecastState.segment(0, 2) + vecNoise;

	std::shared_ptr<KalmanFilter> KF = std::make_shared<KalmanFilter>(vectorMeas, vectorDelta_forecastState, P, dT);
	locator = std::make_shared<Locator>(ship, heli, KF);
	AS = std::make_shared<AimingSystem>(KF->getVectorDelta_awesomeState(), dT);
}

std::shared_ptr<apa::Locator> apa::OnBoardSystem::getLocator()
{
	return locator;
}

std::shared_ptr<apa::AimingSystem> apa::OnBoardSystem::getAimingSystem()
{
	return AS;
}