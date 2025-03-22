#include "OnBoardSystem.h"

apa::OnBoardSystem::OnBoardSystem(Trans_ptr& ship, Trans_ptr& heli, const Vector& X, const Matrix& P, const double& dT)
{
	std::shared_ptr<KalmanFilter> KF = std::make_shared<KalmanFilter>(X, P, dT);
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