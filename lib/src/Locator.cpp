#include "Locator.h"

apa::SensorNoise::SensorNoise()
{
	vectorMean.resize(2);
	matrixCov.resize(2, 2);

	vectorMean << 0.0, 0.0;
	matrixCov << 10.0, 0.0,
		0.0, 10.0;

	noiseSensor = std::make_unique<RandomnessGenerator>(vectorMean, matrixCov);
}

apa::Vec2D apa::SensorNoise::getSensorNoise(const double time)
{
	return noiseSensor->getVectorRejection(time, 0xABCDEFDD);
}


apa::Locator::Locator(Vec4D beginMean, Mat4D beginCov, const double dt)
{
	KF = std::make_unique<KalmanFilter>(std::move(beginMean), std::move(beginCov), dt);
	noise = std::make_unique<SensorNoise>();
}

void apa::Locator::explorStateCurrentTime_Hunter(const double time, const double dt)
{
	locHelic->explorStateCurrentTime_Helicopter(time, dt);
}

void apa::Locator::explorStateCurrentTime_Target(const double time, const double dt)
{
	locShip->explorStateCurrentTime_Ship(time, dt);
}


void apa::Locator::createTargetWithPosition(Vec2D position)
{
	locShip = std::make_shared<OnBoardLocatorShip>(std::move(position));
}

void apa::Locator::createHunterWithPosition(Vec2D position)
{
	locHelic = std::make_shared<OnBoardLocatorHelicopter>(std::move(position));
	locHelic->establishConnection(shared_from_this());
}


void apa::Locator::setFluctationForTarget(Vec2D mean, Mat2D cov)
{
	locShip->setFluctation(std::move(mean), std::move(cov));
}

void apa::Locator::setFluctationForHunter(Vec2D mean, Mat2D cov)
{
	locHelic->setFluctation(std::move(mean), std::move(cov));
}


apa::Vec4D apa::Locator::filtringANDget_DeltaEstimatedVector_Targ_Helic(const double time, const double dt)
{
	Vec2D vec_delta_XY_noise = get_DetltaVector_Targ_Helic(time).head<2>() + noise->getSensorNoise(time);
	KF->perfomFiltring(std::move(vec_delta_XY_noise), dt);
	
	return KF->getVectorDelta_awesomeState();
}

apa::Vec4D apa::Locator::get_DeltaVectorEstimated_Targ_Helic()
{
	return KF->getVectorDelta_awesomeState();
}

Eigen::MatrixXd apa::Locator::getBorderOfConfidenceInterval()
{
	return KF->getMatrixOfConfidenceIntervalBoundsForEstimationVector();
}

apa::Vec4D apa::Locator::get_DetltaVector_Targ_Helic(const double time)
{
	return getVectorStateTarget(time) - getVectorStateHunter(time);
}

apa::Vec4D apa::Locator::getVectorStateTarget(const double time)
{
	return locShip->getVectorState(time);
}

apa::Vec4D apa::Locator::getVectorStateHunter(const double time)
{
	return locHelic->getVectorState(time);
}
