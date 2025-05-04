#include "MonitoringComplex.h"

apa::MonitoringComplex::MonitoringComplex(Vec4D beginMean, Mat4D beginCov, const double allTimeModeling, const double dt) :
	totalSimulationTime(allTimeModeling), stepSimulation(dt)
{
	locator = std::make_shared<Locator>(std::move(beginMean), std::move(beginCov), dt);
}

void apa::MonitoringComplex::startSimulation()
{
	double tik = stepSimulation;
	double allTime = totalSimulationTime;
	double currentTime = 0;
	while (allTime > currentTime)
	{
		locator->explorStateCurrentTime_Hunter(currentTime, tik);
		locator->explorStateCurrentTime_Target(currentTime, tik);
		currentTime += tik;
	}
}

void apa::MonitoringComplex::createTarget_Ship(Vec2D position) //поставить тип данных
{
	locator->createTargetWithPosition(std::move(position));
}

void apa::MonitoringComplex::createHunter_Helicopter(Vec2D position) // поставиить тип днных
{
	locator->createHunterWithPosition(std::move(position));
}

void apa::MonitoringComplex::setFluctationTarget_Ship(Vec2D mean, Mat2D cov)
{
	locator->setFluctationForTarget(std::move(mean), std::move(cov));
}

void apa::MonitoringComplex::setFluctationHunter_Helicopter(Vec2D mean, Mat2D cov)
{
	locator->setFluctationForHunter(std::move(mean), std::move(cov));
}