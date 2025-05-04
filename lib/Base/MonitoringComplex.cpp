#include "MonitoringComplex.h"

apa::MonitoringComplex::MonitoringComplex(Vec4D beginMean, Mat4D beginCov, const double allTimeModeling, const double dt) :
	totalSimulationTime(allTimeModeling), stepSimulation(dt)
{
	locator = std::make_shared<Locator>(std::move(beginMean), std::move(beginCov), dt);
}

void apa::MonitoringComplex::createTarget_Ship(Vec2D position)
{
	locator->createTargetWithPosition(std::move(position));
}

void apa::MonitoringComplex::createHunter_Helicopter(Vec2D position)
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

void apa::MonitoringComplex::startSimulation()
{
	double tik = stepSimulation;
	double allTime = totalSimulationTime;
	double currentTime = 0;

	while (allTime > currentTime)
	{
		appendToVector(vectorDelta_trueState, locator->get_DetltaVector_Targ_Helic(currentTime));

		locator->explorStateCurrentTime_Hunter(currentTime, tik);
		locator->explorStateCurrentTime_Target(currentTime, tik);

		appendToVector(vectorState_ship, locator->getVectorStateTarget(currentTime));
		appendToVector(vectorState_helicopter, locator->getVectorStateHunter(currentTime));

		appendToVector(vectorDelta_awesomeState, locator->get_DeltaVectorEstimated_Targ_Helic());

		appendBoundaryOfConfidenceIntervalToVector(locator->getBorderOfConfidenceInterval());

		currentTime += tik;
		vectorTimes.push_back(currentTime);

		if (checkIntersection(locator->getVectorStateTarget(currentTime - tik), 
			locator->getVectorStateHunter(currentTime - tik))) break;
	}
}

bool apa::MonitoringComplex::checkIntersection(const Vec4D& vec1, const Vec4D& vec2)
{
	return (abs(vec1[0] - vec2[0]) <= 1 && abs(vec1[1] - vec2[1]) <= 1);
}

void apa::MonitoringComplex::appendToVector(VecND& vec, const Vec4D& newVec)
{
	int oldSize = vec.size();
	vec.conservativeResize(oldSize + newVec.size());
	vec.segment(oldSize, newVec.size()) = newVec;
}

void apa::MonitoringComplex::appendBoundaryOfConfidenceIntervalToVector(const Matrix& matrix)
{
	int oldSizeU = vectorUpperLimit_awesomeState.size();
	vectorUpperLimit_awesomeState.conservativeResize(oldSizeU + matrix.rows());
	vectorUpperLimit_awesomeState.segment(oldSizeU, matrix.rows()) = matrix.col(0);

	int oldSizeL = vectorLowerLimit_awesomeState.size();
	vectorLowerLimit_awesomeState.conservativeResize(oldSizeL + matrix.rows());
	vectorLowerLimit_awesomeState.segment(oldSizeL, matrix.rows()) = matrix.col(1);
}



std::vector<double> apa::MonitoringComplex::transformInSTLvector(const VecND& vec) const
{
	return std::vector<double>(vec.data(), vec.data() + vec.size());
}

std::vector<double> apa::MonitoringComplex::stl_getVectorPositionX_ship() const
{
	return extractComponentVector<0>(vectorState_ship);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorPositionY_ship() const
{
	return extractComponentVector<1>(vectorState_ship);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorPositionX_helicopter() const
{
	return extractComponentVector<0>(vectorState_helicopter);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorPositionY_helicopter() const
{
	return extractComponentVector<1>(vectorState_helicopter);
}

// --- true and awesome state --- //

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaPosX_trueState() const
{
	return extractComponentVector<0>(vectorDelta_trueState);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaPosY_trueState() const
{
	return extractComponentVector<1>(vectorDelta_trueState);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaVelX_trueState() const
{
	return extractComponentVector<2>(vectorDelta_trueState);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaVelY_trueState() const
{
	return extractComponentVector<3>(vectorDelta_trueState);
}



std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaPosX_awesomeState() const
{
	return extractComponentVector<0>(vectorDelta_awesomeState);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaPosY_awesomeState() const
{
	return extractComponentVector<1>(vectorDelta_awesomeState);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaVelX_awesomeState() const
{
	return extractComponentVector<2>(vectorDelta_awesomeState);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaVelY_awesomeState() const
{
	return extractComponentVector<3>(vectorDelta_awesomeState);
}


// --- upper and low limit --- //

std::vector<double> apa::MonitoringComplex::stl_getVectorUpperLimit_DeltaPosX_awesomeState() const
{
	return extractComponentVector<0>(vectorUpperLimit_awesomeState);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorUpperLimit_DeltaPosY_awesomeState() const
{
	return extractComponentVector<1>(vectorUpperLimit_awesomeState);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorUpperLimit_DeltaVelX_awesomeState() const
{
	return extractComponentVector<2>(vectorUpperLimit_awesomeState);
}
std::vector<double> apa::MonitoringComplex::stl_getVectorUpperLimit_DeltaVelY_awesomeState() const
{
	return extractComponentVector<3>(vectorUpperLimit_awesomeState);
}


std::vector<double> apa::MonitoringComplex::stl_getVectorLowerLimit_DeltaPosX_awesomeState() const
{
	return extractComponentVector<0>(vectorLowerLimit_awesomeState);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorLowerLimit_DeltaPosY_awesomeState() const
{
	return extractComponentVector<1>(vectorLowerLimit_awesomeState);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorLowerLimit_DeltaVelX_awesomeState() const
{
	return extractComponentVector<2>(vectorLowerLimit_awesomeState);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorLowerLimit_DeltaVelY_awesomeState() const
{
	return extractComponentVector<3>(vectorLowerLimit_awesomeState);
}

std::vector<double> apa::MonitoringComplex::getVectorTime()
{
	return vectorTimes;
}