#include "MonitoringComplex.h"
#include <iostream>

apa::MonitoringComplex::MonitoringComplex(const Trans_ptr& sh, const Trans_ptr& helic, OnBoard_ptr& OBS, const double& allTime)
{
	helicopter = helic;
	ship = sh;
	locator = OBS->getLocator();
	currentTime = 0.0;
	durationOfGoalTracking = allTime * SEC2;
}

void apa::MonitoringComplex::trackMovementOfGoals(const double& deltaT)
{
	double tik = deltaT;
	currentTime += tik;
	while (currentTime < durationOfGoalTracking)
	{
		helicopter->move(currentTime, deltaT);
		ship->move(currentTime, deltaT);

		appendToVector(vectorState_ship, ship->getVectorState(currentTime));
		appendToVector(vectorState_helicopter, helicopter->getVectorState(currentTime));

		appendToVector(vectorDelta_trueState, locator->getVectorDelta_state(currentTime));
		appendToVector(vectorDelta_awesomeState, locator->getVectorDelta_awesomeState());

		appendBoundaryOfConfidenceIntervalToVector(locator->getBorderOfConfidenceInterval());

		currentTime += tik;

		vectorTimes.push_back(currentTime);

		if(checkIntersection(ship->getVectorState(currentTime - tik), helicopter->getVectorState(currentTime - tik)))
			break;
	}
}

bool apa::MonitoringComplex::checkIntersection(const Vector& vec1, const Vector& vec2)
{
	if(abs(vec1[0] - vec2[0]) <= 1 && abs(vec1[1] - vec2[1]) <= 1)
		return true;
	else
		return false;
}

void apa::MonitoringComplex::appendToVector(Vector& vec, const Vector& newVec)
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

	double t = vectorUpperLimit_awesomeState[0];
	double r = vectorLowerLimit_awesomeState[0];
}

std::vector<double> apa::MonitoringComplex::transformInSTLvector(const Vector& vec)
{
	std::vector<double> stlVec(vec.data(), vec.data() + vec.size());
	return stlVec;
}

std::vector<double> apa::MonitoringComplex::stl_getVectorPositionX_ship()
{
	Vector result(vectorState_ship.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorState_ship(4 * i);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorPositionY_ship()
{
	Vector result(vectorState_ship.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorState_ship(4 * i + 1);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorPositionX_helicopter()
{
	Vector result(vectorState_helicopter.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorState_helicopter(4 * i);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorPositionY_helicopter()
{
	Vector result(vectorState_helicopter.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorState_helicopter(4 * i + 1);

	return transformInSTLvector(result);
}

// --- true and awesome state --- //

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaPosX_trueState()
{
	Vector result(vectorDelta_trueState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorDelta_trueState(4 * i);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaPosY_trueState()
{
	Vector result(vectorDelta_trueState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorDelta_trueState(4 * i + 1);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaVelX_trueState()
{
	double t = vectorDelta_trueState[2];
	double t1 = vectorDelta_trueState[6];
	Vector result(vectorDelta_trueState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorDelta_trueState(4 * i + 2);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaVelY_trueState()
{
	Vector result(vectorDelta_trueState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorDelta_trueState(4 * i + 3);

	return transformInSTLvector(result);
}



std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaPosX_awesomeState()
{
	Vector result(vectorDelta_awesomeState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorDelta_awesomeState(4 * i);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaPosY_awesomeState()
{
	Vector result(vectorDelta_awesomeState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorDelta_awesomeState(4 * i + 1);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaVelX_awesomeState()
{
	Vector result(vectorDelta_awesomeState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorDelta_awesomeState(4 * i + 2);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorDeltaVelY_awesomeState()
{
	Vector result(vectorDelta_awesomeState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorDelta_awesomeState(4 * i + 3);

	return transformInSTLvector(result);
}


// --- upper and low limit --- //

std::vector<double> apa::MonitoringComplex::stl_getVectorUpperLimit_DeltaPosX_awesomeState()
{
	Vector result(vectorUpperLimit_awesomeState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorUpperLimit_awesomeState(4 * i);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorUpperLimit_DeltaPosY_awesomeState()
{
	Vector result(vectorUpperLimit_awesomeState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorUpperLimit_awesomeState(4 * i + 1);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorUpperLimit_DeltaVelX_awesomeState()
{
	Vector result(vectorUpperLimit_awesomeState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorUpperLimit_awesomeState(4 * i + 2);

	return transformInSTLvector(result);
}
std::vector<double> apa::MonitoringComplex::stl_getVectorUpperLimit_DeltaVelY_awesomeState()
{
	Vector result(vectorUpperLimit_awesomeState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorUpperLimit_awesomeState(4 * i + 3);

	return transformInSTLvector(result);
}


std::vector<double> apa::MonitoringComplex::stl_getVectorLowerLimit_DeltaPosX_awesomeState()
{
	Vector result(vectorLowerLimit_awesomeState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorLowerLimit_awesomeState(4 * i);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorLowerLimit_DeltaPosY_awesomeState()
{
	Vector result(vectorLowerLimit_awesomeState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorLowerLimit_awesomeState(4 * i + 1);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorLowerLimit_DeltaVelX_awesomeState()
{
	Vector result(vectorLowerLimit_awesomeState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorLowerLimit_awesomeState(4 * i + 2);

	return transformInSTLvector(result);
}

std::vector<double> apa::MonitoringComplex::stl_getVectorLowerLimit_DeltaVelY_awesomeState()
{
	Vector result(vectorLowerLimit_awesomeState.size() / 4);
	for (int i = 0; i < result.size(); i++)
		result[i] = vectorLowerLimit_awesomeState(4 * i + 3);

	return transformInSTLvector(result);
}


std::vector<double> apa::MonitoringComplex::getVectorTime()
{
	return vectorTimes;
}