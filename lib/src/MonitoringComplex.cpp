#include "MonitoringComplex.h"
#include <iostream>

apa::MonitoringComplex::MonitoringComplex(const Trans_ptr& sh, const Trans_ptr& helic, OnBoard_ptr& OBS, const double& allTime)
{
	helicopter = helic;
	ship = sh;
	locator = OBS->getLocator();
	currentTime = 0.0;
	durationOfGoalTracking = allTime;
}

void apa::MonitoringComplex::trackMovementOfGoals()
{
	double tik = 1.0;
	while (currentTime < durationOfGoalTracking)
	{
		ship->move(currentTime);
		helicopter->move(currentTime);

		appendToVector(vectorState_ship, ship->getVectorState(currentTime));
		appendToVector(vectorState_helicopter, helicopter->getVectorState(currentTime));

		appendToVector(vectorDelta_trueState, locator->getVectorDelta_state(currentTime));
		appendToVector(vectorDelta_awesomeState, locator->getVectorDelta_awesomeState());

		currentTime += tik;
	}
}

void apa::MonitoringComplex::appendToVector(Vector& vec, const Vector& newVec)
{
	int oldSize = vec.size();
	vec.conservativeResize(oldSize + newVec.size());
	vec.segment(oldSize, newVec.size()) = newVec;
}
