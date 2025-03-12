#include "MonitoringComplex.h"

apa::MonitoringComplex::MonitoringComplex(const Trans_ptr& obj1, const Trans_ptr& obj2, const double& allTime)
{
	helicopter = obj1;
	ship = obj2;
	currentTime = 0.0;
	durationOfGoalTracking = allTime;
}

