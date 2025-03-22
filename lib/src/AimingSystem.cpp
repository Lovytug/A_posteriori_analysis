#include "AimingSystem.h"

apa::AimingSystem::AimingSystem()
{
	vectorVelocityAiming.resize(2);
}

Vector apa::AimingSystem::getVectorVelocityAiming(const Vector& vectorState, const double& dT)
{
	Vector vecDelta_awePos = vectorState.segment(0, 2);
	Vector vecDelta_aweVel = vectorState.segment(2, 2);

	double div = 0.5 * 1.0 / dT;
	Vector result = vecDelta_awePos * div  + vecDelta_aweVel;

	if (!(getModuleVectorVelocity(result) <= maxModuleVelocity))
		vectorVelocityAiming = normalize(result);

	return vectorVelocityAiming;
}

double apa::AimingSystem::getModuleVectorVelocity(const Vector& vec)
{
	return vec.norm();
}

Vector apa::AimingSystem::normalize(const Vector& vec)
{
	return vec / getModuleVectorVelocity(vec) * maxModuleVelocity;
}