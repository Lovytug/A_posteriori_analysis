#include "AimingSystem.h"

apa::AimingSystem::AimingSystem()
{
	vectorVelocityAiming.resize(2);
	vectorVelocityAiming.setZero();
}

Vector apa::AimingSystem::getVectorVelocityAiming()
{
	return vectorVelocityAiming;
}

Vector apa::AimingSystem::getVectorVelocityAiming(const Vector& vectorState)
{
	Vector vecDelta_awePos = vectorState.segment(0, 2);
	Vector vecDelta_aweVel = vectorState.segment(0, 2);

	Vector result = vecDelta_awePos / 2.0 + vecDelta_aweVel;

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