#include "AimingSystem.h"

apa::AimingSystem::AimingSystem()
{}

apa::Vec2D apa::AimingSystem::getVectorVelocityAiming(const Vec4D& vectorState, const double dT)
{
	Vec2D vecDelta_awePos = vectorState.segment(0, 2);
	Vec2D vecDelta_aweVel = vectorState.segment(2, 2);

	double div = 0.5 / dT;
	Vec2D result = vecDelta_awePos * div  + vecDelta_aweVel;

	if (!(getModuleVectorVelocity(result) <= maxModuleVelocity))
		vectorVelocityAiming = normalize(result);

	return vectorVelocityAiming;
}

double apa::AimingSystem::getModuleVectorVelocity(const Vec2D& vec)
{
	return vec.norm();
}

apa::Vec2D apa::AimingSystem::normalize(const Vec2D& vec)
{
	return vec / getModuleVectorVelocity(vec) * maxModuleVelocity;
}