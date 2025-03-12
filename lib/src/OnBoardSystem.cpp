#include "OnBoardSystem.h"

apa::OnBoardSystem::OnBoardSystem(std::shared_ptr<Locator>& loc, std::shared_ptr<AimingSystem>& as) : locator(loc), AS(as)
{}

std::shared_ptr<apa::Locator> apa::OnBoardSystem::getLocator()
{
	return locator;
}

std::shared_ptr<apa::AimingSystem> apa::OnBoardSystem::getAimingSystem()
{
	return AS;
}