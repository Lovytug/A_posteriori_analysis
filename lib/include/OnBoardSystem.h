#pragma once
#include <memory>
#include "Locator.h"
#include "AimingSystem.h"

namespace apa
{
	class OnBoardSystem
	{
	public:
		OnBoardSystem(std::shared_ptr<Locator>& loc, std::shared_ptr<AimingSystem>& as);
		std::shared_ptr<Locator> getLocator();
		std::shared_ptr<AimingSystem> getAimingSystem();

	private:
		std::shared_ptr<Locator>& locator;
		std::shared_ptr<AimingSystem>& AS;
	};
}