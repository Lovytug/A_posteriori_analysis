#pragma once
#include <memory>
#include "Transport.h"
#include "Locator.h"
#include "AimingSystem.h"

namespace apa
{
	class OnBoardSystem
	{
	public:
		OnBoardSystem(Trans_ptr& ship, Trans_ptr& heli, const Matrix& P, const double& dT);
		std::shared_ptr<Locator> getLocator();
		std::shared_ptr<AimingSystem> getAimingSystem();

	private:
		std::shared_ptr<Locator> locator;
		std::shared_ptr<AimingSystem> AS;
	};
}