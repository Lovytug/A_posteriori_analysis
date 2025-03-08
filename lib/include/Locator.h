#pragma once
#include "Transport.h"

#include "KalmanFilter.h"
#include <memory>
using Trans_ptr = std::shared_ptr<apa::Transport>;

namespace apa
{
	class Locator
	{
	public:
		Locator(const Trans_ptr& hunter, const Trans_ptr& target);
		void location();
		void getVectorDelta_state();
		void getVectorDelta_awesomeState(); //‘ 

	private:
		Trans_ptr me;
		Trans_ptr target;
		Vector delta_position;
		Vector delta_velocityFluct;

		Vector getMeVectorState();
		Vector getTargetVectorState();
		Vector transformVectorStateInVecDelta_position();
		Vector transformVectorStateInVecDelta_velocityFluct();

	};
}