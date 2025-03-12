#pragma once
#include "Transport.h"
#include "Locator.h"
#include "AimingSystem.h"
#include "OnBoardSystem.h"
#include <memory>

using Trans_ptr = std::shared_ptr<apa::Transport>;
using NatDist_ptr = std::shared_ptr<apa::NaturalDisturbances>;

namespace apa
{
	class Wind : public NaturalDisturbances
	{
	public:
		Wind(const Vector& meanVelocity, const Matrix& covMatrix);

	protected:
		Vector getTrueVelocity() override;

	private:
		std::shared_ptr<RandomnessGenerator> fluctation;
		Vector meanVelocity;
		Matrix covMatrix;
	};

	class Helicopter : public Transport
	{
	public:
		Helicopter(const Vector& vecPos, const NatDist_ptr& wind, const std::shared_ptr<OnBoardSystem>& obs);

	protected:
		void move() override;
		Vector getVectorState() override;

	private:
		NatDist_ptr wind;
		std::shared_ptr<Locator> locator;
		std::shared_ptr<AimingSystem> AS;
		std::shared_ptr<OnBoardSystem> OBS;
		Vector radiusVector;
	};
}