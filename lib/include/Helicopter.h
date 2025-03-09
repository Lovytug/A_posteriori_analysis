#pragma once
#include "Transport.h"

#include "Locator.h"
#include <memory>

using Trans_ptr = std::shared_ptr<apa::Transport>;
using NatDist_ptr = std::shared_ptr<apa::NaturalDisturbances>;

namespace apa
{
	class Wind : NaturalDisturbances
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

	class Helicopter : Transport
	{
	public:
		Helicopter(const Vector& vecPos, const NatDist_ptr& wind, const Trans_ptr& target);

	protected:
		void move() override;
		Vector getVectorState() override;

	private:
		NatDist_ptr wind;
		std::shared_ptr<Locator> locator;
		std::shared_ptr<AimingSystem> AS;
		Vector radiusVector;
	};
}