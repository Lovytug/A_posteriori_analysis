#pragma once
#include "Transport.h"

#include <memory>

using NatDist_ptr = std::shared_ptr<apa::NaturalDisturbances>;

namespace apa
{
	class Waves : public NaturalDisturbances
	{
	public:
		Waves(const Vector& meanVelocity, const Matrix& covMatrix);

	protected:
		Vector getTrueVelocity(const double& time) override;

	private:
		std::shared_ptr<RandomnessGenerator> fluctation;
		Vector meanVelocity;
		Matrix covMatrix;
	};

	class Ship : public Transport
	{
	public:
		Ship(const Vector& vecPos, const NatDist_ptr& wave);

	protected:
		void move(const double& time, const double& dT) override;
		Vector getVectorState(const double& time) override;

	private:
		NatDist_ptr wave;
		Vector radiusVector;
	};
}