#pragma once
#include "Transport.h"

#include <memory>

namespace apa
{
	class Wind : Noise
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
		Helicopter(const Vector& vecPos, std::shared_ptr<Noise> wind, std::shared_ptr<Transport> target);

	protected:
		void move() override;
		Vector getVectorState() override;

	private:
		std::shared_ptr<Noise> wind;
		std::shared_ptr<Locator> locator;
		std::shared_ptr<AimingSystem> AS;
		Vector radiusVector;
	};
}