#pragma once
#include "Transport.h"

#include <memory>

namespace apa
{
	class Waves : public Noise
	{
	public:
		Waves(const Vector& meanVelocity, const Matrix& covMatrix);

	protected:
		Vector getTrueVelocity() override;

	private:
		std::shared_ptr<RandomnessGenerator> fluctation;
		Vector meanVelocity;
		Matrix covMatrix;
	};

	class Ship : public Transport
	{
	public:
		Ship(const Vector& vecPos, std::shared_ptr<Noise> wave);

	protected:
		void move() override;
		Vector getVectorState() override;

	private:
		std::shared_ptr<Noise> wave;
		Vector radiusVector;
	};
}