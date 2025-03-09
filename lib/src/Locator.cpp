#include "Locator.h"

apa::SensorNoise::SensorNoise()
{
	vectorMean(2);
	matrixCov(4);

	vectorMean << 0.0, 0.0;
	matrixCov << 10.0, 0.0,
				 0.0, 10.0;
}


apa::Locator::Locator(const Trans_ptr& hunter, const Trans_ptr& target) : me(hunter), target(target)
{
	noiseDelta_position(2);
	noiseDelta_velocityFluct(2);
}

void apa::Locator::location()
{
	getDataInNoiseForm();
}

void apa::Locator::getDataInNoiseForm()
{
	
}

Vector apa::Locator::getVectorDelta_state()
{
	Vector vec(4);
	vec << getMeVectorState() - getTargetVectorState();
	return vec;
}
	