#pragma once
#include "RandomnessGenerator.h"

std::mt19937 apa::RandomnessGenerator::globalGenerator;
bool apa::RandomnessGenerator::isGlobalGeneratorInitialized = false;

apa::RandomnessGenerator::RandomnessGenerator(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov)
{
	this->mean.resize(2);
	this->cov.resize(2, 2);

	this->mean = mean;
	this->cov = cov;

	if (!isGlobalGeneratorInitialized) {
		std::random_device rd;
		globalGenerator.seed(rd());
		isGlobalGeneratorInitialized = true;
	}
}

Eigen::VectorXd apa::RandomnessGenerator::getVectorRejection(const double& seed, uint32_t uniqueOffset)
{
	uint64_t timeAsInt = static_cast<uint64_t>(seed * 1000);
	uint32_t unicSeed = static_cast<uint32_t>(timeAsInt ^ uniqueOffset);
	generator.seed(unicSeed);

	Eigen::VectorXd result(mean.size());
	for (size_t i = 0; i < result.size(); i++)
		result[i] = dist(generator);

	fluctation = result;
	return result;
}