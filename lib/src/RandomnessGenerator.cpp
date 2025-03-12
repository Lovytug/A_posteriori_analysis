#pragma once
#include "RandomnessGenerator.h"

apa::RandomnessGenerator::RandomnessGenerator(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov)
{
	this->mean.resize(2);
	this->cov.resize(2, 2);

	this->mean = mean;
	this->cov = cov;
}

Eigen::VectorXd apa::RandomnessGenerator::getVectorRejection()
{
	generator.seed();

	Eigen::VectorXd result(mean.size());
	for (size_t i = 0; i < result.size(); i++)
		result[i] = dist(generator);

	return result;
}