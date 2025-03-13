#pragma once
#include <Eigen/Dense>
#include <random>

namespace apa
{
	class RandomnessGenerator
	{
	public:
		RandomnessGenerator(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);
		Eigen::VectorXd getVectorRejection(const double& seed);

	private:
		std::mt19937 generator;
		std::normal_distribution<> dist;
		Eigen::VectorXd fluctation;
		Eigen::VectorXd mean;
		Eigen::MatrixXd cov;

	};
}