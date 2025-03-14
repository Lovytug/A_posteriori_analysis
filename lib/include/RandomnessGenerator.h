#pragma once
#include <Eigen/Dense>
#include <random>

namespace apa
{
	class RandomnessGenerator
	{
	public:
		RandomnessGenerator(const Eigen::VectorXd& mean, const Eigen::MatrixXd& cov);
		Eigen::VectorXd getVectorRejection(const double& seed, uint32_t uniqueOffset);

	private:
		std::mt19937 generator;
		std::normal_distribution<> dist;
		Eigen::VectorXd fluctation;
		Eigen::VectorXd mean;
		Eigen::MatrixXd cov;

		static std::mt19937 globalGenerator; // Глобальный генератор для уникальных seed'ов
		static bool isGlobalGeneratorInitialized;
	};
}