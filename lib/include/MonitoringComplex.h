#pragma once
#include <memory>
#include <Eigen/Dense>
#include "Locator.h"

namespace apa
{
	using Vec2D = Eigen::Vector2d;
	using Vec4D = Eigen::Vector4d;
	using Mat2D = Eigen::Matrix2d;
	using Mat4D = Eigen::Matrix4d;

	using Locator_ptr = std::shared_ptr<Locator>;

	class MonitoringComplex
	{
	public:
		MonitoringComplex(Vec4D beginMean, Mat4D beginCov, const double allTimeModeling, const double dt);

		void startSimulation();

		void createTarget_Ship(Vec2D position);
		void createHunter_Helicopter(Vec2D position);
		void setFluctationTarget_Ship(Vec2D mean, Mat2D cov);
		void setFluctationHunter_Helicopter(Vec2D mean, Mat2D cov);

		std::vector<double> getVectorTime();

		std::vector<double> stl_getVectorPositionX_ship() const;
		std::vector<double> stl_getVectorPositionY_ship() const;
		std::vector<double> stl_getVectorPositionX_helicopter() const;
		std::vector<double> stl_getVectorPositionY_helicopter() const;

		std::vector<double> stl_getVectorDeltaPosX_trueState() const;
		std::vector<double> stl_getVectorDeltaPosY_trueState() const;
		std::vector<double> stl_getVectorDeltaVelX_trueState() const;
		std::vector<double> stl_getVectorDeltaVelY_trueState() const;

		std::vector<double> stl_getVectorDeltaPosX_awesomeState() const;
		std::vector<double> stl_getVectorDeltaPosY_awesomeState() const;
		std::vector<double> stl_getVectorDeltaVelX_awesomeState() const;
		std::vector<double> stl_getVectorDeltaVelY_awesomeState() const;

		std::vector<double> stl_getVectorUpperLimit_DeltaPosX_awesomeState() const;
		std::vector<double> stl_getVectorUpperLimit_DeltaPosY_awesomeState() const;
		std::vector<double> stl_getVectorUpperLimit_DeltaVelX_awesomeState() const;
		std::vector<double> stl_getVectorUpperLimit_DeltaVelY_awesomeState() const;

		std::vector<double> stl_getVectorLowerLimit_DeltaPosX_awesomeState() const;
		std::vector<double> stl_getVectorLowerLimit_DeltaPosY_awesomeState() const;
		std::vector<double> stl_getVectorLowerLimit_DeltaVelX_awesomeState() const;
		std::vector<double> stl_getVectorLowerLimit_DeltaVelY_awesomeState() const;

	private:
		Locator_ptr locator;
		double totalSimulationTime;
		double stepSimulation;
		std::vector<double> vectorTimes;

		Eigen::VectorXd vectorState_ship;
		Eigen::VectorXd vectorState_helicopter;
		Eigen::VectorXd vectorDelta_trueState;
		Eigen::VectorXd vectorDelta_awesomeState;

		Eigen::VectorXd vectorUpperLimit_awesomeState;
		Eigen::VectorXd vectorLowerLimit_awesomeState;

		void appendToVector(Vector& vec, const Vector& newVec);
		void appendBoundaryOfConfidenceIntervalToVector(const Matrix& matrixBorder);
		bool checkIntersection(const Vector& vec1, const Vector& vec2);
		std::vector<double> transformInSTLvector(const Vector& vec) const;

		template<size_t Index>
		std::vector<double> extractComponentVector(const Vector& vec) const;
	};
}

template<size_t Index>
std::vector<double> apa::MonitoringComplex::extractComponentVector(const Vector& vec) const
{
	Vector result(vec.size() / 4);
	for (int i = 0; i < result.size(); ++i)
		result[i] = vec(4 * i + Index);

	return transformInSTLvector(result);
}