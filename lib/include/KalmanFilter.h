#pragma once
#include <Eigen/Dense>

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

namespace apa
{
	class KalmanFilter
	{
	public:
		KalmanFilter(const Vector& vec, const Matrix& matrix);
		void perfomFiltring(const Vector& vecMeasurement, const Vector& vecVelocityAiminig);
		Vector getVectorDelta_awesomeState();
		Matrix getMatrixDelta_awesomeMistakeState();


	private:
		Vector vectorDelta_awesomeState; // X*
		Matrix matrixDelta_awesomeMistakeState; // P*
		Vector vectorDelta_forecastState; // X^
		Matrix matrixDelta_forecastMistakeState; // P^
		Matrix H;
		Matrix F;
		Matrix L;
		Matrix U;
		Matrix V;
		Matrix D_nu;
		Matrix D_ksi;
	};
}