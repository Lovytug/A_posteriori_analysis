#pragma once
#include <Eigen/Dense>

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

namespace apa
{
	class KalmanFilter
	{
	public:
		KalmanFilter();
		void porfomFiltring(const Vector& vec);
		Vector getVectorDelta_awesomeState();

	private:
		Vector vectorDelta_awesomeState;
		Matrix H;
		Matrix F;
		Matrix P;
	};
}