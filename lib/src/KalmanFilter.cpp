#include "KalmanFilter.h"

apa::KalmanFilter::KalmanFilter(const Vector& vec_forecast, const Matrix& matrix_forecast)
{
	int32_t n = 4;
	int32_t l = 2;
	int32_t k = 2;
	int32_t m = 4;
	vectorDelta_awesomeState.resize(n);
	matrixDelta_awesomeMistakeState.resize(n, n);
	vectorDelta_forecastState.resize(n);
	matrixDelta_forecastMistakeState.resize(n, n);
	vectorDelta_forecastState = vec_forecast;
	matrixDelta_forecastMistakeState = matrix_forecast;


	F.resize(n, n);
	F << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;

	H.resize(l, n);
	H << 1, 0, 0, 0,
		0, 1, 0, 0;

	D_nu.resize(l, l); // matrix cov locator
	D_nu << 10, 0,
			0, 10;

	U.resize(k, 1); // dynamic add Vh ~ aimingVelisity
	U.setZero();

	V.resize(n, k);
	V << -1, 0,
		0, -1,
		0, 0,
		0, 0;

	L.resize(n, m);
	L << 0, 0, 0, 0,
		0, 0, 0, 0,
		1, 0, -1, 0,
		0, 1, 0, -1;

	D_ksi.resize(m, m); // matrix cov wind and wave
	D_ksi << 0.25, 0, 0, 0,
			0, 0.25, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	
}

void apa::KalmanFilter::perfomFiltring(const Vector& vecMeas, const Vector& vecVelH)
{
	int size = matrixDelta_forecastMistakeState.size();

	Matrix inverse_matrixForecast = matrixDelta_forecastMistakeState.inverse();
	Matrix H_tr = H.transpose();
	Matrix D_nu_in = D_nu.inverse();
	matrixDelta_awesomeMistakeState = (inverse_matrixForecast + H_tr * D_nu_in * H).inverse();
	vectorDelta_awesomeState = vectorDelta_forecastState + matrixDelta_awesomeMistakeState * H_tr * D_nu_in * (vecMeas - H * vectorDelta_forecastState);

	double r = vectorDelta_awesomeState[0];
	double v = vectorDelta_awesomeState[1];
	double v2 = vectorDelta_awesomeState[2];
	double v3 = vectorDelta_awesomeState[3];


	Matrix next_matrixDelta_forecastMistState(4, 4);
	Vector next_vectorDelta_forecastState(4);

	U = vecVelH;
	next_matrixDelta_forecastMistState = F * matrixDelta_awesomeMistakeState * F.transpose() + L * D_ksi * L.transpose();
	next_vectorDelta_forecastState = F * vectorDelta_awesomeState + V * U;


	matrixDelta_forecastMistakeState = next_matrixDelta_forecastMistState;
	vectorDelta_forecastState = next_vectorDelta_forecastState;
}

Vector apa::KalmanFilter::getVectorDelta_awesomeState()
{
	return vectorDelta_awesomeState;
}

Matrix apa::KalmanFilter::getMatrixDelta_awesomeMistakeState()
{
	return matrixDelta_awesomeMistakeState;
}

Matrix apa::KalmanFilter::getMatrixOfConfidenceIntervalBoundsForEstimationVector() // up - low
{
	Matrix result(4, 2);

	double confidenceProbability = 3.0;

	Vector vectorBorder = confidenceProbability * (matrixDelta_awesomeMistakeState.diagonal()).array().sqrt();

	Vector upperLimit = vectorDelta_awesomeState + vectorBorder;
	Vector lowerLimit = vectorDelta_awesomeState - vectorBorder;

	result.col(0) = upperLimit;
	result.col(1) = lowerLimit;

	return result;
}