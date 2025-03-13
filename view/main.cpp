#include "MonitoringComplex.h"
#include <Eigen/Dense>
#include <iostream>

void main()
{
	Vector mean_wave(2);
	mean_wave << 2, 7;

	Matrix cov_wave(2, 2);
	cov_wave << 1, 0,
				0, 1;

	Vector mean_wind(2);
	mean_wind << 0.95, 0.7;

	Matrix cov_wind(2, 2);
	cov_wind << 0.25, 0.0,
				0.0, 0.25;

	NatDist_ptr wave = std::make_shared<apa::Waves>(mean_wave, cov_wave);
	NatDist_ptr wind = std::make_shared<apa::Wind>(mean_wind, cov_wind);

	Vector pos1(2);
	pos1 << 1000, 1000;
	Vector pos2(2);
	pos2 << -1000, -1000;

	Matrix P(4, 4);
	P << 0.1, 0, 0, 0,
		0, 0.1, 0, 0,
		0, 0, 0.01, 0,
		0, 0, 0, 0.01;

	std::shared_ptr<apa::OnBoardSystem> OBS;
	Trans_ptr ship = std::make_shared<apa::Ship>(pos1, wave);
	Trans_ptr heli = std::make_shared<apa::Helicopter>(pos2, wind, OBS);

	OBS = std::make_shared<apa::OnBoardSystem>(ship, heli, P);

	double allTime = 1000;
	std::shared_ptr<apa::MonitoringComplex> monitor = std::make_shared<apa::MonitoringComplex>(ship, heli, OBS, allTime);


	monitor->trackMovementOfGoals();

}