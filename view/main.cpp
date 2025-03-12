#include "MonitoringComplex.h"
#include <Eigen/Dense>
#include <iostream>

void main()
{
	Vector pos1(2);
	pos1 << 1000, 1000;

	Vector mean_wave(2);
	mean_wave << 2, 7;

	Matrix cov_wave(2, 2);
	cov_wave << 1, 0,
				0, 1;


	Vector pos2(2);
	pos2 << -1000, -1000;

	Vector mean_wind(2);
	mean_wind << 0.95, 0.7;

	Matrix cov_wind(2, 2);
	cov_wind << 0.25, 0.0,
				0.0, 0.25;


	NatDist_ptr wave = std::make_shared<apa::Waves>(mean_wave, cov_wave);
	NatDist_ptr wind = std::make_shared<apa::Wind>(mean_wind, cov_wind);

	Trans_ptr ship;
	Trans_ptr heli;
	std::shared_ptr<apa::Locator> locator = std::make_shared<apa::Locator>(heli, ship);
	ship = std::make_shared<apa::Ship>(pos1, wave);
	heli = std::make_shared<apa::Helicopter>(pos2, wind, locator);

	heli->move();
}