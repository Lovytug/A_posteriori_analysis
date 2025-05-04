#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <matplot/matplot.h>
#include "MonitoringComplex.h"

using stlVec_doub = std::vector<double>;

void drawPlot(const stlVec_doub& X1, const stlVec_doub& Y1, const stlVec_doub& X2, const stlVec_doub& Y2,
	const std::string& label1, const std::string& label2, const std::string xlabel, const std::string ylabel)
{
	auto fig = matplot::figure();
	fig->size(800, 800);
	fig->position(0, 0);
	matplot::xlabel(xlabel);
	matplot::ylabel(ylabel);
	matplot::hold(true);
	auto plot1 = matplot::plot(X1, Y1);
	plot1->color("blue");
	plot1->line_width(2);
	plot1->display_name(label1);
	auto plot2 = matplot::plot(X2, Y2);
	plot2->color("red");
	plot2->line_width(2);
	plot2->display_name(label2);
	matplot::legend();
}

void drawPlot(const stlVec_doub& X1, const stlVec_doub& X2, const stlVec_doub& time, 
	const std::string& label1, const std::string& label2, const std::string xlabel, const std::string ylabel)
{
	auto fig = matplot::figure();
	fig->size(800, 800);
	matplot::xlabel(xlabel);
	matplot::ylabel(ylabel);
	matplot::hold(true);
	fig->position(500, 0);
	auto plot1 = matplot::plot(time, X1);
	plot1->color("blue");
	plot1->line_width(2);
	plot1->display_name(label1);
	auto plot2 = matplot::plot(time, X2);
	plot2->color("red");
	plot2->line_width(2);
	plot2->display_name(label2);
	matplot::legend();
}

void drawPlot(const stlVec_doub& X, const stlVec_doub& UP, const stlVec_doub& LOW, const stlVec_doub& time, 
	const std::string& label, const std::string xlabel, const std::string ylabel)
{
	auto fig = matplot::figure();
	fig->size(800, 800);
	fig->position(1000, 0);
	matplot::xlabel(xlabel);
	matplot::ylabel(ylabel);
	matplot::hold(true);
	auto plot = matplot::plot(time, X);
	plot->color("green");
	plot->line_width(2);
	plot->display_name(label);
	auto plot1 = matplot::plot(time, UP);
	plot1->color("blue");
	plot1->line_width(2);
	plot1->display_name("upper limimt");
	auto plot2 = matplot::plot(time, LOW);
	plot2->color("red");
	plot2->line_width(2);
	plot2->display_name("lower limit");
	matplot::legend();
}

std::vector<double> operator-(const std::vector<double>& lhs, const std::vector<double>& rhs) {
	// Проверяем, что размеры векторов совпадают
	if (lhs.size() != rhs.size()) {
		throw std::invalid_argument("Размеры векторов не совпадают");
	}

	// Создаем результирующий вектор
	std::vector<double> result(lhs.size());

	// Выполняем поэлементное вычитание
	for (size_t i = 0; i < lhs.size(); ++i) {
		result[i] = lhs[i] - rhs[i];
	}

	return result;
}

void main()
{
	using namespace apa;

	double time = 45;
	double dt = 0.01;
	Vec4D aPOS{ -2, 5, 1, -3.75 };
	Mat4D aCOV = Mat4D::Zero();
	aCOV.diagonal() << 10e7, 10e7, 100, 100;
	MonitoringComplex monitor(aPOS, aCOV, time, dt);


	Vec2D pos1{ 700, 700 };
	Vec2D pos2{ -500, 0 };
	monitor.createTarget_Ship(pos1);
	monitor.createHunter_Helicopter(pos2);

	Vec2D mean1{ 0.95, 0.7 };
	Vec2D mean2{ 2, 7 };
	Mat2D cov1 = Mat2D::Zero();
	cov1.diagonal() << 0.25, 0.25;
	Mat2D cov2 = Mat2D::Zero();
	cov2.diagonal() << 1, 1;
	monitor.setFluctationTarget_Ship(mean1, cov1);
	monitor.setFluctationHunter_Helicopter(mean2, cov2);

	monitor.startSimulation();


	auto vecTime = monitor.getVectorTime();

	auto posX_ship = monitor.stl_getVectorPositionX_ship();
	auto posY_ship = monitor.stl_getVectorPositionY_ship();
	auto posX_heli = monitor.stl_getVectorPositionX_helicopter();
	auto posY_heli = monitor.stl_getVectorPositionY_helicopter();
	drawPlot(posX_ship, posY_ship, posX_heli, posY_heli, "ship", "helicopter", "coord X, m", "coord Y, m");

	auto delPosX_tr = monitor.stl_getVectorDeltaPosX_trueState();
	auto delPosY_tr = monitor.stl_getVectorDeltaPosY_trueState();
	auto delPosX_aw = monitor.stl_getVectorDeltaPosX_awesomeState();
	auto delPosY_aw = monitor.stl_getVectorDeltaPosY_awesomeState();
	drawPlot(delPosX_tr, delPosX_aw, vecTime, "delta(X)", "delta(X*)", "t, sec", "delta(X) and delta(X*), m");
	drawPlot(delPosY_tr, delPosY_aw, vecTime, "delta(Y)", "delta(Y*)", "t, sec", "delta(Y) and delta(Y*), m");

	auto delVelX_tr = monitor.stl_getVectorDeltaVelX_trueState();
	auto delVelY_tr = monitor.stl_getVectorDeltaVelY_trueState();
	auto delVelX_aw = monitor.stl_getVectorDeltaVelX_awesomeState();
	auto delVelY_aw = monitor.stl_getVectorDeltaVelY_awesomeState();
	drawPlot(delVelX_tr, delVelX_aw, vecTime, "delta(Vx)", "delta(Vx*)", "t, sec", "delta(Vx) and delta(Vx*), m/sec");
	drawPlot(delVelY_tr, delVelY_aw, vecTime, "delta(Vy)", "delta(Vy*)", "t, sec", "delta(Vy) and delta(Vy*), m/sec");


	///
	auto upDelPosX_aw = monitor.stl_getVectorUpperLimit_DeltaPosX_awesomeState();
	auto upDelPosY_aw = monitor.stl_getVectorUpperLimit_DeltaPosY_awesomeState();
	auto upDelVelX_aw = monitor.stl_getVectorUpperLimit_DeltaVelX_awesomeState();
	auto upDelVelY_aw = monitor.stl_getVectorUpperLimit_DeltaVelY_awesomeState();

	auto lowDelPosX_aw = monitor.stl_getVectorLowerLimit_DeltaPosX_awesomeState();
	auto lowDelPosY_aw = monitor.stl_getVectorLowerLimit_DeltaPosY_awesomeState();
	auto lowDelVelX_aw = monitor.stl_getVectorLowerLimit_DeltaVelX_awesomeState();
	auto lowDelVelY_aw = monitor.stl_getVectorLowerLimit_DeltaVelY_awesomeState();

	drawPlot(delPosX_tr - delPosX_aw, upDelPosX_aw, lowDelPosX_aw, vecTime, "delta(X) - delta(X*)", "t, sec", "delta(X) - delta(X*), m");
	drawPlot(delPosY_tr - delPosY_aw, upDelPosY_aw, lowDelPosY_aw, vecTime, "delta(Y) - delta(Y*)", "t, sec", "delta(Y) - delta(Y*), m");
	drawPlot(delVelX_tr - delVelX_aw, upDelVelX_aw, lowDelVelX_aw, vecTime, "delta(Vx) - delta(Vx*)", "t, sec", "delta(Vx) - delta(Vx*), m/sec");
	drawPlot(delVelY_tr - delVelY_aw, upDelVelY_aw, lowDelVelY_aw, vecTime, "delta(Vy) - delta(Vy*)", "t, sec", "delta(Vy) - delta(Vy*), m/sec");


	matplot::show();
}