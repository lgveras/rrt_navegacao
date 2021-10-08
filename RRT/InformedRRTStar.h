#pragma once
#include "RRTStar.h"
#include <Eigen/Core>
// Inversion and determinants
#include <Eigen/LU>
// SVD decomposition
#include <Eigen/SVD>
#include <iostream>

class InformedRRTStar :	public RRTStar{
public:
	InformedRRTStar(NavigationEnvironment ne, const double r, int const n);
	InformedRRTStar(NavigationEnvironment ne, const double r, int const n, string experimentName);
	~InformedRRTStar();
	void build(NodeStar* qInit, NodeStar *qGoal, double distanceToGenerateRoute, string envName);
protected:
	NodeStar* informedRRTSampling();
	void updateTransformingData(double currentCost);
	void configElipseData(NodeStar* qInit, NodeStar* qGoal);
private:
	double minTransverseDiameter;
	Eigen::Matrix2d rotationMatrix;
	Eigen::Matrix2d transformationMatrix;
	Eigen::Vector2d center;
	double bestCost;
	Eigen::Matrix2d calcRotationToWorldFrame(Eigen::Vector2d focus1, Eigen::Vector2d focus2);
	Eigen::Matrix2d calcTransformation(double currentCost);
	Point_2 circle2Elipse(double x, double y);
	Point_2 getRandomPointOnDisc();
	CGAL::Random * randBeaconGenerator = nullptr;
	string experiment;
};
