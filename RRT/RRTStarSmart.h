#pragma once
#include "RRTStar.h"
#include <CGAL/point_generators_2.h>

class RRTStarSmart : public RRTStar {
public:
	RRTStarSmart(NavigationEnvironment ne, const double r,  int const n);
	~RRTStarSmart();
	void build(Node* qInit, Node *qGoal, double distanceToGenerateRoute, string envName);
	void build(NodeStar* qInit, NodeStar* qGoal, double distanceToGenerateRoute, string envName);
private:
	vector<NodeStar*> beacons;
	NodeStar* generateBiasedNode(int n);
	double optimizePath(NodeStar * qInit, NodeStar * qGoal);
	void updatePathNodesCost(NodeStar * qInit, NodeStar * qGoal);
	vector<NodeStar*> collectBeaconsFromPath(NodeStar* qInit, NodeStar* qGoal);
	CGAL::Random * randBeaconGenerator = nullptr;
	std::mt19937 selectBeaconGenerator;
};