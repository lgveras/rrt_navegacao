#pragma once
#include "RRT.h"

class RRTCurve : public RRT {
public:
	RRTCurve(NavigationEnvironment ne, int const n);
	~RRTCurve();
	void build(Node* qInit, Node *qGoal, double distanceToGenerateRoute, string envName);
	G2PlanarPH* smoothSegmentPair(Node* nodeRef_i, Node* nodeRef_c, Node* nodeRef_o, double L);
	void insertInSmoothedTree(Node* nodeRef_i, Node* nodeRef_c, G2PlanarPH* curve, Node* nodeRef_o, double L);
	vector<Node*> getStackSmoothNodes();
	Node* getSmoothRoot();
	Node* getSmoothGoal();
private:
	vector<Node*> stackSmoothNode;
	Node* smoothRoot;
	Node* smoothGoal;
};

