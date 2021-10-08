#pragma once
#include "RRT.h"
class RRTGrid :	public RRT{
public:
	RRTGrid(NavigationEnvironment ne, int const n);
	~RRTGrid();
	void build(Node* qInit, Node *qGoal, double distanceToGenerateRoute, string envName, int numberCells);
private:
	Node* createNewNodeBySulharevGrid(Node* qNear, Node*qRand);
};