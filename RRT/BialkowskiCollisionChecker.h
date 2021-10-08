#pragma once

#include"Node.h"
#include "MathUtils.h"
#include "CollisionCheckerSample.h"
#include "NavigationEnvironment.h"
#include <map>
#include "NodePointerCompare.h"
using namespace std;

class BialkowskiCollisionChecker{
public:
	BialkowskiCollisionChecker();
	BialkowskiCollisionChecker(NavigationEnvironment* ne);
	~BialkowskiCollisionChecker();
	bool collisionFreePoint(Node* sample);
	void insertInColisionFreePointSet(Node * node, double distance);
	bool batchCollisionFreePath(Node* node, Node * sample);
	bool batchCollisionFreePath(vector<Node*> nodes, Node* sample);
	Node* nearestNodeInSet(map<Node*, double, NodePointerCompare> set, Node* sample);
	double distanceToObstacles(Node* sample);
	double distanceToFreeSpace(Node* sample);
	double calcMinimumDistanceToObstacles(Node * sample);
	bool testFreeSegment(Node* initNode, Node* endNode);
	int getCollisionsDetected();
	bool insideObstacle(Node* sample);
	map<Node*, double, NodePointerCompare>  getCollisionFreeSet();
	map<Node*, double, NodePointerCompare>  getCollisionObsSet();
private:
	map<Node*, double, NodePointerCompare> collisionFree;
	map<Node*, double, NodePointerCompare> collisionObs;
	NavigationEnvironment* ne;
	int collisionsDetected = 0;
};