#pragma once

#include "BialkowskiCollisionChecker.h"
#include "NavigationEnvironment.h"
#include <vector>
#include "SearchGrid.h"
#include "Graphics.h"
#include "Route.h"

class VisibilityGraph{
public:
	VisibilityGraph(NavigationEnvironment ne);
	~VisibilityGraph();
	void build(Node* qInit, Node* qGoal);
	vector<Node*> getStackNodes();
	Route* getRoute();
	Node* getRoot();
	Node* getGoal();
	NavigationEnvironment* getNavigationEnvironment();
private:
	void addToGraph(Node * newNode);
	void addEdgeToGraph(Node * newNode, Node * testNode);
	Node* nearestNode(Node* vertex);
	Route* lesserPathDijkstra();
	Node* extractMin(vector<Node*> nodes);
	void removeFromGraph(vector<Node*>& nodes, Node * currentNode);
	void displayNow();
	NavigationEnvironment navigationSpace;
	Node* root;
	Node* goal;
	vector<Node*> stackNodes;
	BialkowskiCollisionChecker collisionChecker;
	double planningTime = 0;
	Graphics* g;
	Route* route;
	int qtdNodes;
};