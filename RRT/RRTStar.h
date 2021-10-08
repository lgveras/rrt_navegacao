#ifndef RRTSTAR_H
#define RRTSTAR_H

#include "NavigationEnvironment.h"
#include "RRT.h"
#include "NodeStar.h"

class RRTStar : public RRT {
public:
	RRTStar(NavigationEnvironment ne, const double r, int const n);
    virtual ~RRTStar();
    virtual void build(Node* qInit, Node *qGoal, double distanceToGenerateRoute, string envName, string dispersionName);
	void build(NodeStar* qInit, NodeStar *qGoal, double distanceToGenerateRoute, string envName, string dispersionName);
	double getRadius();
	Node* getCurrentNew();
protected:
	NodeStar* createNewNodeStar(Node * qNear, Node * qRand);
	NodeStar* chooseParent(vector<NodeStar*> nodeNeighborhood, NodeStar* qNear, NodeStar* qNew);
	void extend(Node* qNear, NodeStar* qNew);
	const double radiusNeighborhood;
	using RRT::insertNode;
	void insertNode(NodeStar* qParent, NodeStar* qChild);
	void rewire(vector<NodeStar*> nodeNeighborhood, NodeStar* qMin, NodeStar* qNew);
	vector<NodeStar*> getNeighborsInRadius(NodeStar * node, double radius);
	vector<NodeStar*> getNeighborsInRadius(NodeStar * node);
	vector<NodeStar*> copyRoute(NodeStar* qInit, NodeStar* qGoal);
};

#endif /* RRTSTAR_H */