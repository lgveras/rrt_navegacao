#pragma once
#include "RRTStar.h"
#include "NavigationEnvironment.h"
#include "KDTree.h"

class RRTStarSukharevVertices :	public RRTStar{
public:
	RRTStarSukharevVertices(NavigationEnvironment ne, const double r, int const n);
	RRTStarSukharevVertices(NavigationEnvironment ne, const double r, int const n, string experimentName);
	~RRTStarSukharevVertices();
	NodeStar* generateBiasedNode();
	void build(Node* qInit, Node *qGoal, double distanceToGenerateRoute, string envName, int cellsSukharev);
	void buildSukharev(Node* qInit, Node *qGoal, double distanceToGenerateRoute, string envName, int cellsSukharev);
	void buildVerticesKdTree(Node* qInit, Node * qGoal, double distanceToGenerateRoute, string envName, string dispersionName);
	void buildSukharevVertices(Node* qInit, Node *qGoal, double distanceToGenerateRoute, string envName, int cellsSukharev);
	void buildSukharevVerticesKdTreeNoOpt(Node * qInit, Node * qGoal, double distanceToGenerateRoute, string envName, int cellsSukharev);
	void buildSukharevVerticesHough(Node * qInit, Node * qGoal, double distanceToGenerateRoute, string envName, int cellsSukharev);
	void buildSukharevVerticesProportion(Node * qInit, Node * qGoal, double distanceToGenerateRoute, string envName, int cellsSukharev);
	void buildSukharevVerticesKdTree(Node * qInit, Node * qGoal, double distanceToGenerateRoute, string envName, int cellsSukharev);
	KDTree * createKdTreeToPointSet(const int size, vector<Point_2> points);
	vector<Point_2> getConvexHull();
	vector<vector<Point_2>> getObstacleConvexHulls();
	int findByProximity(Point_2 & v, vector<pair<double, Point_2*>>& vtable, double deltaX, double deltaY);
	int localSearch(Point_2 * p, vector<pair<double, Point_2*>> vtable, int neighborhoodSize, int pivot);
protected:
	vector<Point_2> addedVertices;
	NodeStar* createNewNodeBySukharevGrid(Node* qNear, Node*qRand);
	double optimizePath(NodeStar * qInit, NodeStar * qGoal);
	void updatePathNodesCost(NodeStar * qInit, NodeStar * qGoal);
	NodeStar * searchInStackNodes(NodeStar * nodeToSearch);
	vector<pair<int, Point_2*>> getConvexVerticesOrderedList(NodeStar* qNear);
	Point_2 findNearestConvexVertice(NodeStar* qNear);
	Point_2 findNearestConvexVertice(NodeStar* qNear, vector<Point_2>* testedVertices);
	bool alreadyAddedVertice(Point_2 vertice);
	bool collisionCheckedVertice(vector<Point_2>* testedVertices, Point_2 vertice);
	vector<NodeStar*> getNearestFreeCollisionVertices(NodeStar* node);
	bool isInList(vector<NodeStar*> list, NodeStar* node);
	vector<Point_2> convexHull;
	vector<vector<Point_2>> obstacleConvexHulls;
	string experiment;
};