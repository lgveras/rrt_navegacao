#pragma once
#include "InformedRRTStar.h"

typedef pair<double, NodeStar*> Vertex;
typedef pair<double, pair<NodeStar*, NodeStar*>> Edge;

typedef vector<pair<double, NodeStar*>>::iterator VertexIt;
typedef vector<pair<double, pair<NodeStar*, NodeStar*>>>::iterator EdgeIt;

class BITStar :	public InformedRRTStar{
public:
	BITStar(NavigationEnvironment ne, const double r, int const n);
	BITStar(NavigationEnvironment ne, const double r, int const n, string experimentName);

	~BITStar();
	void build(NodeStar* qInit, NodeStar *qGoal, double distanceToGenerateRoute, string envNames);
	void prune(double goalCost);
	//void insertIntoQueue(vector<pair<double, NodeStar*>> queueVertices, vector<NodeStar*> stackNodes);
	void expandVertex(NodeStar* v);
	double calcRealCostEdge(Edge bestEdge);
	double costToVertice(NodeStar* v);
	double costFromVertice(NodeStar* v);
	double calcCostEdge(NodeStar * v, NodeStar * x);
	double calcCostEdge(pair<NodeStar*, NodeStar*> edge);
	vector<NodeStar*> generateBatch(double batchValue);
	double calcRadius(int samplesQtd);
	void sortVerticesQueue(vector<Node*> samplesRGG);
	double bestQueueValue(vector<pair<double, NodeStar*>> &queue);
	double bestQueueValue(vector <pair<double, pair<NodeStar*, NodeStar*>>> &queue);
	NodeStar* bestInQueue(vector<Vertex> &queueVertices);
	Edge bestInQueue(vector<Edge> &queueEdges);
	bool removeFromQueue(NodeStar * v);
	bool removeFromQueue(pair<NodeStar*, NodeStar*> edge);
	vector<NodeStar*> collectNear(vector<NodeStar*> samplesRGG, NodeStar * v, double radius) const;
	bool isOnOldVertices(NodeStar* v);
	bool isInEdgeQueue(NodeStar * v, NodeStar * w);
	bool isOnVertices(NodeStar* v);
	bool removeFromSamplesRGG(NodeStar* v);
	double lebesgueMeasure();
	void insertIntoVerticesQueue(NodeStar* v);
	void insertIntoEdgesQueue(NodeStar* v, NodeStar* x);
	vector<NodeStar*> getSamplesRGG();
	string experiment;
private:
	double bestVertexIndex = 0;
	double bestEdgeIndex = 0;
	//Linha 2
	vector<NodeStar*> samplesRGG;
	//vector<pair<NodeStar*, NodeStar*>> queueEdges;
	vector<Edge> queueEdges;
	pair<NodeStar*, NodeStar*> bestEdgeInQueue;
	//vector<NodeStar*> queueVertices;
	vector<pair<double, NodeStar*>> queueVertices;
	NodeStar* bestVerticeInQueue = nullptr;
	double radiusRGG = numeric_limits<double>::max();
	vector<Node*> nodesOld;
	vector<NodeStar*> samples; //Xsamples
};