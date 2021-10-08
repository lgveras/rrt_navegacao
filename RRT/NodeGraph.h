#pragma once
#include "Node.h"
class NodeGraph : public Node{
public:
	NodeGraph();
	NodeGraph(double latitude, double longitude);
	NodeGraph(Node * node);
	~NodeGraph();
	double getWeight();
	void setWeight(double weight);
private:
	double weight;
};

