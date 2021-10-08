#pragma once
#include "Displayable.h"
#include "VisibilityGraph.h"
#include "Node.h"

class GraphDisplayer :	public Displayable{
public:
	GraphDisplayer(VisibilityGraph* graph);
	~GraphDisplayer();
	void display(Graphics* graphic);
private:
	Graphics* graphics;
	VisibilityGraph* graph;
	int countPrintedPoints;
	void displayRoute(Node* qGoal, float lineColor[3], float pointColor[3]);
	void displayGraph(vector<Node*> nodes);
};

