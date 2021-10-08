#pragma once
#include <vector>
#include "Node.h"

class CellGrid{
public:
	CellGrid();
	~CellGrid();
	void insert(Node* node);
	Node* searchNearest(Node* node);
	vector<Node*> getNodes();
private:
	vector<Node*> nodes;
};

