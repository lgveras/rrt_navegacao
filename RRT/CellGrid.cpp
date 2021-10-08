#include "CellGrid.h"
#include "MathUtils.h"

CellGrid::CellGrid(){
	this->nodes = std::vector<Node*>();
}

CellGrid::~CellGrid(){
}

void CellGrid::insert(Node * node){
	this->nodes.push_back(node);
}

Node* CellGrid::searchNearest(Node * node){
	Node* nearestNode = nullptr;
	double distance;
	double lesserDistance = numeric_limits<double>::max();

	for (int i = 0; i < this->nodes.size(); i++) {
		distance = MathUtils::euclidianDistanceNode(this->nodes[i], node);
		if (distance < lesserDistance) {
			lesserDistance = distance;
			nearestNode = this->nodes[i];
		}
	}
	return nearestNode;
}

vector<Node*> CellGrid::getNodes() {
	return this->nodes;
}