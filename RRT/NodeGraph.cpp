#include "NodeGraph.h"

NodeGraph::NodeGraph(){
	this->weight = numeric_limits<double>::max();
}

NodeGraph::NodeGraph(double latitude, double longitude):Node(latitude, longitude){
	this->weight = numeric_limits<double>::max();
}

NodeGraph::NodeGraph(Node* node) : Node(node->getX(), node->getY()) {
	this->weight = numeric_limits<double>::max();
}

NodeGraph::~NodeGraph(){
}

double NodeGraph::getWeight(){
	return weight;
}

void NodeGraph::setWeight(double weight){
	this->weight = weight;
}
