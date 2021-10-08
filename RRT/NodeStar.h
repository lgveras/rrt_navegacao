#pragma once
#include "Node.h"
#include <map>

using namespace std;

//Usei para construir um mapa de custos. Mas depois vi que não era necessário
//TODO: ver o que fazer com esses código
//struct compareNode {
//	bool operator() (const Node& nodeCompare, const Node& nodeSearched) const {
//		return nodeCompare.getX() == nodeSearched.getX() && nodeCompare.getY() == nodeSearched.getY();
//	}
//};

class NodeStar : public Node{
public:
	NodeStar();
	NodeStar(Node* node);
	NodeStar(double latitude, double longitude);
	~NodeStar();
	double costTo(NodeStar* nodeToCost);
	short int getIdSucessorNode(Node* node, Node* antecessorNode);
	double getCost();
	double setCost(double cost);
	void updateCost();
	void insertNode(NodeStar* qParent, NodeStar* qChild);
	bool operator ==(const NodeStar &node2){
		return (node2.getX() == this->getX() && node2.getY() == this->getY());
	}
private:
	//Custo da aresta de um nó em relação a seu antecessor
	double cost = 0;
};