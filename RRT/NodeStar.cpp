#include "NodeStar.h"
#include <iostream>
#include "MathUtils.h"

using namespace std;

NodeStar::NodeStar(){
}

NodeStar::NodeStar(Node * node){
	this->nodeWaypoint = new Waypoint{ node->getNodeWaypoint()->getLatitude(), node->getNodeWaypoint()->getLongitude()};
	this->antecessor = node->getAntecessor();
	this->sucessores = node->getSucessors();
	this->visited = node->isVisited();
	this->startPosition = node->isStartPosition();
}

NodeStar::NodeStar(double latitude, double longitude):Node(latitude, longitude){
}

NodeStar::~NodeStar(){

}

double NodeStar::costTo(NodeStar * nodeToCost) {
	//Inicia a busca do n� que � o pr�prio objeto
	Node* currentNode = this;
	vector<Node*> sucessorsNode;
	vector<int> stackSucessor , stackAntecessor;
	vector<Node*>::const_iterator sucessorNodeIt;
	int indiceInit = 0;
	double totalCost = 0;

	while (true) {
		Node* candidateNode;
		sucessorsNode = currentNode->getSucessors();

		//Se nulo significa que n�o existem sucessores
		if (sucessorsNode.empty()) {
			short int indice = stackSucessor.back();			
			indice++;
			stackSucessor.pop_back();
			totalCost -= ((NodeStar*)currentNode)->getCost();

			candidateNode = currentNode->getAntecessor()->getSucessors().at(indice);
			
			////Se n�o h� mais sucessores para este n�, ent�o voltemos com o n� antecessor
			if (candidateNode == NULL) {
				////Se vazio ent�o devemos olhar os sucessores do antecessor
				if (stackSucessor.empty()) {
					//Caso em que se alcan�a a raiz da �rvore.
					if (currentNode->getAntecessor()->getAntecessor() == NULL) {
						candidateNode = currentNode->getAntecessor();
					}else {
						//TODO: Passando um n� sucessor e o seu antecessor, o getIdSucessorNode
						//ir� retornar um indice relacionado aos outros sucessores.
						short int indiceAntecessor = getIdSucessorNode(currentNode->getAntecessor(),
													 currentNode->getAntecessor()->getAntecessor());
						candidateNode = currentNode->getAntecessor()->getAntecessor();
						stackAntecessor.push_back(indiceAntecessor);		
					}
					totalCost += ((NodeStar*)currentNode->getAntecessor())->getCost();
					currentNode = candidateNode;

					//Evitar a situa��o em que na decida de n�vel, quando este n�
					//corrente for um sucessor de seu n� origem e ele seja o n� procurado,
					//ele n�o seja contabilizado duas vezes.
					if (currentNode == nodeToCost) {
						return totalCost;
					}					
					continue;
				//Volta um n�vel na �rvore
				} else {
					indice = stackSucessor.back();
					indice++;
					stackSucessor.pop_back();

					totalCost -= ((NodeStar*)currentNode->getAntecessor())->getCost();
					candidateNode = currentNode->getAntecessor()->getAntecessor()->getSucessors().at(indice);

					if (!stackAntecessor.empty()) {
						if (candidateNode == currentNode->getAntecessor()->getSucessors().at(stackAntecessor.back())) {
							candidateNode = currentNode->getAntecessor()->getSucessors().at(++indice);
							stackAntecessor.pop_back();
						}
					}
				}				
			}					

			currentNode = candidateNode;
			stackSucessor.push_back(indice);
			totalCost += ((NodeStar*)currentNode)->getCost();
			continue;
		}else {
			//Para todos os sucessores do n� corrente verificar se � o no procurado
			for (sucessorNodeIt = sucessorsNode.begin(); sucessorNodeIt < sucessorsNode.end(); sucessorNodeIt++) {
				Node* currentSucessor = *sucessorNodeIt;
				if (currentSucessor == nodeToCost) {
					totalCost += ((NodeStar*)currentSucessor)->getCost();
					return totalCost;
				}
			}
		}

		if (!stackAntecessor.empty() && stackAntecessor.back() == indiceInit) {
			stackAntecessor.pop_back();

			candidateNode = sucessorsNode.at(indiceInit + 1);

			if (candidateNode == NULL) {
				totalCost += ((NodeStar*)currentNode)->getCost();
				currentNode = currentNode->getAntecessor();
				stackAntecessor.push_back(indiceInit);

				if (currentNode == nodeToCost) {
					return totalCost;
				}

				continue;
			}

			currentNode = candidateNode;
			stackSucessor.push_back(indiceInit + 1);
			totalCost += ((NodeStar*)currentNode)->getCost();
		} else {
			currentNode = sucessorsNode.at(indiceInit);
			stackSucessor.push_back(indiceInit);
			totalCost += ((NodeStar*)currentNode)->getCost();
		}			
	}
}

short int NodeStar::getIdSucessorNode(Node* node, Node* antecessorNode){
	vector<Node*>::const_iterator sucessoresToCompareIt;
	vector<Node*> sucessoresToCompare = antecessorNode->getSucessors();

	for (sucessoresToCompareIt = sucessoresToCompare.begin(); sucessoresToCompareIt < sucessoresToCompare.end(); sucessoresToCompareIt++){
		Node* currentSucessor = *sucessoresToCompareIt;
		if(currentSucessor == node){
			return sucessoresToCompareIt - sucessoresToCompare.begin();
		}
	}
}

double NodeStar::setCost(double cost){
	return this->cost = cost;
}

//Retorna o custo da aresta formada entre o n� e seu antecessor.
//TODO: exclu�do porque o mapa de custos foi retirado
/*double NodeStar::nodeEdgeCost(Node* node) {
	auto mapPair = this->mapCost.find((*(NodeStar*)(node)));
	return mapPair->second;
}*/

double NodeStar::getCost(){
	/*if (this) {
		if (this->cost >= 0) {
			return this->cost;
		}
	}*/
	//return 0;

	double totalCost = 0;
	bool rootReached = false;
	NodeStar* node = this;

	while (node->getAntecessor() != nullptr) {
		totalCost += MathUtils::euclidianDistanceNode(node, node->getAntecessor());
		node = (NodeStar*)node->getAntecessor();
		if (node->startPosition) {
			rootReached = true;
		}
	}

	if ((totalCost == 0 || !rootReached) && !this->startPosition)
		totalCost = std::numeric_limits<double>::max();

	return totalCost;
}

void NodeStar::updateCost() {
	double totalCost = 0;
	NodeStar* node = this;
	while (node->getAntecessor() != nullptr) {
		totalCost += MathUtils::euclidianDistanceNode(node, node->getAntecessor());
		node = (NodeStar*)node->getAntecessor();
	}

	this->cost = totalCost;
}