#include "VisibilityGraph.h"
#include "NodeGraph.h"
#include "BialkowskiCollisionChecker.h"
#include "StaticObstacle.h"
#include "GraphDisplayer.h"
#include "EnvironmentDisplayer.h"
#include "CollisionCheckerDisplayer.h"
#include <vector>
#include "CGALDefinitions.h"

using namespace std;

VisibilityGraph::VisibilityGraph(NavigationEnvironment ne):navigationSpace(ne){
	this->qtdNodes = 0;
	this->stackNodes;
	this->collisionChecker = BialkowskiCollisionChecker{ &(this->navigationSpace) };
}

VisibilityGraph::~VisibilityGraph(){
}

void VisibilityGraph::build(Node* qInit, Node* qGoal){
	this->root = new NodeGraph{qInit};
	((NodeGraph*)this->root)->setWeight(0);
	//addToGraph(this->root);

	this->goal = new NodeGraph{ qGoal };

	//Conect each conver vertex with other
	vector<StaticObstacle*> obstacles = this->navigationSpace.getStaticObstacles();
	vector<StaticObstacle*>::iterator itObs = obstacles.begin();
	
	for (StaticObstacle* currentObst : obstacles) {
		vector<Point_2> obstacleVertices;
		if (currentObst->getOffSet()->size()!=0) {
			obstacleVertices = currentObst->getOffSetConvexVertices();
		} else {
			obstacleVertices = currentObst->getConvexVertices();
		}
		for (int k = 0; k < obstacleVertices.size(); k++) {
			NodeGraph* newNode = new NodeGraph{ obstacleVertices[k].x(), obstacleVertices[k].y() };
			addToGraph(newNode);
		}
	}

	//for (StaticObstacle* currentObst:obstacles) {
	//	VertexIterator itVertex = currentObst->getShape()->vertices_begin();
	//	for (itVertex; itVertex < currentObst->getShape()->vertices_end(); itVertex++) {
	//		//Comparação do vértice corrente com todos os outros vértices
	//		for (StaticObstacle* obs : obstacles) {
	//			VertexIterator itTestVertex = obs->getShape()->vertices_begin();
	//			for (itTestVertex; itTestVertex < obs->getShape()->vertices_end(); itTestVertex++) {
	//				if (*itTestVertex != *itVertex) {
	//					NodeGraph* testNode = new NodeGraph{ (*itTestVertex).x(),  (*itTestVertex).y()};
	//					if (collisionChecker.batchCollisionFreePath(newNode, testNode)){
	//						addEdgeToGraph(newNode, testNode);
	//					}
	//				}
	//			}
	//		}
	//	}
	//}

	for (Node* currentVertex : stackNodes) {		
		for (Node* testVertex : stackNodes) {
			if (testVertex != currentVertex) {
				if (collisionChecker.batchCollisionFreePath(currentVertex, testVertex)){
					addEdgeToGraph(currentVertex, testVertex);
				}
			}
		}
	}

	//Add qInit and qGoal to the Graph
	stackNodes.insert(stackNodes.begin(), this->root);
	//Node* nearestInit = nearestNode(this->root);
	for (Node* testVertex : stackNodes) {
		if (collisionChecker.batchCollisionFreePath(this->root, testVertex)) {
			addEdgeToGraph(this->root, testVertex);
		}
		
	}
	//addEdgeToGraph(this->root, nearestInit);

	//Node* nearestGoal = nearestNode(this->goal);
	addToGraph(this->goal);
	//addEdgeToGraph(nearestGoal, this->goal);
	for (Node* testVertex : stackNodes) {
		if (collisionChecker.batchCollisionFreePath(this->goal, testVertex)) {
			addEdgeToGraph(testVertex, this->goal);
		}
	}

	Route* r = lesserPathDijkstra();
	this->route = r;
	cout << "Custo da rota ótima: " << ((NodeGraph*)r->getGoal())->getWeight() << endl;

	displayNow();
}

void VisibilityGraph::addToGraph(Node* newNode){
	this->stackNodes.push_back(newNode);
}

void VisibilityGraph::addEdgeToGraph(Node* newNode, Node* adjacentCandidate) {
	/*bool edgeExist = false;

	for (int i = 0; i < this->stackNodes.size(); i++) {
		if (stackNodes[i] == newNode) {
			vector<Node*> adjacents = stackNodes[i]->getSucessors();
			for (Node* adj : adjacents) {
				if (adj == adjacentCandidate) {
					edgeExist = true;
				}
			}
		}

		if (stackNodes[i] == adjacentCandidate) {
			vector<Node*> adjacents = stackNodes[i]->getSucessors();
			for (Node* adj : adjacents) {
				if (adj == newNode) {
					edgeExist = true;
				}
			}
		}
	}

	if (!edgeExist) {
		newNode->addSucessor(adjacentCandidate);
	}*/
	newNode->addSucessor(adjacentCandidate);
}

Node* VisibilityGraph::nearestNode(Node* vertex) {
	Node* qNear = NULL;
	double distance, limitDistance;

	limitDistance = numeric_limits<double>::max();

	vector<Node*>::iterator currentNodeIt;
	for (currentNodeIt = this->stackNodes.begin(); currentNodeIt < this->stackNodes.end(); currentNodeIt++) {
		if (vertex != *currentNodeIt && collisionChecker.batchCollisionFreePath(vertex, *currentNodeIt)) {
			distance = MathUtils::euclidianDistanceNode(*currentNodeIt, vertex);
			if (distance < limitDistance) {
				limitDistance = distance;
				qNear = *currentNodeIt;
			}
		}
	}
	return qNear;
}

Route* VisibilityGraph::lesserPathDijkstra(){
	vector<Node*> graphNodes = this->stackNodes;
	while (graphNodes.size() != 0) {
		Node* currentNode = extractMin(graphNodes);
		removeFromGraph(graphNodes, currentNode);
		cout << "Current node: " << "   x: " << currentNode->getX() << "  y:" << currentNode->getY() << "  weight: " << ((NodeGraph*)currentNode)->getWeight() << endl;
		for (Node* adjNode : currentNode->getSucessors()) {
			double costEdge = MathUtils::euclidianDistanceNode(adjNode, currentNode);
			cout << "\t adj node before: " << "  x: " << adjNode->getX() << "  y:" << adjNode->getY() << "  weight: " << ((NodeGraph*)adjNode)->getWeight() << endl;
			if (((NodeGraph*)adjNode)->getWeight() > ((NodeGraph*)currentNode)->getWeight() + costEdge) {
				((NodeGraph*)adjNode)->setWeight(((NodeGraph*)currentNode)->getWeight() + costEdge);
				adjNode->setAntecessor(currentNode);
			}
			cout << "\t\t\t adj node after: " << "  x: " << adjNode->getX() << "  y:" << adjNode->getY() << "  weight: " << ((NodeGraph*)adjNode)->getWeight() << endl;
		}
	}
	/*double costGoal = MathUtils::euclidianDistanceNode(this->goal->getAntecessor(), this->goal);
	((NodeGraph*)this->goal)->setWeight() +*/
	Route* route = new Route{ this->root, this->goal};
	return route;
}

Node* VisibilityGraph::extractMin(vector<Node*> nodes){
	vector<Node*>::iterator itNodes = nodes.begin();
	double min = numeric_limits<double>::max();
	Node* nodeMin = *itNodes;
	for (itNodes; itNodes < nodes.end(); itNodes++) {
		double currentCost = ((NodeGraph*)(*itNodes))->getWeight();
		Node* adjNode = (*itNodes);
		cout << "\t weight acessado: " << "  x: " << adjNode->getX() << "  y:" << adjNode->getY() << "  weight: " << ((NodeGraph*)adjNode)->getWeight() << endl;
		if (currentCost < min) {
			min = currentCost;
			nodeMin = *itNodes;
		}
	}

	return nodeMin;
}

void VisibilityGraph::removeFromGraph(vector<Node*> &nodes, Node* currentNode) {
	vector<Node*>::iterator itNodes = nodes.begin();
	
	for (itNodes; itNodes < nodes.end(); itNodes++) {
		if ((*itNodes) == currentNode) {
			nodes.erase(itNodes);
			return;
		}		
	}
}

void VisibilityGraph::displayNow() {
	int argc = 0;
	char **argv = NULL;

	if (g == NULL) {
		g = new Graphics(false);
		////Cria o objeto para mostrar a RRT.
		GraphDisplayer* graphDisplayer = new GraphDisplayer{ this };
		//Cria o objeto para mostrar o Ambiente de navegação.
		EnvironmentDisplayer* edDisplayer = new EnvironmentDisplayer{ this->navigationSpace };
		//Cria o objeto para mostrar os certificados da checagem de colisão
		CollisionCheckerDisplayer* ccDisplayer = new CollisionCheckerDisplayer{ this->collisionChecker };

		//A Ordem de registro define quem será mostrado primeiro no layer do OpenGl
		g->registerDisplayable(edDisplayer);
		g->registerDisplayable(ccDisplayer);
		g->registerDisplayable(graphDisplayer);
	}

	//string fileImageName = pathFile + "_print_it_" + to_string(iteration) + ".bmp";
	string fileImageName = "";
	g->init(argc, argv, this->navigationSpace.getInitialCoordinate()->getNodeWaypoint(),
		this->navigationSpace.getFinalCoordinate()->getNodeWaypoint(), fileImageName);
	g->startGraphics();
}

vector<Node*> VisibilityGraph::getStackNodes() {
	return this->stackNodes;
}

Route* VisibilityGraph::getRoute() {
	return this->route;
}

Node* VisibilityGraph::getRoot() {
	return this->root;
}

Node* VisibilityGraph::getGoal() {
	return this->goal;
}


NavigationEnvironment* VisibilityGraph::getNavigationEnvironment() {
	return &this->navigationSpace;
}