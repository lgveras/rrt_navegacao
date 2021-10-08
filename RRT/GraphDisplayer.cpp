#include "GraphDisplayer.h"
#include <G2PlanarPH.h>

GraphDisplayer::GraphDisplayer(VisibilityGraph* graph):graph(graph){
}

GraphDisplayer::~GraphDisplayer(){
}

void GraphDisplayer::displayGraph(vector<Node*> nodes) {
	float lineColor[] = { 0.8, 0.3, 0.4};
	float pointColor[] = { 1,0.8,0.1 };
	float initColor[] = { 0,1,0 };
	float goalColor[] = { 1, 0, 0 };

	Node* current = this->graph->getNavigationEnvironment()->getDispersionGrid().getCentroidByIndex(0);
	int index = 1;
	float color[3] = { 0.5 , 0.2 , 0 };
	while (current) {
		this->graphics->drawPoint(current->getX(), current->getY(), color, 2);
		current = this->graph->getNavigationEnvironment()->getDispersionGrid().getCentroidByIndex(index);
		index++;
	}

	//imprime ele e seu antecessor e conecta com uma linha.
	//se tem com quem formar uma linha, liga-os
	for (int i = 0; i < nodes.size(); i++) {
		float xPlot = nodes[i]->getX();
		float yPlot = nodes[i]->getY();
		//std::cout << "Ponto lat = " << xPlot << "; long = " << yPlot << " lido." << endl;
		countPrintedPoints = countPrintedPoints + 1;
		//std::cout << "Total de pontos até o momento = " << countPrintedPoints << endl;
		vector<Node*> sucessores = nodes[i]->getSucessors();
		if (sucessores.size() != 0) {
			vector<Node*>::iterator it;
			for (it = sucessores.begin(); it < sucessores.end(); it++) {
				Node* sucessorNode = *it;
				float xAntPlot = sucessorNode->getX();
				float yAntPlot = sucessorNode->getY();
				this->graphics->drawLine(xPlot, yPlot, xAntPlot, yAntPlot, lineColor, 2);
			}
		}
		this->graphics->drawPoint(nodes[i]->getX(), nodes[i]->getY(), pointColor, 6);
	}

	//Draw init point, goal point, and region distance to connect the tree of the goal
	this->graphics->drawPoint(this->graph->getRoot()->getX(), this->graph->getRoot()->getY(), initColor, 20);
	this->graphics->drawPoint(this->graph->getGoal()->getX(), this->graph->getGoal()->getY(), goalColor, 20);
}


void GraphDisplayer::displayRoute(Node* qGoal, float lineColor[3], float pointColor[3]) {
	float curveColor[] = { 0.2, 0.5, 1 };
	float initColor[] = { 0,1,0 };
	float goalColor[] = { 1, 0, 0 };

	Node* currentNode = qGoal;
	if (!qGoal->getAntecessor()) {
		return;
	}

	while (true) {
		if (currentNode->getAntecessor() == nullptr) {
			this->graphics->drawPoint(currentNode->getX(), currentNode->getY(), initColor, 20);
			break;
		}

		Node* toReprintOnCurve = nullptr;

		Node* updateNode = nullptr;

		if (currentNode->getCurve()) {
			G2PlanarPH* curve = currentNode->getCurve();
			double deltaT = 0.01;
			double a1, a2, b1, b2;
			bool printLine = false;

			//Print the smoothed path
			for (double t = 0; t <= 1; t += deltaT) {
				CPointPH ptTemp = CPointPH(real(G2beval(Dgr, curve->spline[1].p, t)), imag(G2beval(Dgr, curve->spline[1].p, t)));
				if (printLine) {
					double a1 = ptTemp.xPos;
					double a2 = ptTemp.yPos;
					this->graphics->drawLine(a1, a2, b1, b2, curveColor, 5);
				}

				/*if (t == 1) {
				Node* node = new Node(ptTemp.xPos, ptTemp.yPos);
				this->graphics->drawPoint(node->getX(), node->getY(), pointColor, 6);
				}*/

				if (t == 0) {
					toReprintOnCurve = new Node(ptTemp.xPos, ptTemp.yPos);
				}

				b1 = ptTemp.xPos;
				b2 = ptTemp.yPos;
				printLine = true;
			}

			//currentNode = currentNode->getAntecessor()->getAntecessor();
			updateNode = currentNode->getAntecessor()->getAntecessor();
		} /*else if(!currentNode->getAntecessor()->getCurve()) {
		  this->graphics->drawLine(currentNode->getX(), currentNode->getY(),
		  currentNode->getAntecessor()->getX(), currentNode->getAntecessor()->getY(), lineColor, 3);
		  }*/else {
			this->graphics->drawLine(currentNode->getX(), currentNode->getY(),
				currentNode->getAntecessor()->getX(), currentNode->getAntecessor()->getY(), lineColor, 5);
			//currentNode = currentNode->getAntecessor();
			updateNode = currentNode->getAntecessor();
		}

		if (currentNode == qGoal) {
			this->graphics->drawPoint(currentNode->getX(), currentNode->getY(), goalColor, 20);
		}
		else {
			this->graphics->drawPoint(currentNode->getX(), currentNode->getY(), pointColor, 10);
			/*if (toReprintOnCurve) {
			this->graphics->drawPoint(toReprintOnCurve->getX(), toReprintOnCurve->getY(), pointColor, 6);
			}*/
		};

		currentNode = updateNode;
	}
}

void GraphDisplayer::display(Graphics* graphic) {
	this->graphics = graphic;

	this->displayGraph(this->graph->getStackNodes());
	
	Route* route = this->graph->getRoute();

	float lineColorRoute[] = { 1,0,0 };
	float pointColorRoute[] = { 0.6, 0 ,0.8 };

	if (route) {	
		this->displayRoute(route->getGoal(), lineColorRoute, pointColorRoute);
	}
}