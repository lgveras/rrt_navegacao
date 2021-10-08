/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RRTDisplayer.cpp
 * Author: LuizGustavo
 * 
 * Created on 30 de Agosto de 2016, 10:05
 */

#include "RRTDisplayer.h"
#include "Graphics.h"
#include "RRT.h"
#include "RRTStarSukharevVertices.h"
#include "RRTStarSmart.h"
#include "RRTCurve.h"
#include "BITStar.h"
#include "Node.h"
#include "Waypoint.h"
#include "SearchGrid.h"
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include "G2PlanarPH.h"

RRTDisplayer::RRTDisplayer(RRT* rrt){
	this->rrt = rrt;
}

RRTDisplayer::RRTDisplayer(const RRTDisplayer& orig) {
}

RRTDisplayer::~RRTDisplayer() {
}

RRT* RRTDisplayer::getRRT(){
    return this->rrt;
}

void RRTDisplayer::displayRRT(vector<Node*> nodes){
	float lineColor[] = { 0.1,0.98,0 };
	//float lineColor[] = { 0.973, 0.023, 0.805 };

	float pointColor[] = { 0.95,0.6,0.05 };
	float initColor[] = { 0,1,0 };
	float goalColor[] = { 1, 0, 0 };	

	Node* current = this->rrt->getNavigationEnvironment()->getDispersionGrid().getCentroidByIndex(0);
	int index = 1;
	float color[3] = { 0.5 , 0.2 , 0 };
	while (current) {
		this->graphics->drawPoint(current->getX(), current->getY(), color, 2);
		current = this->rrt->getNavigationEnvironment()->getDispersionGrid().getCentroidByIndex(index);
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
		this->graphics->drawPoint(nodes[i]->getX(), nodes[i]->getY(), pointColor, 10);
	}

	//Draw init point, goal point, and region distance to connect the tree of the goal
	this->graphics->drawPoint(this->rrt->getRoot()->getX(), this->rrt->getRoot()->getY(), initColor, 20);
	this->graphics->drawPoint(this->rrt->getGoal()->getX(), this->rrt->getGoal()->getY(), goalColor, 20);
	//this->graphics->drawUnfilledCircle(this->rrt->getGoal()->getX(), this->rrt->getGoal()->getY(), this->rrt->getDistanceToGoal(), goalColor, 2);
	RRTStar* rrtStar = dynamic_cast<RRTStar*> (this->getRRT());

	if (rrtStar) {
	//if (nullptr) {
		Node* qNew = rrtStar->getCurrentNew();
	/*	double x = qNew.getX();
		double y = qNew.getY();*/
		if (qNew) {
			double radius = rrtStar->getRadius();
			float color[] = { 0.5, 0.2, 0.8 };
			this->graphics->drawUnfilledCircle(qNew->getX(), qNew->getY(), radius, color, 5);
		}		
	}
}

void RRTDisplayer::displaySmoothRRT(vector<Node*> nodes){
	float lineColor[] = { 0,0.8,1 };
	float pointColor[] = { 1,1,0 };
	float initColor[] = { 0,1,0 };
	float goalColor[] = { 1,1,0 };

	int index = 1;
	float color[3] = { 0.5 , 0.2 , 0 };

	int count = 0;
	//imprime ele e seu antecessor e conecta com uma linha.
	//se tem com quem formar uma linha, liga-os
	for (int i = 0; i < nodes.size(); i++) {
		if (nodes[i]->getAntecessor() != nullptr) {
			//if (nodes[i]->getCurve()) {
			//	Node* nodeP_c = nodes[i]->getSucessors()[0];
			//	//desenha curva para todos os sucessores
			//	drawCurveForSucessors(nodes[i], nodeP_c, nodeP_c->getSucessors());
			//} else {
			//	if (!nodes[i]->getAntecessor()->getCurve()){
			//		float startX = nodes[i]->getX();
			//		float startY = nodes[i]->getY();
			//		float endX = nodes[i]->getSucessors()[0]->getX();
			//		float endY = nodes[i]->getSucessors()[0]->getY();
			//		this->graphics->drawLine(startX, startY, endX, endY, lineColor, 1);
			//	} else {
			//		this->graphics->drawPoint(nodes[i]->getX(), nodes[i]->getY(), pointColor, 2);
			//	}
			//}
			if (nodes[i]->getSucessors().size() > 0) {
				if (nodes[i]->getSucessors()[0]->getCurve()) {
					displayCurveForSucessors(nodes[i]->getSucessors());
					continue; // Will not print all the nodes that are p_c.
				} else if (!nodes[i]->getCurve()) {
					float startX = nodes[i]->getX(); float startY = nodes[i]->getY();
					float endX = nodes[i]->getAntecessor()->getX(); float endY = nodes[i]->getAntecessor()->getY();
					this->graphics->drawLine(startX, startY, endX, endY, lineColor, 1);
				}
			} else {
				float startX = nodes[i]->getX(); float startY = nodes[i]->getY();
				float endX = nodes[i]->getAntecessor()->getX(); float endY = nodes[i]->getAntecessor()->getY();
				this->graphics->drawLine(startX, startY, endX, endY, lineColor, 1);
			}
		}
		this->graphics->drawPoint(nodes[i]->getX(), nodes[i]->getY(), pointColor, 2);
		count++;
	}

	cout << "counted points: " << count <<endl;

	//Draw init point, goal point, and region distance to connect the tree of the goal
	this->graphics->drawPoint(this->rrt->getRoot()->getX(), this->rrt->getRoot()->getY(), initColor, 20);
	this->graphics->drawPoint(this->rrt->getGoal()->getX(), this->rrt->getGoal()->getY(), goalColor, 20);
	this->graphics->drawUnfilledCircle(this->rrt->getGoal()->getX(), this->rrt->getGoal()->getY(), this->rrt->getDistanceToGoal(), goalColor, 2);
}

void RRTDisplayer::displayCurveForSucessors(vector<Node*> listNodesP_o) {
	vector<Node*>::iterator it;
	for (it = listNodesP_o.begin(); it < listNodesP_o.end(); it++) {
		Node* currentNodeP_o = *it;
		displayPHCurve(currentNodeP_o->getCurve());
	}
}

void RRTDisplayer::displayPHCurve(G2PlanarPH* curve) {
	double deltaT = 0.01;
	double a1, a2, b1, b2;
	bool printLine = false;
	float curveColor[] = { 0.2, 0.5, 1 };
	//{ 0.85, 0.6, 0};

	for (double t = 0; t <= 1; t += deltaT) {
		CPointPH ptTemp = CPointPH(real(G2beval(Dgr, curve->spline[1].p, t)), imag(G2beval(Dgr, curve->spline[1].p, t)));

		if (printLine) {
			double a1 = ptTemp.xPos;
			double a2 = ptTemp.yPos;
			this->graphics->drawLine(a1, a2, b1, b2, curveColor, 5);
		}

		b1 = ptTemp.xPos;
		b2 = ptTemp.yPos;
		printLine = true;
	}
}

void RRTDisplayer::displayRRT(SearchGrid* nodes){
	float lineColor[] = { 0,0.3,1 };
	float pointColor[] = { 1,1,0};

	vector<CellGrid*> nodeCells = nodes->getCells();
	//imprime ele e seu antecessor e concecta com uma linha.
    //se tem com quem formar uma linha, liga-os
	for (int j = 0; j < nodeCells.size(); j++) {
		vector<Node*> nodes = nodeCells[j]->getNodes();
		for (int i = 0; i < nodes.size(); i++) {
			float xPlot = nodes[i]->getX();
			float yPlot = nodes[i]->getY();
			/*std::cout << "Ponto lat = " << xPlot << "; long = " << yPlot << " lido." << endl;
			countPrintedPoints = countPrintedPoints + 1;
			std::cout << "Total de pontos até o momento = " << countPrintedPoints << endl;*/
			vector<Node*> sucessores = nodes[i]->getSucessors();
			if (sucessores.size() != 0) {
				vector<Node*>::iterator it;
				for (it = sucessores.begin(); it < sucessores.end(); it++) {
					Node* sucessorNode = *it;
					float xAntPlot = sucessorNode->getX();
					float yAntPlot = sucessorNode->getY();
					this->graphics->drawLine(xPlot, yPlot, xAntPlot, yAntPlot, lineColor, 1);
				}
			}
			this->graphics->drawPoint(nodes[i]->getX(), nodes[i]->getY(), pointColor, 2);
		}
	}
}

void RRTDisplayer::displayRoute(Node* qGoal, float lineColor[3], float pointColor[3]) {
	//float initColor[] = {0,1,0};
	//float goalColor[] = {1,1,0};
	float curveColor[] = { 0.2, 0.5, 1};
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
		}*/else{
			this->graphics->drawLine(currentNode->getX(), currentNode->getY(),
				currentNode->getAntecessor()->getX(), currentNode->getAntecessor()->getY(), lineColor, 5);
			//currentNode = currentNode->getAntecessor();
			updateNode = currentNode->getAntecessor();
		}

		if (currentNode == qGoal) {
			this->graphics->drawPoint(currentNode->getX(), currentNode->getY(), goalColor, 20);
		} else {
			this->graphics->drawPoint(currentNode->getX(), currentNode->getY(), pointColor, 15);
			/*if (toReprintOnCurve) {
				this->graphics->drawPoint(toReprintOnCurve->getX(), toReprintOnCurve->getY(), pointColor, 6);
			}*/
		};

		currentNode = updateNode;
	}
}

void RRTDisplayer::displaySmoothRoute(Node* qGoal, float lineColor[3], float pointColor[3]) {
	//float initColor[] = { 0,1,0 };
	//float goalColor[] = { 1,1,0 };
	float curveColor[] = { 0.2, 0.5, 1 };
	float initColor[] = { 0,1,0 };
	float goalColor[] = { 1, 0, 0 };

	Node* currentNode;
	for (int i = 0; i < 2; i++) {
		currentNode = qGoal;
		Node* lastDrown = qGoal;

		while (currentNode->getAntecessor() != nullptr) {
			if (i == 1) {
				if (!currentNode->getCurve()) {
					this->graphics->drawPoint(currentNode->getX(), currentNode->getY(), pointColor, 10);
				}
			} else if (i == 0) {
				if (currentNode->getAntecessor() != nullptr && !currentNode->getCurve() && !currentNode->getAntecessor()->getCurve()) {
					this->graphics->drawLine(currentNode->getX(), currentNode->getY(),
											 currentNode->getAntecessor()->getX(), currentNode->getAntecessor()->getY(), lineColor, 5);
				}

				if (currentNode->getCurve()) {
					displayPHCurve(currentNode->getCurve());
				}
			}
			//lastDrown = currentNode;
			currentNode = currentNode->getAntecessor();
		}
	}
	//this->graphics->drawLine(currentNode->getX(), currentNode->getY(), lastDrown->getX(), lastDrown->getY(), lineColor, 5);

	this->graphics->drawPoint(qGoal->getX(), qGoal->getY(), goalColor, 20);
	this->graphics->drawPoint(currentNode->getX(), currentNode->getY(), initColor, 20);
}

void RRTDisplayer::displayConvexHulls(vector<Point_2> convexHull, vector<vector<Point_2>> obstacleConvexHulls) {
	//Display obstacles convex hulls
	float colorObstaclesConvexHull[3] = { 0 , 1 ,0 };
	vector<vector<Point_2>>::iterator obstacle;
	for (obstacle = obstacleConvexHulls.begin(); obstacle < obstacleConvexHulls.end(); obstacle++) {
		vector<Point_2> vertices = *obstacle;
		Polygon_2 shape(vertices.begin(), vertices.end());
		this->graphics->drawUnfilledPolygon(shape, colorObstaclesConvexHull);
	}

	//Display convex hulls
	float colorConvexHUll[3] = { 1 , 0 , 1 };
	Polygon_2 shape(convexHull.begin(), convexHull.end());
	this->graphics->drawUnfilledPolygon(shape, colorConvexHUll);

	//Display all vertices selected on vision sampling
	//RRTStarSukharevVertices* rrtVision = (RRTStarSukharevVertices*) this->getRRT();
	//this->displayBITSamples(rrtVision->get);
}

void RRTDisplayer::displayBITSamples(vector<NodeStar*> samples) {
	float pointColor[] = { 0.5, 0.1,  0.2};
	
	vector<NodeStar*>::iterator it = samples.begin();
	for (it; it < samples.end(); it++) {
		this->graphics->drawPoint((*it)->getX(), (*it)->getY(), pointColor, 8);
	}
}


void RRTDisplayer::display(Graphics* graphic) {
	this->graphics = graphic;

	RRTStarSukharevVertices* rrtVision = dynamic_cast<RRTStarSukharevVertices*> (this->getRRT());
	
	this->displayRRT(this->getRRT()->getStackNodes());
	//this->displaySmoothRRT(((RRTCurve*)this->getRRT())->getStackSmoothNodes());

	//if (((RRTCurve*)this->getRRT())->getSmoothRoot() != nullptr) {
	//	//this->displaySmoothRRT(((RRTCurve*)this->getRRT())->getStackSmoothNodes());
	//}

	//optimized route
	Route* route = this->getRRT()->getRoute();

	float lineColorRoute[] = { 1,0,0 };
	float lineColorNonOptRoute[] = { 0,0,0};
	float pointColorRoute[] = { 0.6, 0 ,0.8 };
	float pointColorNonOptRoute[] = { 0.6, 0.8 ,0 };

	if (route) {
		///*	float lineColor[] = { 1,1,0 };
		//	float pointColor[] = { 0.1, 0,5 ,0.3 };
		//	this->displayRoute(this->getRRT()->getGoal(), lineColor, pointColor);*/

		if (route->getSmoothInit()) {
			this->displaySmoothRoute(route->getSmoothGoal(), lineColorRoute, pointColorRoute);
		} else {
			RRTStarSmart* rrtSmart = dynamic_cast<RRTStarSmart*> (this->getRRT());
			RRTStarSukharevVertices* rrtSV = dynamic_cast<RRTStarSukharevVertices*> (this->getRRT());

			if (rrtSmart) {
				this->displayRoute(route->getOriginalGoal(), lineColorNonOptRoute, pointColorNonOptRoute);
			} else if (rrtSV) {
				this->displayRoute(route->getOriginalGoal(), lineColorNonOptRoute, pointColorNonOptRoute);
			}
			this->displayRoute(route->getGoal(), lineColorRoute, pointColorRoute);
		}
		//this->displayRoute(route->getSmoothGoal(), lineColorRoute, pointColorRoute);
		//}else{
		//	this->displayRoute(this->getRRT()->getGoal(), lineColorRoute, pointColorRoute);
	}

	BITStar* bit = dynamic_cast<BITStar*> (this->getRRT());

	if (bit) {
		this->displayBITSamples(bit->getSamplesRGG());
	}

	if (rrtVision) {
		RRTStarSukharevVertices* rrtVision = (RRTStarSukharevVertices*) this->getRRT();
		this->displayConvexHulls(rrtVision->getConvexHull(), rrtVision->getObstacleConvexHulls());
	}
}