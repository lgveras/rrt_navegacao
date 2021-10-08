/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Rote.cpp
 * Author: LuizGustavo
 * 
 * Created on 8 de Agosto de 2016, 14:03
 */

// a rota vai ser um Vector (ou list) de pontos.
#include "Route.h"
#include "Node.h"
#include "NodeStar.h"
#include <cstddef>
#include "CPointPH.h"
#include "G2PlanarPH.h"
#include "Utils.h"
#include "MathUtils.h"

Route::Route() {
}

Route::Route(Node* routeInit, Node* routeGoal){
    this->routeGoal = routeGoal;
	this->routeInit = routeInit;
}

Route::~Route(){
}

void Route::build(){
    //Gerar a rota. Implementar o algoritmo a seguir na classe Rote
    //vector<int>::iterator = this->nodesOfRote.begin();
    //algoritmo equivalente no programa do Felipe é o Inserir_No_Pilha
    Node* node = this->routeGoal;
    while(node != NULL){
        this->nodesOfRote.push_back(node);
        node = node->getAntecessor();
    }
}

Node * Route::getInit(){
	return this->routeInit;
}

Node * Route::getGoal(){
	return this->routeGoal;
}

vector<Node*> Route::getNodesOfRoute(){
	return this->nodesOfRote;
}

double Route::getRouteCost(){
	double totalCost = 0;
	Node* currentNode = this->getGoal();

	while (currentNode->getAntecessor() != nullptr) {
		if (currentNode->getCurve()) {
			totalCost += currentNode->getCurve()->G2getTotalArcLength();
		} else {
			if (currentNode->getAntecessor()->getCurve()) {
				currentNode = currentNode->getAntecessor();
				continue;
			}
			totalCost += MathUtils::euclidianDistanceNode(currentNode, currentNode->getAntecessor());
		}
		currentNode = currentNode->getAntecessor();
	}

	return totalCost;
}

double Route::getSmoothRouteCost() {
	double totalCost = 0;
	Node* currentNode = this->getSmoothGoal();

	while (currentNode->getAntecessor() != nullptr) {
		if (currentNode->getCurve()) {
			totalCost += currentNode->getCurve()->G2getTotalArcLength();
		} else {
			if (currentNode->getAntecessor()->getCurve()) {
				currentNode = currentNode->getAntecessor();
				continue;
			}
			totalCost += MathUtils::euclidianDistanceNode(currentNode, currentNode->getAntecessor());
		}
		currentNode = currentNode->getAntecessor();
	}

	return totalCost;
}

Route* smoothRoute(Route* route, double curvature){

	Node* currentNode = route->getGoal();
	Node* currentPo = nullptr;

	vector<CPointPH> p;
	int count = 0;

	//double L = MathUtils::euclidianDistanceNode(route->getInit(), route->getInit()->getSucessors()[0]);

	while (currentNode) {
		if (count == 0) {
			currentPo = new Node{ route->getGoal()->getX(), route->getGoal()->getY() };
		}

		count++;

		if (count <= 2) {
			p.push_back(CPointPH(currentNode->getX(), currentNode->getY()));
		}

		if (count >= 2 && currentNode->getAntecessor()) {
			p.push_back(CPointPH(currentNode->getAntecessor()->getX(), currentNode->getAntecessor()->getY()));

			double L = G2getMaxL(&p[0], curvature);
			//double L = 2000;

			CPointPH* points = Utils::cutEdge(&p[0], L);
			CPointPH cuttedPoint[3] = { points[0], points[1], points[2]};

			double distanceCuttedPoint = MathUtils::euclidianDistance(cuttedPoint[2].xPos, cuttedPoint[2].yPos, cuttedPoint[1].xPos, cuttedPoint[1].yPos);
			double distanceOriginalPoint = MathUtils::euclidianDistance(p[2].xPos, p[2].yPos, p[1].xPos, p[1].yPos);

			if (distanceCuttedPoint <= distanceOriginalPoint) {
				G2PlanarPH* curve = Utils::smoothEdge(cuttedPoint, L);

				//the order is inverse beacuse the route is smoothed from goal to init (inverse to sequence pi, pc, po from article)
				Node* nodePi = new Node(cuttedPoint[2].xPos, cuttedPoint[2].yPos);
				Node* nodePc = new Node(p[1].xPos, cuttedPoint[1].yPos, curve);
				Node* nodePo = new Node(cuttedPoint[0].xPos, cuttedPoint[0].yPos);

			/*	if ((currentNode->getX() == nodePi->getX() && currentNode->getY() == nodePi->getY()) == false) {
					nodePi->setAntecessor(new Node(currentNode->getX(), currentNode->getY()));
				}*/

				nodePc->setAntecessor(nodePi);
				nodePo->setAntecessor(nodePc);

				//Nova estrutura que conserva a rota não suavizada
				if (currentPo->getX() == route->getGoal()->getX() && currentPo->getY() == route->getGoal()->getY()) {
					route->setSmoothGoal(currentPo);
				}

				if (currentPo->getX() == nodePo->getX() && currentPo->getY() == nodePo->getY()) {
					currentPo->setAntecessor(nodePc);
					currentPo = nodePc;
				} else {
					currentPo->setAntecessor(nodePo);
					currentPo = nodePi;
				}				

				//currentPo->setAntecessor(nodePo);
				//if (currentPo == route->getGoal()) {
				//	Node* copyPo = new Node(currentPo->getX(), currentPo->getY());
				//	route->setSmoothGoal(copyPo);
				//	currentPo = copyPo;
				//}

				if (currentNode->getAntecessor()->getAntecessor() == nullptr) {
					if (currentPo->getX() == route->getInit()->getX() && currentPo->getY() == route->getInit()->getY()) {
						route->setSmoothInit(currentPo);
					} else {
						Node* smoothInit = new Node{ route->getInit()->getX() , route->getInit()->getY()};
						currentPo->setAntecessor(smoothInit);

						route->setSmoothInit(currentPo);
					}
					break;
					/*if (nodePi->getAntecessor()) {
						route->setSmoothInit(nodePi->getAntecessor());
					} else {
						route->setSmoothInit(nodePi);
					}*/
				}

				/*	if ((currentPo->getX() == nodePo->getX() && currentPo->getY() == nodePo->getY()) == false) {
					currentPo->setAntecessor(nodePo);
				} else {
					currentPo->setAntecessor(nodePi);
				}*/

				p.clear();
				p.push_back(CPointPH(currentNode->getX(), currentNode->getY()));
				p.push_back(CPointPH(currentNode->getAntecessor()->getX(), currentNode->getAntecessor()->getY()));
				//p.push_back(CPointPH(nodePi->getX(), nodePi->getY()));
			} else {
				p.pop_back();
			}
		} 
		
		currentNode = currentNode->getAntecessor();
	}

	return route;
}

void Route::generateCurvatureFile(string fileName) {
	Node* currentNode = this->getSmoothGoal();
	double stepCurve = 0.01;
	double step = 20;
	int itTotal = 0;
	cout << "step;curvature" << endl;
	while (currentNode->getAntecessor() != nullptr) {
		if (currentNode->getAntecessor()->getCurve()) {
			G2PlanarPH* curve = currentNode->getAntecessor()->getCurve();
			//Algoritmo para cálculo da curvatura!
			double kappa;
			for (double t = 0; t <= 1; t += stepCurve) {
				//TODO: Modificar a parametrização da curva pelo seu comprimento!
				itTotal++;
				kappa = G2getCurvature(*curve, t);
				//CPointPH ptTemp = CPointPH(real(G2beval(Dgr, curve->spline[1].p, t)), imag(G2beval(Dgr, curve->spline[1].p, t)));
				//print itTotal; curvature
				cout << itTotal << ";" << abs(kappa) << endl;
			}
		} else {
			double distance = MathUtils::euclidianDistanceNode(currentNode, currentNode->getAntecessor());
			int it = distance / step;
			double kappa = 0;
			for (double t = 0; t <= distance; t += step) {
				itTotal++;
				//print itTotal; curvature
				cout << itTotal << ";" << kappa << endl;
			}
		}
		currentNode = currentNode->getAntecessor();
	}
}

void Route::setSmoothGoal(Node* smoothGoal) {
	this->smoothGoal = smoothGoal;
}

void Route::setSmoothInit(Node* smoothInit) {
	this->smoothInit = smoothInit;
}

void Route::setOriginalGoal(Node* originalGoal) {
	this->originalGoal = originalGoal;
}

Node* Route::getSmoothGoal() {
	return this->smoothGoal;
}

Node* Route::getSmoothInit() {
	return this->smoothInit;
}

Node* Route::getOriginalGoal() {
	return this->originalGoal;
}
