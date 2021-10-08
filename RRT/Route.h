/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Route.h
 * Author: LuizGustavo
 *
 * Created on 8 de Agosto de 2016, 14:03
 */

#ifndef ROUTE_H
#define ROUTE_H

#include <vector>
#include "Node.h"

using namespace std;

class Route {
public:
    Route();
    Route(Node* routeInit, Node* routeGoal);
    virtual ~Route();
    void build();
	Node* getInit();
	Node* getGoal();
	vector<Node*> getNodesOfRoute();
	friend Route* smoothRoute(Route* route, double curvature);
	double getRouteCost();
	double getSmoothRouteCost();
	void generateCurvatureFile(string fileName);
	void setSmoothGoal(Node* smoothGoal);
	void setSmoothInit(Node* smoothInit);
	void setOriginalGoal(Node * originalGoal);
	Node* getSmoothGoal();
	Node* getSmoothInit();
	Node * getOriginalGoal();
private:
    vector<Node*> nodesOfRote;
	Node* routeInit;
    Node* routeGoal;
	Node* smoothInit;
	Node* smoothGoal;
	Node* originalGoal;
};

#endif /* ROTE_H */