/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RRTPrinter.h
 * Author: LuizGustavo
 *
 * Created on 30 de Agosto de 2016, 10:05
 */

#ifndef RRTDISPLAYER_H
#define RRTDISPLAYER_H

#include "Graphics.h"
#include "RRT.h"
#include "Displayable.h"

//TODO: Refatorar para criar uma classe de Displayer e extende-la
class RRTDisplayer : public Displayable {
public:
    RRTDisplayer(RRT* rrt);
    RRTDisplayer(const RRTDisplayer& orig);
    virtual ~RRTDisplayer();	
	void displayRoute(Node * qGoal, float lineColor[3], float pointColor[3]);
	void display(Graphics* graphic);
private:
    Graphics* graphics;
    RRT* rrt;
    int countPrintedPoints;
	RRT* getRRT();
	void displayRRT(vector<Node*> nodes);
	void displayPHCurve(G2PlanarPH* curve);
	void displayCurveForSucessors(vector<Node*> listNodesP_o);
	void displaySmoothRRT(vector<Node*> nodes);
	void displayRRT(SearchGrid* nodes);
	void displayRoute(void);
	void displaySmoothRoute(Node* qGoal, float lineColor[3], float pointColor[3]);
	void displayConvexHulls(vector<Point_2> convexHull,	vector<vector<Point_2>> obstacleConvexHulls);
	void displayBITSamples(vector<NodeStar*> samples);
};

#endif /* RRTDISPLAYER_H */