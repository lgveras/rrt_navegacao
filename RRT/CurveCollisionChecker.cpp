#include "CurveCollisionChecker.h"
#include "StaticObstacle.h"
#include <G2PlanarPH.h>

CurveCollisionChecker::CurveCollisionChecker(NavigationEnvironment* ne):ne(ne){
}

CurveCollisionChecker::~CurveCollisionChecker(){
}

bool CurveCollisionChecker::isCurveCollisionFree(G2PlanarPH* curve) {
	//Ponto médio da curva
	double t = 0.5;
	CPointPH ptTemp = CPointPH(real(G2beval(Dgr, curve->spline[1].p, t)), imag(G2beval(Dgr, curve->spline[1].p, t)));
	Node* curveMiddleNode = new Node(ptTemp.xPos, ptTemp.yPos);

	vector<StaticObstacle*> obstacles = this->ne->getStaticObstacles();
	if(obstacles.size() > 0){
		for (int i = 0; i < obstacles.size(); i++) {
			if (obstacles[i]->intersectionNode(curveMiddleNode)) {
				return true;
			}
		}
	} 

	return false;
}