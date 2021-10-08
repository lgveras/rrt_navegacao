#include "RRTCurve.h"
#include "CurveCollisionChecker.h"
#include "G2PlanarPH.h"
#include "CPointPH.h"
#include "Utils.h"
#include <vector>
using namespace std;

RRTCurve::RRTCurve(NavigationEnvironment ne, int const n) : RRT(ne, n){
}

RRTCurve::~RRTCurve(){
}

void RRTCurve::build(Node* qInit, Node *qGoal, double distanceToGenerateRoute, string envName) {
	Node* qRand;
	Node* qNear;
	int i = 0;
	int s = 0;
	
	//this->collisionChecker = BialkowskiCollisionChecker{&(this->navigationSpace)};
	CurveCollisionChecker curveCheker = CurveCollisionChecker{&(this->navigationSpace)};

	double accumulatedTime = 0, timeDataReport = 0;
	
	this->setRoot(qInit);
	this->stackNodes.push_back(qInit);
	this->qtdNodes++;
	this->goal = qGoal;
	this->distanceToGoal = distanceToGenerateRoute;

	this->smoothGoal = qGoal->createCopy();

	//File name details
	int seed = navigationSpace.getSeedToRandomGenerator();
	string folderPath = "C:/RRTSimulations/byIteration/RRT_Smart/";
	string simulationName = envName + "/rrtstar_smart_byIteration_" + envName + "_" + to_string(seed);

	double L = 100;

	for (int i = 1; i <= this->n; i++) {
		double sampleTime = 0;
	
		qRand = this->generateRandomNode();
		qNear = this->nearestNode(qRand);

		Node* qNew = this->createNewNode(qNear, qRand);

		if (qNew != NULL) {
			if (collisionChecker.batchCollisionFreePath(qNear, qNew)) {
				this->insertNode(qNear, qNew);
				this->stackNodes.push_back(qNew);
				this->qtdNodes++;

				if (i%250 == 0) {
					this->displayNow(folderPath + simulationName + "_smoothed_founded_path", i);
				}

				if (qNear->getAntecessor()) {
					//suaviza a rota em qNear, ou seja, suavizar considerando qNear.Antecessor()-qNear/qNear-qNew
					G2PlanarPH* curve = smoothSegmentPair(qNear->getAntecessor(), qNear, qNew, L);
					//Passa a rota suavizada para a checagem de colisão
					if (curve && !curveCheker.isCurveCollisionFree(curve)) {
						//salvar o novo pedaço da rota, sem estar suavizado e suavizado, separadamente
						insertInSmoothedTree(qNear->getAntecessor(), qNear, curve, qNew, L);
						
						if (MathUtils::euclidianDistanceNode(qNew, qGoal) <= distanceToGenerateRoute) {
							//TODO: INSERIR UMA CONDIÇÃO EM QUE O qGoal SÓ É ADICIONADO À ÁRVORE SE PERMITE SUAVIZAÇÃO PARA A CURVATURA DEFINIDA
							G2PlanarPH* curveGoal = smoothSegmentPair(qNew->getAntecessor(), qNew, qGoal, L);
							if (collisionChecker.batchCollisionFreePath(qNew, qGoal) && curveGoal) {
								insertInSmoothedTree(qNew->getAntecessor(), qNew, curveGoal, qGoal, L);
								this->insertNode(qNew, qGoal);
								//vector<Node*>::iterator endStack = this->stackSmoothNode.end();
								/*Node* qSmoothNew = this->stackSmoothNode.at(this->stackSmoothNode.size()-1);
								this->insertNode(qSmoothNew, this->getSmoothGoal());*/

								Route* newRoute = new Route{ qInit, qGoal };
								newRoute->setSmoothInit(this->smoothRoot);
								newRoute->setSmoothGoal(this->smoothGoal);
								newRoute->build();
								this->route = newRoute;
								this->displayNow(folderPath + simulationName + "_smoothed_founded_path", i);
							}
						}
					} else {
						//se negada, imprimir a situação, caso contrário adicionar o nó a rota com a curva já gerada!!
						this->removeNode(qNear, qNew);
						this->stackNodes.pop_back();
						this->qtdNodes--;
						//this->displayNow(folderPath + simulationName + "_founded_route_", i);
					}
				}
			}
		}
	}

	//Increment to count qGoal addition
	this->qtdNodes++;
}

G2PlanarPH* RRTCurve::smoothSegmentPair(Node* nodeRef_i, Node* nodeRef_c, Node* nodeRef_o, double L) {
	double distance1 = MathUtils::euclidianDistanceNode(nodeRef_i, nodeRef_c);
	double distance2 = MathUtils::euclidianDistanceNode(nodeRef_c, nodeRef_o);
	double L2 = L*2;

	//cout << "Pi-Pc: " << distance1 << " Pc-Po: " << distance2 << "L: " << L << " Dgoal: " << this->distanceToGoal;

	if (distance1 > L2 && distance2 > L2){
		vector<CPointPH> p;
		p.push_back(CPointPH(nodeRef_i->getX(), nodeRef_i->getY()));
		p.push_back(CPointPH(nodeRef_c->getX(), nodeRef_c->getY()));
		p.push_back(CPointPH(nodeRef_o->getX(), nodeRef_o->getY()));

		CPointPH* points = Utils::cutEdge(&p[0], L);
		CPointPH cuttedPoint[3] = { points[0], points[1], points[2] };

		double distanceCuttedPoint = MathUtils::euclidianDistance(cuttedPoint[2].xPos, cuttedPoint[2].yPos, cuttedPoint[1].xPos, cuttedPoint[1].yPos);
		double distanceOriginalPoint = MathUtils::euclidianDistance(p[2].xPos, p[2].yPos, p[1].xPos, p[1].yPos);

		//Aplication of restrition 1, when L is greater than the length of one of the segments
		if (distanceCuttedPoint < distanceOriginalPoint) {
			return Utils::smoothEdge(cuttedPoint, L);
		}
	}

	return nullptr;
}

void RRTCurve::insertInSmoothedTree(Node * nodeRef_i, Node * nodeRef_c, G2PlanarPH* curve, Node * nodeRef_o, double L){

	vector<CPointPH> p;
	p.push_back(CPointPH(nodeRef_i->getX(), nodeRef_i->getY()));
	p.push_back(CPointPH(nodeRef_c->getX(), nodeRef_c->getY()));
	p.push_back(CPointPH(nodeRef_o->getX(), nodeRef_o->getY()));

	CPointPH* points = Utils::cutEdge(&p[0], L);
	CPointPH cuttedPoint[3] = { points[0], points[1], points[2] };
	
	Node* nodePi = new Node(cuttedPoint[0].xPos, cuttedPoint[0].yPos);
	Node* nodePc;

	if (nodeRef_c->getReferenceCurveNode()) {
		nodePc = nodeRef_c->getReferenceCurveNode();
	} else {
		nodePc = new Node(cuttedPoint[1].xPos, cuttedPoint[1].yPos);
	}

	Node* nodePo = new Node(cuttedPoint[2].xPos, cuttedPoint[2].yPos, curve);
	
	//If don't exists reference, so p_i and p_c were not created yet!
	if (!nodeRef_c->getReferenceCurveNode()) {
		Node* antecessorPi ;
		if (!nodeRef_i->getReferenceCurveNode()) {
			antecessorPi = nodeRef_i->createCopy();
			this->stackSmoothNode.push_back(antecessorPi);
			this->smoothRoot = antecessorPi;
			nodeRef_i->setReferenceCurveNode(antecessorPi);
		} else {
			antecessorPi = nodeRef_i->getReferenceCurveNode();
		}
	
		insertNode(antecessorPi, nodePi);
		this->stackSmoothNode.push_back(nodePi);

		insertNode(nodePi, nodePc);
		this->stackSmoothNode.push_back(nodePc);

		nodeRef_c->setReferenceCurveNode(nodePc);
	//Else, take the existed p_i and p_c were not created yet!
	} else {
		Node* lastPo = nodePc->getAntecessor();
		if (lastPo->getX() != nodePi->getX() && lastPo->getY() != nodePi->getY()) {
			insertNode(lastPo, nodePi);
			this->stackSmoothNode.push_back(nodePi);
			insertNode(nodePi, nodePc);
		}
	}

	insertNode(nodePc, nodePo);
	this->stackSmoothNode.push_back(nodePo);
	
	Node* sucessorPo;

	if (nodeRef_o->getX() == this->smoothGoal->getX() && nodeRef_o->getY() == this->smoothGoal->getY()){
		sucessorPo = smoothGoal;
		if (!nodeRef_o->getReferenceCurveNode()){
			nodeRef_o->setReferenceCurveNode(sucessorPo);
			this->stackSmoothNode.push_back(sucessorPo);			
		}		
	}else{
		sucessorPo = nodeRef_o->createCopy();
		nodeRef_o->setReferenceCurveNode(sucessorPo);
		this->stackSmoothNode.push_back(sucessorPo);
		insertNode(nodePo, sucessorPo);
	}

	insertNode(nodePo, sucessorPo);
}

vector<Node*> RRTCurve::getStackSmoothNodes() {
	return this->stackSmoothNode;
}

Node* RRTCurve::getSmoothRoot() {
	return this->smoothRoot;
}

Node* RRTCurve::getSmoothGoal() {
	return this->smoothGoal;
}