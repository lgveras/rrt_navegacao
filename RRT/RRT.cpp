
/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RRT.cpp
 * Author: LuizGustavo
 * 
 * Created on 8 de Agosto de 2016, 14:03
 */

#include "RRT.h"
#include "MathUtils.h"
#include "Node.h"
#include "Waypoint.h"
#include "Route.h"
#include "BialkowskiCollisionChecker.h"
#include "RRTDisplayer.h"
#include "CollisionCheckerDisplayer.h"
#include "EnvironmentDisplayer.h"
#include "ResultOutputer.h"
#include <cstddef>
#include <vector>
#include <iostream>
#include <limits>
#include <cmath>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
//#include <CGAL/intersections.h>
//#include <CGAL/Boolean_set_operations_2.h>
#include <ctime>
#include "CGALDefinitions.h"

//typedef CGAL::Exact_predicates_exact_constructions_kernel K;
//typedef K::Point_2 Point_2;
//typedef K::Segment_2 Segment_2;
//typedef K::Intersect_2 Intersect_2;
//typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes_2;

using namespace std;

Graphics* g = NULL;
RRT::RRT(NavigationEnvironment ne, int const n):navigationSpace(ne), n(n){
	this->qtdNodes = 0;
	this->stackNodes;
	this->collisionChecker = BialkowskiCollisionChecker{ &(this->navigationSpace) };

	/*this->nodesGrid = new SearchGrid{ ne.getInitialCoordinate()->getX(), ne.getInitialCoordinate()->getY(),
					 ne.getFinalCoordinate()->getX(), ne.getFinalCoordinate()->getY(),
					 ne.getQtdColumns(), ne.getQtdRows()};*/
}

RRT::~RRT() {
}

void RRT::setRoot(Node* node) {
    this->root = node;
	//this->nodesGrid->insertNode(node);
	//this->stackNodes.push_back(node);
}

void RRT::setRoute(Route* route) {
	this->route = route;
}

void RRT::setDispersion(double dispersion) {
	this->deltaQ = dispersion;
}

Node* RRT::generateRandomNode() {
  /*  if((!this->navigationSpace.gridSukharevEmpty())){
        return this->navigationSpace.getNodeFromSukharevGrid();
    }else{*/
    return this->navigationSpace.generateNode();
    /*}*/
}

Node* RRT::nearestNode(Node* qRand){
	Node* qNear = NULL;
    double distance, limitDistance;

    limitDistance = numeric_limits<double>::max();

	vector<Node*>::iterator currentNodeIt;
	for(currentNodeIt=this->stackNodes.begin(); currentNodeIt < this->stackNodes.end(); currentNodeIt++) {
        distance = MathUtils::euclidianDistanceNode(*currentNodeIt, qRand);
        if (distance < limitDistance) {
            limitDistance = distance;
            qNear = *currentNodeIt;
        }
    }
    return qNear;
	//return this->nodesGrid->searchNearestTo(qRand);
}


//Nó criado pelo Teorema de Pitágoras
Node* RRT::createNewNode(Node* qNear, Node* qRand) {
	if (qNear->getX() == qRand->getX() && qNear->getY() == qRand->getY()) {
		return nullptr;
	}

    Node* qNew;
    double hipotenusa, deltaQLat, deltaQLong, porcentagemDeltaQ, catetoSin, catetoCos, senoTeta, cosTeta, vetorULat, vetorULong;
	double qNearX = qNear->getNodeWaypoint()->getLatitude(), qNearY = qNear->getNodeWaypoint()->getLongitude();
	double qRandX = qRand->getNodeWaypoint()->getLatitude(), qRandY = qRand->getNodeWaypoint()->getLongitude();

	//O avanço de um nó se dá sempre 10% do comprimento do ambiente de navegação.
    //porcentagemDeltaQ = 0.0628;
	porcentagemDeltaQ = 0.03; //Usado no artigo da RRT*-SV
	//porcentagemDeltaQ = 0.01;
    deltaQLat = porcentagemDeltaQ * (this->getNEFinalCoordinate()->getLatitude() -
								     this->getNEInitialCoordinate()->getLatitude());
	
	deltaQLong = porcentagemDeltaQ * (this->getNEFinalCoordinate()->getLongitude() -
									  this->getNEInitialCoordinate()->getLongitude());
	/*if (this->qtdNodes == 2) {
		cout << "deltaqlat: " << deltaQLat << "; deltaqlat: " << deltaQLong << endl;
	}*/

	// if is setted deltaQ, so it must be used to correspond to dispersion value of Sukharev grid
	if (this->deltaQ > 1) {
		deltaQLat = this->deltaQ;
		deltaQLong = this->deltaQ;
	}

	hipotenusa = MathUtils::euclidianDistanceNode(qNear, qRand);

	catetoCos = qRandX - qNearX;
	catetoSin = qRandY - qNearY;

	/*cosTeta = catetoCos / hipotenusa;
	vetorULat = qNear->getNodeWaypoint()->getLatitude() + (cosTeta)*deltaQLat;

	senoTeta = catetoSin / hipotenusa;
	vetorULong = qNear->getNodeWaypoint()->getLongitude() + (senoTeta)*deltaQLong;*/

    if ( hipotenusa > deltaQLat && hipotenusa > deltaQLong) {
		cosTeta = catetoCos / hipotenusa;
		vetorULat = qNear->getNodeWaypoint()->getLatitude() + (cosTeta)*deltaQLat;

		senoTeta = catetoSin / hipotenusa;
		vetorULong = qNear->getNodeWaypoint()->getLongitude() + (senoTeta)*deltaQLong;
		
	} else {
		cosTeta = catetoCos / hipotenusa;
		vetorULat = qNear->getNodeWaypoint()->getLatitude() + (cosTeta)*hipotenusa;

		senoTeta = catetoSin / hipotenusa;
		vetorULong = qNear->getNodeWaypoint()->getLongitude() + (senoTeta)*hipotenusa;
	}

	qNew = new Node{ vetorULat, vetorULong };
	this->getNavigationEnvironment()->putNodeInsideLimits(qNew);
	return qNew;
}

void RRT::insertNode(Node* qParent, Node* qChild){
    if(this->root == nullptr && qParent == nullptr){
        this->root = qChild;
    } else {
		qChild->setAntecessor(qParent);
		qParent->addSucessor(qChild);
    }
	//this->nodesGrid->insertNode(qChild);
	//this->stackNodes.push_back(qChild);
	//this->qtdNodes++;
}

void RRT::removeNode(Node* qParent, Node* qChild){
	/*if (this->root != nullptr && qParent != nullptr) {
		qChild->setAntecessor(nullptr);
		qParent->removeSucessor(qChild);
		for (int i = 0; i < this->stackNodes.size(); i++) {
			if (this->stackNodes[i] == qChild) {
				this->stackNodes.erase(this->stackNodes.begin()+i);
			}
		}
		this->qtdNodes--;
	}*/
	qChild->setAntecessor(nullptr);
	qParent->removeSucessor(qChild);
	//this->qtdNodes--;
	/*for (int i = 0; i < this->stackNodes.size(); i++) {
		if (this->stackNodes[i] == qChild) {
			this->stackNodes.erase(this->stackNodes.begin() + i);
			break;
		}
	}*/
}

BialkowskiCollisionChecker RRT::getCollisionChecker() {
	return this->collisionChecker;
}

Node* RRT::pathCompleted(Node* qNew, Node* qGoal, double distanceToGenerateRoute) {
	if (MathUtils::euclidianDistanceNode(qNew, qGoal) <= distanceToGenerateRoute) {
		vector<Node*> nodes;
		nodes.push_back(qNew);
		if (collisionChecker.batchCollisionFreePath(nodes, qGoal)) {
			return qNew;
		}
	}
	return nullptr;
}

//bool RRT::pathCompleted(Node* qNew, Node* qGoal, double distanceToGenerateRoute) {
//	if (MathUtils::euclidianDistanceNode(qNew, qGoal) <= distanceToGenerateRoute) {
//		vector<Node*> nodes;
//		nodes.push_back(qNew);
//		if (collisionChecker.batchCollisionFreePath(nodes, qGoal)) {
//			if (qGoal->getAntecessor()) {
//
//			} else {
//				this->insertNode(qNew, qGoal);
//				return true;
//			}
//			
//			//TODO: Testar a rota
//			//Route* route{qGoal};
//			//percorre a árvore de trás pra frente até chegar em qInit
//			//route->build();  
//		}
//	}
//	return false;
//}

void RRT::build(Node* qInit, Node *qGoal, double distanceToGenerateRoute, string envName, int selectDispersion, string cells) {
    Node* qRand;
    Node* qNear;
    int s = 0;
    //pilha para busca do mais próximo
    //this->stackNodes.push_back(qInit);
    // ALGORITMO RRT - LINHA 2: s recebe zero
    // ALGORITMO RRT - LINHA 3: enquanto s for igual a zero faca
	double accumulatedTime = 0, timeDataReport = 0;
	//Output file config for data generated by information
	int seed = navigationSpace.getSeedToRandomGenerator();
	string folderPath = "C:/RRTSimulations/byIteration/RRT/";
	string simulationName = envName + "/rrt_byIteration_" + envName + "_" + to_string(seed);

	if (selectDispersion == 1) {
		simulationName = envName + "/rrt_sukha_" + cells + "_byIteration_" + envName + "_" + to_string(seed);
	}else if(selectDispersion == 2) {
		simulationName = envName + "/rrt_lattice_" + cells + "_byIteration_" + envName + "_" + to_string(seed);
	}

	string fileByIteractionName = folderPath + simulationName + ".txt";
	string columns[] = { "iteration", "cost", "sample_time", "total_time", "nodes" };
	char fileSeparator = ';';
	int sizeOfData = 5 - 1;
	ResultOutputer* costsByIterationsOutputer = new ResultOutputer{ fileByIteractionName, columns, columns + sizeOfData, fileSeparator };
	
	int planningTimeInit = clock();

	// ALGORITMO RRT - LINHA 1: inserir q_init na arvore RRT
	this->setRoot(qInit);
	this->stackNodes.push_back(qInit);
	this->qtdNodes++;
	this->goal = qGoal;
	this->distanceToGoal = distanceToGenerateRoute;

	int i = 0;
	int imgCount = 1; int lastQtdNodes = 0;
	double nextTime = 0;

	//while (accumulatedTime <= this->n) {
    //while (s == 0) {
		/*if (i%250 == 0) {
			this->displayNow(folderPath + simulationName, i);
		}*/
		/*this->displayNow(folderPath + simulationName + "_example_", 0);
		break;*/
	for (int i = 1; i <= this->n; i++) {
       /* cout << "iteracao = " <<  i << " numero de elementos = " << this->qtdNodes << endl;/*/
		//i++;
		double sampleTime = 0;
		int sampleTimeInit = clock();
        // ALGORITMO RRT - LINHA 4: q_rand recebe RAND_CONFIG(C)
        qRand = this->generateRandomNode();
		//if (collisionChecker.collisionFreePoint(qRand)){
        // ALGORITMO RRT - LINHA 5: q_near recebe NEAREST_VERTEX(q_rand,G)
        qNear = this->nearestNode(qRand);
		//vector<Node*> nodes;
		//nodes.push_back(qNear);
		//if (collisionChecker.batchCollisionFreePath(nodes, qRand)) {
		// ALGORITMO RRT - LINHA 6: q_new recebe NEW_CONF(q_near, delta_q)
		// ALGORITMO RRT - LINHA 7: se o segmento de reta (q_near,q_new) nao intercepta qualquer obstaculo faca
		Node* qNew = this->createNewNode(qNear, qRand);

		int sampleTimeEnd = clock();
		sampleTime = (sampleTimeEnd - sampleTimeInit) / double(CLOCKS_PER_SEC);
		int addNewNodeTimeInit = clock();

		if (qNew != NULL){
			// Garantindo que q_new esteja dentro dos limites do ambiente de navegacao
		//	if (collisionChecker.collisionFreePoint(qNew)) {
				// ALGORITMO RRT - LINHA 8: inserir o no q_new na arvore G
				// ALGORITMO RRT - LINHA 9: inserir a aresta que une q_near a q_new na arvore G
				// ALGORITMO RRT - LINHA 10: antecessor(q_new) recebe q_near
		/*	vector<Node*> nodes;
			nodes.push_back(qNear);*/
			if (collisionChecker.batchCollisionFreePath(qNear, qNew)) {
				this->insertNode(qNear, qNew);
				this->stackNodes.push_back(qNew);
				this->qtdNodes++;
				//this->insertNode(qNear, qRand);
				//pilha para busca do mais próximo
				//this->stackNodes.push_back(qNew);
				//this->stackNodes.push_back(qNew);
				//if (MathUtils::euclidianDistanceNode(qNew, qGoal) <= distanceToGenerateRoute){
				if (MathUtils::euclidianDistanceNode(qNew, qGoal) <= distanceToGenerateRoute) {
					//if (this->testFreeSegment(qNew, qGoal)) {
			/*		vector<Node*> nodes;
					nodes.push_back(qNew);*/
					if (collisionChecker.batchCollisionFreePath(qNew, qGoal)) {
						//qNew já foi adicionado, agora é o último ponto da arvore
						//se passou por esse if, é porque qGol deve ser adicionado como novo ponto
						//this->insertNode(qNew, qGoal);
						if (qGoal->getAntecessor()) {
							this->removeNode(qGoal->getAntecessor(), qGoal);
						}
						this->insertNode(qNew, qGoal);
						// ALGORITMO RRT - LINHA 15: s recebe 1
						s = 1;
						//TODO: Testar a rota
						Route* newRoute = new Route{ qInit, qGoal };
						newRoute->build();
						this->route = newRoute;

						this->displayNow(folderPath + simulationName + "_founded_route_", i);

						//Route* route{qInit, qGoal};
						////percorre a árvore de trás pra frente até chegar em qInit
						//route->build();            
					}
				}
					
			}
		}	

		int addNewNodeTimeEnd = clock();
		double addNewNodeTime = (addNewNodeTimeEnd - addNewNodeTimeInit) / double(CLOCKS_PER_SEC);

		int init = clock();
		accumulatedTime += (sampleTime + addNewNodeTime);

		////***INICIO GERAÇÃO DE ESTATÍSTICAS***
		//Save data to current iterator {"Iteration", "cost", "sample_time", "total_time", "nodes"};
		float data[] = { i, ((NodeStar*)this->goal)->getCost(), sampleTime, accumulatedTime, this->qtdNodes };
		//costsByIterationsOutputer->pushData(data, data + sizeOfData);
/*
		if (!this->goal->getAntecessor() && (this->qtdNodes - lastQtdNodes) != 0) {
			this->displayNow(folderPath + simulationName, imgCount);
			costsByIterationsOutputer->pushData(data, data + sizeOfData);
			lastQtdNodes = this->qtdNodes;
			nextTime = accumulatedTime;
			imgCount++;
		} else */if ((i % 350)==0) {
			this->displayNow(folderPath + simulationName, imgCount);
			costsByIterationsOutputer->pushData(data, data + sizeOfData);
			nextTime = accumulatedTime;
			imgCount++;
		}

		//Save the simulation to BMP file
		/*if (i % 250 == 0) {
			this->displayNow(folderPath + simulationName, i);
		}*/
		int end = clock();		
		timeDataReport += (end - init) / double(CLOCKS_PER_SEC);
		////***FIM GERAÇÃO DE ESTATÍSTICAS***
    }   

	//Increment to count qGoal addition
	this->qtdNodes++;

	//Display to final time
	this->displayNow(folderPath + simulationName, imgCount);
	float data[] = { i, ((NodeStar*)(this->route->getGoal()))->getCost(), 0, accumulatedTime, this->qtdNodes };
	costsByIterationsOutputer->pushData(data, data + sizeOfData);

	int planningTimeEnd = clock();
	double time = ((planningTimeEnd - planningTimeInit) / double(CLOCKS_PER_SEC)) - timeDataReport;
	this->planningTime = time <= 0? 0 : time;
	/*std::cout << "planningTimeEnd: " << planningTimeEnd << ";  planningTimeInit: " << planningTimeInit << "; clock:" << double(CLOCKS_PER_SEC) << ";  timeDataReport: " << timeDataReport << endl;
		std::cout << "Tempo: " << ((planningTimeEnd - planningTimeInit) / double(CLOCKS_PER_SEC)) - timeDataReport << endl;*/
	std::cout << "Tempo de planejamento: " << this->planningTime << endl;
}

Node* RRT::getRoot(){
    return this->root;
}

Node * RRT::getGoal(){
	return this->goal;
}

Waypoint* RRT::getNEInitialCoordinate(){
    return this->navigationSpace.getInitialCoordinate()->getNodeWaypoint();
}

Waypoint* RRT::getNEFinalCoordinate(){
    return this->navigationSpace.getFinalCoordinate()->getNodeWaypoint();
}

vector<Node*> RRT::getStackNodes() {
	return this->stackNodes;
}

SearchGrid* RRT::getSearchGrid() {
	return this->nodesGrid;
}

double RRT::getPlanningTime(){
	return this->planningTime;
}

int RRT::getQuantityNodes(){
	return this->qtdNodes;
}

NavigationEnvironment* RRT::getNavigationEnvironment() {
	return &(this->navigationSpace);
}

void RRT::displayNow(string pathFile, int iteration) {
	int argc = 0;
	char **argv = NULL;

	if (g == NULL) {
		g = new Graphics();
		////Cria o objeto para mostrar a RRT.
		RRTDisplayer* rrtDisplayer = new RRTDisplayer{ this };
		//Cria o objeto para mostrar o Ambiente de navegação.
		EnvironmentDisplayer* edDisplayer = new EnvironmentDisplayer{ this->navigationSpace };
		//Cria o objeto para mostrar os certificados da checagem de colisão
		CollisionCheckerDisplayer* ccDisplayer = new CollisionCheckerDisplayer{ this->getCollisionChecker() };

		//A Ordem de registro define quem será mostrado primeiro no layer do OpenGl
		g->registerDisplayable(edDisplayer);
		g->registerDisplayable(ccDisplayer);
		g->registerDisplayable(rrtDisplayer);
	}

	string convResult = to_string(iteration);
	/*stringstream strb;
	strb << setfill('0') << setw(4) << convResult << std::endl;
	string result;
	std::getline(strb, result);*/
	string fileImageName = pathFile + "_print_it_" + convResult + ".bmp";
	g->init(argc, argv, this->navigationSpace.getInitialCoordinate()->getNodeWaypoint(),
		this->navigationSpace.getFinalCoordinate()->getNodeWaypoint(), fileImageName);
	g->startGraphics();
}

void RRT::displayNow(string pathFile, double iteration) {
	int argc = 0;
	char **argv = NULL;

	if (g == NULL) {
		g = new Graphics();
		////Cria o objeto para mostrar a RRT.
		RRTDisplayer* rrtDisplayer = new RRTDisplayer{ this };
		//Cria o objeto para mostrar o Ambiente de navegação.
		EnvironmentDisplayer* edDisplayer = new EnvironmentDisplayer{ this->navigationSpace };
		//Cria o objeto para mostrar os certificados da checagem de colisão
		CollisionCheckerDisplayer* ccDisplayer = new CollisionCheckerDisplayer{ this->getCollisionChecker() };

		//A Ordem de registro define quem será mostrado primeiro no layer do OpenGl
		g->registerDisplayable(edDisplayer);
		g->registerDisplayable(ccDisplayer);
		g->registerDisplayable(rrtDisplayer);
	}

	string fileImageName = pathFile + "_print_it_" + to_string(iteration) + ".bmp";
	g->init(argc, argv, this->navigationSpace.getInitialCoordinate()->getNodeWaypoint(),
		this->navigationSpace.getFinalCoordinate()->getNodeWaypoint(), fileImageName);
	g->startGraphics();
}

double RRT::getDistanceToGoal(){
	return this->distanceToGoal;
}

Route * RRT::getRoute(){
	return this->route;
}

void RRT::zigZagTest() {
	Node* node1 = new Node(500, 500);
	this->setRoot(node1);
	Node* node2 = new Node(1500, 4500);
	this->insertNode(node1,node2);
	Node* node3 = new Node(2500, 500);
	this->insertNode(node2, node3);
	Node* node4 = new Node(3500, 4500);
	this->insertNode(node3, node4);
	Node* node5 = new Node(4500, 500);
	this->insertNode(node4, node5);

	this->setRoot(node1);
	this->goal = node1;
	
	this->route = new Route(node1,node5);
	this->route->build();

	this->displayNow("ok", 0);
}