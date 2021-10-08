#include "BITStar.h"
#include <utility>
#include <boost/sort/sort.hpp>
//#include <boost/range/algorithm/sort.hpp>
#include "ResultOutputer.h"
#include <chrono> 

using namespace std;

bool sortVertexCompare(const Vertex v1, const Vertex v2) {
	return v1.first < v2.first;
}

bool sortEdgeCompare(const Edge e1, const Edge e2) {
	return e1.first < e2.first;
}

BITStar::BITStar(NavigationEnvironment ne, const double r, int const n) : InformedRRTStar(ne, r, n) {
}

BITStar::BITStar(NavigationEnvironment ne, const double r, int const n, string experimentName) : InformedRRTStar(ne, r, n) {
	this->experiment = experimentName;
}

BITStar::~BITStar(){
}

////Linha 2
//vector<NodeStar*> samplesRGG;
////vector<pair<NodeStar*, NodeStar*>> queueEdges;
//vector<Edge> queueEdges;
//pair<NodeStar*, NodeStar*> bestEdgeInQueue;
////vector<NodeStar*> queueVertices;
//vector<pair<double, NodeStar*>> queueVertices;
//NodeStar* bestVerticeInQueue = nullptr;
//double radiusRGG = numeric_limits<double>::max();
//vector<Node*> nodesOld;
//vector<NodeStar*> samples; //Xsamples

void BITStar::build(NodeStar* qInit, NodeStar *qGoal, double distanceToGenerateRoute, string envName) {
	//int n = 15000, 
	bool activatePrint = false;
	int batchValue = 100; //Medida utilizada no artigo gammel BIT* 2015
	double changeCostRate = 0.01; //Medida utilizada no artigo gammel BIT* 2015 para aplicar o m�todo Prune
	double oldCost = numeric_limits<double>::max();

	//int batchValue = 200;
	/*NodeStar* qInit = new NodeStar{50, 50};
	NodeStar* qGoal = new NodeStar{800, 50};*/

	double accumulatedTime = 0, timeDataReport = 0;
	//Output file config for data generated by information
	int seed = navigationSpace.getSeedToRandomGenerator();

	string folderPath, simulationName, fileByIteractionName;
	if (this->experiment.size() == 0) {
		folderPath = "C:/RRTSimulations/byIteration/Informed_RRT/";
		folderPath = "C:/RRTSimulations/byIteration/BIT_Star/";
	} else {
		folderPath = "C:/RRTSimulations/byIteration/" + experiment + "Informed_RRT/";
		folderPath = "C:/RRTSimulations/byIteration/" + experiment + "BIT_Star/";
	}

	simulationName = envName + "/bitstar_byIteration_" + envName + "_" + to_string(seed);
	fileByIteractionName = folderPath + simulationName + ".txt";

	//string folderPath = "C:/RRTSimulations/byIteration/BIT_Star/";
	//string simulationName = envName + "/bitstar_byIteration_" + envName + "_" + to_string(seed);
	//string fileByIteractionName = folderPath + simulationName + ".txt";

	string columns[] = {"iteration", "cost", "sample_time", "total_time", "nodes" };
	//string columns[] = { "iteration", "cost", "sample_time", "add_node_time", "nodes" };
	char fileSeparator = ';';
	int sizeOfData = 5 - 1;
	ResultOutputer* costsByIterationsOutputer = new ResultOutputer{fileByIteractionName, columns, columns + sizeOfData, fileSeparator};

	int planningTimeInit = clock();
	qInit->setStartPosition(true);
	stackNodes.push_back(qInit);
	this->qtdNodes++;
	this->setRoot(qInit); this->goal = qGoal;
	samplesRGG.push_back(qGoal); //Linha 1
	//Fill elipse sampling data for the Informed RRT*
	this->configElipseData(qInit, qGoal);

	//while (i <= this->n) {
	//int i = 1;
	//while (accumulatedTime <= this->n) {
		//if (i == 150) {
		//	this->displayNow("C:/RRTSimulations/byIteration/BIT_Star/cluttered5Obstacles/bitstar_test2_zigzag", i);
		//}
		//this->displayNow(folderPath + simulationName, i);
		/*if (i % 1000 == 0) {
			this->displayNow(folderPath + simulationName, i);
		}*/
	int imgCount = 1; int lastQtdNodes = 0;
	double nextTime = 0;

	int i = 0;
	while (accumulatedTime <= this->n) {
		i++;
		/*if (nextTime <= accumulatedTime) {
			this->displayNow(folderPath + simulationName, nextTime);
			if (accumulatedTime == 0 || accumulatedTime / ((double)this->n) < 0.03) {
				nextTime = accumulatedTime + (((double)this->n) / 80000.00);
			} else {
				nextTime = accumulatedTime + (((double)this->n) / 200.00);
			}
		}*/
		double sampleTime = 0, displayTimeInit = 0, displayTimeEnd = 0;	

		//Primeira Parte = atualiza��o da RGG e gera��o do batch
		//int sampleTimeInit = clock();
		auto sampleTimeInit = chrono::high_resolution_clock::now();

		if (queueEdges.empty() && queueVertices.empty()) {
			//Batch creation
			double goalCost = ((NodeStar*)this->goal)->getCost();
			if((oldCost - goalCost)/oldCost >= changeCostRate){
				prune(goalCost); //remove amostras que n�o melhoram a solu��o //Linha 5
				oldCost = ((NodeStar*)this->goal)->getCost();
			}

			samples = generateBatch(batchValue);
			//samplesRGG.clear();
			samplesRGG.insert(samplesRGG.end(), samples.begin(), samples.end()); //Linha 6
			//samplesRGG.push_back(qGoal); //Linha 1
			nodesOld = stackNodes; //Linha 7
			sortVerticesQueue(stackNodes);
			radiusRGG = calcRadius(stackNodes.size() + samplesRGG.size()); //atualiza raio para o tamanho do RGG
			//radiusRGG = 1000;
			//this->displayNow("C:/RRTSimulations/byIteration/BIT_Star/zig_zag/bitstar_test2_zigzag", i);
		}

		//int sampleTimeEnd = clock();
		auto sampleTimeEnd = chrono::high_resolution_clock::now();

		//sampleTime = double(sampleTimeEnd - sampleTimeInit) / double(CLOCKS_PER_SEC);
		sampleTime = chrono::duration_cast<chrono::nanoseconds>(sampleTimeEnd - sampleTimeInit).count()*1e-9;

		//Segunda Parte: Edge selection and expansion
		//int addNewNodeTimeInit = clock();
		auto addNewNodeTimeInit =  chrono::high_resolution_clock::now();

		while (bestQueueValue(queueVertices) <= bestQueueValue(queueEdges)) {
			if (!queueVertices.empty()) {
				expandVertex(bestInQueue(queueVertices));;
			}			
			if (queueVertices.empty() && queueEdges.empty()) {
				break;
			}
		}
				
		if (queueEdges.empty() && queueVertices.empty()) {
			continue;
		}

		Edge bestEdge = bestInQueue(queueEdges); //Linha 12 //retorna a melhor aresta (menor custo aproximado)
		pair<NodeStar*, NodeStar*> edgeToRemove((NodeStar*)bestEdge.second.first, bestEdge.second.second);
		/*cout << "v: " << bestEdge.second.first->getX() << "," << bestEdge.second.first->getY();
		cout << "\t x: " << bestEdge.second.second->getX() << "," << bestEdge.second.second->getY()<< endl;
*/
		removeFromQueue(edgeToRemove); //Linha 13
		//Qe.remove_edge((v_m, x_m)); esse comando ir� dentro de bestInQueue		

	/*	if (queueEdges.empty() || queueVertices.empty()) {
			continue;
		}*/

		//Terceira Parte: Expande a �rvore baseada na melhor aresta selecionada
		NodeStar* vm = bestEdge.second.first;
		NodeStar* xm = bestEdge.second.second;
		double costFromNodeX = costFromVertice(bestEdge.second.second);
		double selectedCostEdge = calcCostEdge(bestEdge.second);
		double currentCostGoal = qGoal->getCost(); //Linha 14
		//custo = custo(qInit, v_m) + custo(v_m, x_m) + custo(x_m, qGoal);
		//a fun��o custo eh uma estimativa, que n�o realiza teste de colis�es. Isso evita que teste de colis�o desnecess�rios sejam realizados
		double costToNodeVTree = bestEdge.second.first->getCost();
		//double costByTree = costToNodeVTree + selectedCostEdge + costFromNodeX;

		double costByTree = vm->getCost() + selectedCostEdge + costFromVertice(xm);

		if (costByTree < currentCostGoal){
			//Custo_sample usa o conjunto X ao inv�s somente de V (que � referente � �rvore) para calcular o custo
			//custo = custo_samples(qInit, v_m) + true_custo(v_m, x_m) + custo(x_m, qGoal);
			double realCostEdge = calcRealCostEdge(bestEdge);
			//double costToNodeV = costToVertice(bestEdge.second.first); //Linha 15
			double costToNodeV = costToVertice(vm); //Linha 15
			//cout << costToVertice(vm) << "\t" << realCostEdge << "\t" << costFromVertice(xm) << endl;
			if (costToVertice(vm) + realCostEdge + costFromVertice(xm) < currentCostGoal) {
				double realCostToNodeX = costToNodeVTree  + realCostEdge; //Linha 16
				if (vm->getCost() + realCostEdge < xm->getCost()) {
					NodeStar* oldParent = nullptr;
					if (isOnVertices(xm)) { //rewiring
						//Linha 18
						oldParent = (NodeStar*)xm->getAntecessor();
						if (oldParent) {
							oldParent->removeSucessor(xm);
							xm->setAntecessor(nullptr);
						}
						
						insertNode(vm, xm);//Linha 22
					} else { //expansion
						   // se n�o est� em V, � porque � novo e ser� transferido do batch (Xsamples) a �rvore
						/*if (xm ==  this->goal){
							int ok = 10;
						} else {*/
						removeFromSamplesRGG(xm);
						//}
						
						this->stackNodes.push_back(xm);
						this->qtdNodes++;
						//queueVertices.erase(queueVertices.begin() + this->bestVertexIndex);
						insertNode(vm, xm);//Linha 22
						insertIntoVerticesQueue(xm);
						boost::sort::flat_stable_sort(queueVertices.begin(), queueVertices.end(), sortVertexCompare);
						//queueVertices.push_back();
					}

					//insertNode(vm, xm);//Linha 22 Aqui � onde a �rvore realmente � expandida!!

					//Linha 23
					if (oldParent && oldParent->getCost() + calcCostEdge(oldParent, xm) >= xm->getCost()) {
						pair<NodeStar*, NodeStar*> edgeToRemove(oldParent, xm);
						removeFromQueue(edgeToRemove);
						if(queueEdges.size() > 0)
							boost::sort::flat_stable_sort(queueEdges.begin(), queueEdges.end(), sortEdgeCompare);
					}

					/*if (xm == this->goal) {
						this->displayNow(folderPath + simulationName + "_founded", i);
					}*/
				}
			}
		} else {
			//delete Qe;
			//delete Qv;	
			queueEdges.clear();
			//bestEdgeIndex = -1;
			queueVertices.clear();
			//bestVertexIndex = -1;
		}

		if (this->goal->getAntecessor() && oldCost == numeric_limits<double>::max()) {
			Route* newRoute = new Route{ this->root, this->goal };
			newRoute->build();
			this->route = newRoute;
			if (activatePrint) {
				displayTimeInit = clock();
				this->displayNow(folderPath + simulationName + "_founded_route_", i);
				displayTimeEnd = clock();
				activatePrint = false;
			}
		}

		//int addNewNodeTimeEnd = clock();
		auto addNewNodeTimeEnd = chrono::high_resolution_clock::now();

		//double addNewNodeTime = double(addNewNodeTimeEnd - addNewNodeTimeInit) / double(CLOCKS_PER_SEC);
		double addNewNodeTime = chrono::duration_cast<chrono::nanoseconds>(addNewNodeTimeEnd - addNewNodeTimeInit).count()*1e-9;

		addNewNodeTime = addNewNodeTime - ((displayTimeEnd - displayTimeInit) / double(CLOCKS_PER_SEC));

		accumulatedTime += (sampleTime + addNewNodeTime);
		
		////***INICIO GERA��O DE ESTAT�STICAS***
		//Save data to current iterator {"Iteration", "cost", "sample_time", "total_time", "nodes"};
		int init = clock();

		double cost;
		if (this->route) {
			cost = ((NodeStar*)(this->route->getGoal()))->getCost();
		} else {
			cost = 0;
		}

		float data[] = {i, cost, sampleTime, accumulatedTime, this->qtdNodes };
		//float data[] = { i, cost, sampleTime, addNewNodeTime, accumulatedTime };
		costsByIterationsOutputer->pushData(data, data + sizeOfData);
		/*if (!this->goal->getAntecessor() && (this->qtdNodes - lastQtdNodes) >= 2 && (accumulatedTime - nextTime) > 0.0012) {
			this->displayNow(folderPath + simulationName, imgCount);
			costsByIterationsOutputer->pushData(data, data + sizeOfData);
			lastQtdNodes = this->qtdNodes;
			nextTime = accumulatedTime;
			imgCount++;
		}
		else */
		/*if (accumulatedTime <= 0.0004 && ((accumulatedTime - nextTime) >= 0.000020)){
			this->displayNow(folderPath + simulationName, imgCount);
			costsByIterationsOutputer->pushData(data, data + sizeOfData);
			lastQtdNodes = this->qtdNodes;
			nextTime = accumulatedTime;
			imgCount++;
		}else if (accumulatedTime <= 0.015 && ((accumulatedTime - nextTime) >= 0.0003)) {
			this->displayNow(folderPath + simulationName, imgCount);
			costsByIterationsOutputer->pushData(data, data + sizeOfData);
			lastQtdNodes = this->qtdNodes;
			nextTime = accumulatedTime;
			imgCount++;
		} else if ((accumulatedTime - nextTime) >= 0.015) {
			this->displayNow(folderPath + simulationName, imgCount);
			costsByIterationsOutputer->pushData(data, data + sizeOfData);
			nextTime = accumulatedTime;
			imgCount++;
		}*/

		int end = clock();
		timeDataReport += ((end - init) / double(CLOCKS_PER_SEC));
		////***FIM GERA��O DE ESTAT�STICAS***

		//i++;
	}
	

	int planningTimeEnd = clock();
	this->planningTime = ((planningTimeEnd - planningTimeInit) / double(CLOCKS_PER_SEC)) - timeDataReport;
	std::cout << "Tempo de planejamento: " << this->planningTime << endl;
	if (activatePrint) {
		this->displayNow(folderPath + simulationName, this->n);

	}

	//Display to final time
	//this->displayNow(folderPath + simulationName, imgCount);
	float data[] = { i, ((NodeStar*)(this->route->getGoal()))->getCost(), 0, accumulatedTime, this->qtdNodes };
	costsByIterationsOutputer->pushData(data, data + sizeOfData);
}

/*PSEUDOCODE*/
////Primeira Parte = atualiza��o da RGG e gera��o do batch
//if (RGG vazio) {
//	//Batch creation
//	prune(qGoal); //remove amostras que n�o melhora a solu��o
//	Xsamples = batchCreation(m, ); //adiciona m amostras para o RGG
//	vOld = v; //soment novos estados s�o considerados e
//	Qv = v // reenfileirados para expans�o
//	radiusRGG = updatgeRadius(v.size(Xsamples)) //atualiza raio para o tamanho do RGG
//}

////Segunda Parte: Edge selection
//while (BestInQueueValue(Qv) <= BestInQueueValue(Qe)) {
//	ExpandVertex(BestInQueue(Qv));
//}
//(v_m, x_m) = BestInQueue(Qe); //retorna a melhor aresta (menor custo aproximado)
//Qe.remove_edge((v_m, x_m));

////Terceira Parte: 
//custo = custo(qInit, v_m) + custo(v_m, x_m) + custo(x_m, qGoal);
////a fun��o custo eh uma estimativa, que n�o realiza teste de colis�es. Isso evita que teste de colis�o desnecess�rios sejam realizados
//if (custo < custo(qInit, qGoal)){
//	//Custo_sample usa o conjunto X ao inv�s somente de V (que � referente � �rvore) para calcular o custo
//	custo = custo_samples(qInit, v_m) + true_custo(v_m, x_m) + custo(x_m, qGoal);
//	if (custo < custo(qInit, qGoal)) {
//		custo_x = custo(qInit, v_m) + true_custo(v_m, x_m);
//		if (custo_x < custo(qInit, x_m)) {
//			if (V.is_in(x_m)) { //rewiring
//				E.remove(x_m.antecessor(), x_m);
//			} else { //expansion
//				// se n�o est� em V, � porque � novo e ser� transferido do batch (Xsamples) a �rvore 
//				Xsamples.remove(x_m);
//				V.add(x_m);
//				Qv.add(x_m);
//			}
//			E.add(v_m, x_m);
//			//pelo que parece,  x_m.antecessor() na verdade � v_m
//			custo = custo(qInit, x_m.antecessor()) + custo(x_m.antecessor(), x_m);
//			if (custo >= custo(qInit, x_m)) {
//				Qe.remove(x_m.antecessor(), x_m);
//			}
//		}
//	}
//}
//delete Qe;
//delete Qv;

void BITStar::prune(double goalCost) {
	double theoricalCost;

	//Prune X_samples removing: Linha 1
	vector<NodeStar*>::iterator itRGG = this->samplesRGG.begin();
	for (itRGG; itRGG < samplesRGG.end(); itRGG++) {
		NodeStar* x = (*itRGG);
		theoricalCost = costToVertice(x) + costFromVertice(x);
		if (theoricalCost >= goalCost) {
			this->samplesRGG.erase(itRGG);
		}
	}

	//Prune edges E: Linha 3
	vector<Node*>::iterator itEdges = this->stackNodes.begin();
	for (itEdges; itEdges < this->stackNodes.end(); itEdges++) {
		NodeStar* w = (NodeStar*)(*itEdges);
		NodeStar* v = (NodeStar*)w->getAntecessor();
		if (v) {
			double theoricalCostV = costToVertice(v) + costFromVertice(v);
			double theoricalCostW = costToVertice(w) + costFromVertice(w);

			if (theoricalCostV > goalCost || theoricalCostW > goalCost) {
				v->removeSucessor(w);
				w->setAntecessor(nullptr);
			}
		}
	}

	//Prune vertices V: Linha 2
	vector<Node*>::iterator itVertices = this->stackNodes.begin();
	for (itVertices; itVertices < this->stackNodes.end(); itVertices++) {
		NodeStar* v = (NodeStar*)(*itVertices);
		theoricalCost = costToVertice(v) + costFromVertice(v);
		if (theoricalCost > goalCost) {
			this->stackNodes.erase(itVertices);
		}
	}	

	//Prune X_samples adding from vertices tree: Linhas 4 e 5
	itVertices = this->stackNodes.begin();
	for (itVertices; itVertices < this->stackNodes.end(); itVertices++) {
		NodeStar* v = (NodeStar*)(*itVertices);
		double infinity = numeric_limits<double>::max();
		if (v->getCost() == infinity) {
			this->stackNodes.erase(itVertices);
			this->samplesRGG.push_back(v);
		}
	}
}

//Expands the edgeQueue acoording with RGG
void BITStar::expandVertex(NodeStar* v){
	removeFromQueue(v);
	vector<NodeStar*> xNear = collectNear(samplesRGG, v, radiusRGG);
	double goalCost = ((NodeStar*)this->goal)->getCost();

	//**expande primeiro considerando Xsamples (os batchs)
	for (NodeStar* x : xNear) {
		double costEdge = calcCostEdge(v, x);
		double pathCost = costToVertice(v) + costEdge + costFromVertice(x);
		//se o custo pelo vertice v for menor do que o custo corrente ent�o adiciona ele a fila de arestas da RGG.
		if (pathCost < goalCost) {
			insertIntoEdgesQueue(v, x);
		}
	}

	//**e depois expande considerando os v�rtices da �rvore
	if (!isOnOldVertices(v)) { // se n�o faz parte de v�rtices j� adicionados anteriormente
		vector<NodeStar*> qNear = this->getNeighborsInRadius(v, radiusRGG); 
		//V.collectNear(v, radius);
		for(NodeStar* w : qNear){
			double costEdgeVW = calcCostEdge(v, w);
			double costByW = costToVertice(v) + costEdgeVW + costFromVertice(w);
			double costToW = v->getCost() + costEdgeVW;

			//se o custo pelo vertice v for menor do que o custo corrente ent�o adiciona ele a fila de arestas da RGG.
			if ((costByW < goalCost) && costToW < w->getCost() && !isInEdgeQueue(v, w)) {
				insertIntoEdgesQueue(v, w);
			}
		}
	}

	if (queueEdges.size() != 0) {
		boost::sort::flat_stable_sort(queueEdges.begin(), queueEdges.end(), sortEdgeCompare);
	}

	/*PSEUDOCODE*/
	//Qv.remove(v);
	//Xnear = Xsamples.collectNear(v, radius);

	////**expande primeiro considerando Xsamples (os batchs)
	//for (x in Xnear) {
	//	custo_path = custo(qInit, v) + custo(v, x) + custo(x, qGoal);
	//	//se o custo pelo vertice v for menor do que o custo corrente ent�o adiciona ele a fila de arestas da RGG.
	//	if (custo_path < custo(qInit, qGoal)) {
	//		Qe.add_edge(v, x);
	//	}
	//}

	////**e depois expande considerando os v�rtices da �rvore
	//if (!Vold.is_inserted(v)) { // se n�o faz parte de v�rtices j� adicionados anteriormente
	//	Vnear = V.collectNear(v, radius);
	//	for (w in Vnear) {
	//		custo_path = custo(qInit, w) + custo(v, w) + custo(w, qGoal);
	//		custo_w = custo(qInit, v) + custo(v, w);

	//		//se o custo pelo vertice v for menor do que o custo corrente ent�o adiciona ele a fila de arestas da RGG.
	//		if (custo_path < custo(qInit, qGoal) && 
	//			!E.is_inserted(v,w) && 
	//			custo_w < custo(qInit, w)) {
	//			Qe.add_edge(v, w);
	//		}
	//	}
	//}
}

double BITStar::calcRealCostEdge(Edge bestEdge){
	double resultCost;
	if (collisionChecker.batchCollisionFreePath(bestEdge.second.first, bestEdge.second.second)) {
		resultCost = MathUtils::euclidianDistanceNode(bestEdge.second.first, bestEdge.second.second);
	} else {
		resultCost = numeric_limits<double>::max();
	}

	return resultCost;
}

void BITStar::insertIntoVerticesQueue(NodeStar* v) {
	double estimatedCost = (v)->getCost() + costFromVertice(v);
	pair<double, NodeStar*> key(estimatedCost, v);
	queueVertices.push_back(key);
	//boost::sort::flat_stable_sort(queueVertices.begin(), queueVertices.end(), sortVertexCompare);
}


void BITStar::insertIntoEdgesQueue(NodeStar* v, NodeStar* x) {
	pair<NodeStar*, NodeStar*> newEdge(v, x);
	double costEdge = MathUtils::euclidianDistanceNode(v,x);
	double estimatedCostPath = v->getCost() + costEdge + costFromVertice(x);
	Edge key(estimatedCostPath, newEdge);
	queueEdges.push_back(key);
}

//Corresponds to g^(v) function
double BITStar::costToVertice(NodeStar* v) {
	double result = MathUtils::euclidianDistanceNode(this->root, v);
	return result;
}

//Corresponds to h^(v) function
double BITStar::costFromVertice(NodeStar* v){
	double result = MathUtils::euclidianDistanceNode(v, this->goal);
	return result;
}

double BITStar::calcCostEdge(NodeStar* v, NodeStar* x) {
	double result = MathUtils::euclidianDistanceNode(v, x);
	return result;
}

double BITStar::calcCostEdge(pair<NodeStar*, NodeStar*> edge) {
	double result = MathUtils::euclidianDistanceNode(edge.first, edge.second);
	return result;
}

vector<NodeStar*> BITStar::generateBatch(double batchValue) {
	vector<NodeStar*> batchSamples;
	if (this->goal->getAntecessor()) {
		this->updateTransformingData(((NodeStar*)this->goal)->getCost());
	}
	while (batchValue > 0) {
		batchValue--;
		//N�o precisa passar o custo como parametro pois o mesmo ser� obtido pelo campor "rota" da classe 
		//RRT em informedRRTSampling
		NodeStar* sample = this->informedRRTSampling();
		batchSamples.push_back(sample);
	}

	return batchSamples;
}

double BITStar::calcRadius(int samplesQtd) {
	double nabla = 1.1;
	double lambda = lebesgueMeasure();
	double sigma = 3.14;

	double termoA = (2.0 * nabla);
	double termoB = pow(1 + (1.0 / 2.0), (1.0 / 2.0));
	double termoC = pow(lambda / sigma, (1.0 / 2.0));
	double termoD = pow(log(samplesQtd) / samplesQtd, (1.0 / 2.0));

	/*double radius = (2 * nabla) * 
					pow(1 + (1 / 2),(1 / 2)) * 
					pow(lambda / sigma, (1 / 2)) * 
					pow(log(samplesQtd/samplesQtd), (1/2));*/
	double radius = termoA * termoB * termoC * termoD;
	return radius;
}

double BITStar::lebesgueMeasure() {
	/*double higherX = numeric_limits<double>::min(), higherY = numeric_limits<double>::min();
	double lesserX = numeric_limits<double>::max(), lesserY = numeric_limits<double>::max();

	for (Vertex v : queueVertices) {
		if (higherX < v.second->getX()) {
			higherX = v.second->getX();
		}
		if (higherY < v.second->getY()) {
			higherY = v.second->getY();
		}
		if (lesserX > v.second->getX()) {
			lesserX = v.second->getX();
		}
		if (lesserY > v.second->getY()) {
			lesserY = v.second->getY();
		}
	}
	
	double sideA = higherX - lesserX;
	double sideB = higherY - lesserY;
	return sideA*sideB;*/
	if (route) {
		double transverseDiameter = ((NodeStar*)this->goal)->getCost();
		double minTransverseDiameter = MathUtils::euclidianDistanceNode(this->root, this->goal);
		double conjugateDiameter = std::sqrt(transverseDiameter * transverseDiameter - minTransverseDiameter * minTransverseDiameter);
		double area = Pi*transverseDiameter*conjugateDiameter;
		return area;
	}else {
		Node* begin = this->getNavigationEnvironment()->getInitialCoordinate();
		Node* end = this->getNavigationEnvironment()->getFinalCoordinate();
		double area = (end->getX() - begin->getX()) * (end->getY() - begin->getY());
		return area;
	}
}

void BITStar::sortVerticesQueue(vector<Node*> vertices) {
	vector<Node*>::iterator verticesIt;
	for (verticesIt = vertices.begin(); verticesIt < vertices.end(); verticesIt++) {
		NodeStar* v = ((NodeStar*)*verticesIt);
		insertIntoVerticesQueue(v);
	}

	boost::sort::flat_stable_sort(queueVertices.begin(), queueVertices.end(), sortVertexCompare);
}


double BITStar::bestQueueValue(vector<Vertex> &queue){
	if (queue.size() != 0) {
		//this->bestVertexIndex++; //inicia em -1
		double bestValue = queue.at(this->bestVertexIndex).first;
		return bestValue;
	}
	return numeric_limits<double>::max();
}

double BITStar::bestQueueValue(vector<Edge> &queue){
	if (queue.size()!=0) {
		//this->bestEdgeIndex++; //inicia em -1
		double bestValue = queue.at(this->bestEdgeIndex).first;
		return bestValue;
	}

	return numeric_limits<double>::max();
}

NodeStar* BITStar::bestInQueue(vector<Vertex> &queueVertices) {
	if (queueVertices.size()!=0) {
		return queueVertices.at(this->bestVertexIndex).second;
	}
	return nullptr;
}

Edge BITStar::bestInQueue(vector<Edge> &queueEdges) {
	if (queueEdges.size() != 0) {
		return queueEdges.at(this->bestEdgeIndex);
	}
	return Edge();
}

bool BITStar::removeFromQueue(NodeStar* v){
	NodeStar* vertice = bestInQueue(queueVertices);
	//NodeStar* vertice = queueVertices.at(this->bestVertexIndex).second;
	if (vertice == v) {
		VertexIt vIt = queueVertices.begin() + this->bestVertexIndex;
		queueVertices.erase(vIt);
		return true;
	}
	return false;
}

bool BITStar::removeFromQueue(pair<NodeStar*, NodeStar*> edgeToRemove) {
	for (EdgeIt e = queueEdges.begin(); e < queueEdges.end(); e++) {
		if ((*e).second.first == edgeToRemove.first && (*e).second.second == edgeToRemove.second){
			queueEdges.erase(e);
			return true;
		}
	}
	return false;
}

vector<NodeStar*> BITStar::collectNear(vector<NodeStar*> samplesRGG, NodeStar* v, double radius) const{
	//vector<NodeStar*> result = new vector<NodeStar*>{};
	vector<NodeStar*> result;

	for (int i = 0; i < samplesRGG.size(); i++) {
		if (!(v == samplesRGG[i])) {
			if (MathUtils::euclidianDistanceNode(samplesRGG[i], v) <= radius) {
				result.push_back(samplesRGG[i]);
			}
		}
	}

	return result;
}

bool BITStar::isOnOldVertices(NodeStar* v) {
	for (Node* oldV : nodesOld){
		NodeStar compareV = (NodeStar*)oldV;
		if (compareV == v) {
			return true;
		}
	}
	return false;
}

bool BITStar::isOnVertices(NodeStar* v) {
	for (Node* oldV : this->stackNodes) {
		NodeStar compareV = (NodeStar*)oldV;
		if (compareV == v) {
			return true;
		}
	}
	return false;
}

bool BITStar::isInEdgeQueue(NodeStar* v, NodeStar* w) {
	for (Edge edge : queueEdges) {
		if (edge.second.first == v && edge.second.second == w) {
			return true;
		}
	}
	return false;
}

bool BITStar::removeFromSamplesRGG(NodeStar* v){
	vector<NodeStar*>::iterator it;
	for (it = samplesRGG.begin(); it < samplesRGG.end(); it++) {
		if ((*it) == v) {
			samplesRGG.erase(it);
			return true;
		}
	}
	return false;
}

vector<NodeStar*> BITStar::getSamplesRGG() {
	return this->samplesRGG;
}