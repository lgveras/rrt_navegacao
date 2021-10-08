#include "BialkowskiCollisionChecker.h"
//#include "CGALDefinitions.h"

//typedef CGAL::Exact_predicates_exact_constructions_kernel K;
//typedef K::Point_2 Point_2;
//typedef K::Segment_2 Segment_2;
//typedef CGAL::Polygon_2<K> Polygon_2;
//typedef Polygon_2::Vertex_iterator VertexIterator;
//typedef Polygon_2::Edge_const_iterator EdgeIterator;
//typedef CGAL::Polytope_distance_d_traits_2<K> Traits;
//typedef CGAL::Polytope_distance_d<Traits> PolytopeDistance;

BialkowskiCollisionChecker::BialkowskiCollisionChecker(){
}

BialkowskiCollisionChecker::BialkowskiCollisionChecker(NavigationEnvironment* ne):ne(ne){
}

BialkowskiCollisionChecker::~BialkowskiCollisionChecker(){
}

bool BialkowskiCollisionChecker::insideObstacle(Node* sample) {
	vector<StaticObstacle*> obstacles = ne->getStaticObstacles();
	for (int i = 0; i < obstacles.size(); i++) {
		vector<Cell*> cells = obstacles[i]->getObstacleCells();
		for (int j = 0; j < cells.size(); j++) {
			Point_2 samplePoint{ sample->getX(), sample->getY() };
			int sideOfPolygon = CGAL::bounded_side_2(cells[j]->getShape()->vertices_begin(), cells[j]->getShape()->vertices_end(), samplePoint);
			if (sideOfPolygon == CGAL::ON_BOUNDED_SIDE || sideOfPolygon == CGAL::ON_BOUNDARY) {
				return true;
			}
		}
	}
	return false;
}

map<Node*, double, NodePointerCompare>  BialkowskiCollisionChecker::getCollisionFreeSet(){
	return this->collisionFree;
}

map<Node*, double, NodePointerCompare>  BialkowskiCollisionChecker::getCollisionObsSet(){
	return this->collisionObs;
}

bool BialkowskiCollisionChecker::collisionFreePoint(Node * sample){
	if (this->ne->getStaticObstacles().size() == 0) {
		return true;
	}

	Node* vNear = nearestNodeInSet(collisionFree, sample);
	if (vNear != NULL && MathUtils::euclidianDistanceNode(sample, vNear) <= collisionFree[vNear]) {
		return true;
	}else {
		Node* oNear = nearestNodeInSet(collisionObs, sample);
		if(oNear != NULL && MathUtils::euclidianDistanceNode(sample, oNear) <= collisionObs[oNear]) {
			return false;
		}
	}
	
	//TODO:: Distance to free space é altamente custoso computacionalmente
	double dObs = distanceToObstacles(sample);
	//double dObs = 50;
	if ( dObs >= 0 ) {	
		collisionFree[sample] = dObs;
		return true;
	} else {
		double dFree = distanceToFreeSpace(sample);
		//double dFree = 50;
		collisionObs[sample] = dFree;
		return false;
	}
}

void BialkowskiCollisionChecker::insertInColisionFreePointSet(Node* node, double distance) {
	this->collisionFree[node] = distance;
}

bool BialkowskiCollisionChecker::batchCollisionFreePath(Node* node, Node * sample) {
	vector<Node*> nodes;
	nodes.push_back(node);
	return batchCollisionFreePath(nodes, sample);
}

bool BialkowskiCollisionChecker::batchCollisionFreePath(vector<Node*> nodes, Node * sample) {
	return testFreeSegment(nodes[0], sample);
	if (this->ne->getStaticObstacles().size() == 0) {
		return true;
	}

	//TODO: otimizar nearestNodeInSet
	Node* vNear = nearestNodeInSet(collisionFree, sample);
	//Do jeito que está aqui, só dá pra testar para um segmento. Para testar vários segmentos um container com os segmentos
	//selecionados deveria ser retornado ao invês de um valor booleano
	for (int i = 0; i < nodes.size(); i++) {
		if(MathUtils::euclidianDistanceNode(nodes[i], vNear) <= collisionFree[vNear]){
			return true;
		} if (testFreeSegment(nodes[i], sample)) {
			return true;
		}
	}

	return false;
}

Node* BialkowskiCollisionChecker::nearestNodeInSet(map<Node*, double, NodePointerCompare> set, Node * sample){
	Node* nearestNode = NULL;
	double lesserDistance = std::numeric_limits<double>::max();

	for (map<Node*, double>::iterator it = set.begin(); it != set.end(); ++it) {
		if (it->first->getX() == sample->getX() && it->first->getY() == sample->getY()) {
			return it->first;
		}
		double distance = MathUtils::euclidianDistanceNode(it->first, sample);
		if (distance < lesserDistance) {
			nearestNode = it->first;
			lesserDistance = distance;
		}
	}

	return nearestNode;
}

double BialkowskiCollisionChecker::distanceToObstacles(Node * sample) {
	Cell* sampleCell = ne->getCellByXYCoordinate(sample->getX(), sample->getY());
	if (sampleCell->getBinaryGridValue() == 1) {
		return -1;
	}
	sampleCell->~Cell();

	return calcMinimumDistanceToObstacles(sample);
}

double BialkowskiCollisionChecker::distanceToFreeSpace(Node * sample) {
	Cell* sampleCell = ne->getCellByXYCoordinate(sample->getX(), sample->getY());
	if (sampleCell->getBinaryGridValue() == 0) {
		return -1;
	}
	sampleCell->~Cell();

	return calcMinimumDistanceToObstacles(sample);
}

//double BialkowskiCollisionChecker::calcMinimumDistanceToObstacles(Node * sample) {
//	double minimalDistanceToObstacle = std::numeric_limits<double>::max();
//	vector<StaticObstacle*> obstacles = ne->getStaticObstacles();
//	Cell* nearestCell = NULL;
//
//	for (int i = 0; i < obstacles.size(); i++) {
//		vector<Cell*> cells = obstacles[i]->getObstacleCells();
//		for (int j = 0; j < cells.size(); j++) {
//			//Ver como matar esse iterador vit
//			for (VertexIterator vit = cells[j]->getShape()->vertices_begin(); vit < cells[j]->getShape()->vertices_end(); vit++) {
//				//Encontrar vértice mais próximo somente
//				Point_2 vertice = *vit;
//				double x = vertice.x().approx().inf();
//				double y = vertice.y().approx().inf();
//				
//				Node vertexNode{x, y};
//				double distance = MathUtils::euclidianDistanceNode(&vertexNode, sample);
//				vertexNode.~Node();
//				//vertice.~Point_2();
//
//				if (distance < minimalDistanceToObstacle) {
//					minimalDistanceToObstacle = distance;
//					nearestCell = cells[j];					
//				}
//			}
//		}
//		cells.~vector();
//	}	
//	obstacles.~vector();
//
//	vector<Cell*> neighborhood = ne->getNeigborhoodObstacleCell(nearestCell);
//	Point_2 samplePoint{sample->getX(), sample->getY()};
//	//faz os calculos em cima dos vizinhos
//	for (int i = 0; i < neighborhood.size(); i++) {
//		//Calcular sobre cada lado dos vizinhos
//		//Se não ficar muito lento pode deixar assim
//		int vertexIndex = 0;
//		for (VertexIterator vit = neighborhood[i]->getShape()->vertices_begin(); vit < neighborhood[i]->getShape()->vertices_end(); vit++) {
//			Point_2 vertex = *vit;
//			//Pega o vertice mais proximo.
//			//Pega o vertice anterior e posterior
//			Point_2 previousVertex = neighborhood[i]->getShape()->vertex(vertexIndex == 0 ? 3 : vertexIndex - 1);
//			Point_2 nextVertex = neighborhood[i]->getShape()->vertex(vertexIndex == 3 ? 0 : vertexIndex + 1);
//			//Vê de quem a amostra está mais próxima
//			Point_2 selectedVertex =
//				MathUtils::euclidianDistancePoint(&previousVertex, &samplePoint) > MathUtils::euclidianDistancePoint(&nextVertex, &samplePoint) ?
//				nextVertex : previousVertex;
//			//O incremento será em cima da reta formada pelo vertice selecionado anteriormente e o mais próximo do ponto amostrado
//			//Calcular o coeficiente angular ou qual eixo é igual entre os pontos
//			//Só funciona para polígono regular de quatro arestas
//			double distance;
//			if (selectedVertex.x() == vertex.x()) {
//				//se as coordanas x de ambos os vertíces forem iguais,
//				//então faz-se o incremento no eixo y.
//				Point_2 smallerYPoint = selectedVertex.y().exact().to_double() > vertex.y().exact().to_double() ?
//					vertex : selectedVertex;
//				double interval = abs(selectedVertex.y().exact().to_double() - vertex.y().exact().to_double());
//				double steps = interval / 100;
//				for (double y = 0; y < interval; y += steps) {
//					Node nodeOnEdge{ smallerYPoint.x().exact().to_double(),
//						smallerYPoint.y().exact().to_double() + y };
//					distance = MathUtils::euclidianDistanceNode(sample, &nodeOnEdge);
//					if (distance < minimalDistanceToObstacle) {
//						minimalDistanceToObstacle = distance;
//					}
//					nodeOnEdge.~Node();
//				}
//				//smallerYPoint.~Point_2();
//			} else {
//				//se as coordanas y de ambos os vertíces forem iguais,
//				//então faz-se o incremento no eixo x.
//				Point_2 smallerXPoint = selectedVertex.x().exact().to_double() > vertex.x().exact().to_double() ?
//					vertex : selectedVertex;
//				double interval = abs(selectedVertex.x().exact().to_double() - vertex.x().exact().to_double());
//				double steps = interval / 100;
//				for (double x = 0; x < interval; x += steps) {
//					Node nodeOnEdge{ smallerXPoint.x().exact().to_double() + x,
//						smallerXPoint.y().exact().to_double() };
//					distance = MathUtils::euclidianDistanceNode(sample, &nodeOnEdge);
//					if (distance < minimalDistanceToObstacle) {
//						minimalDistanceToObstacle = distance;
//					}
//					nodeOnEdge.~Node();
//				}
//				//smallerXPoint.~Point_2();
//			}
//			vertexIndex++;
//		/*	vertex.~Point_2();
//			previousVertex.~Point_2();
//			nextVertex.~Point_2();
//			selectedVertex.~Point_2();*/
//		}	
//	}
//	neighborhood.~vector();
//	samplePoint.~Point_2();
//	return minimalDistanceToObstacle;
//}

double BialkowskiCollisionChecker::calcMinimumDistanceToObstacles(Node * sample) {
	double minimalDistanceToObstacle = std::numeric_limits<double>::max();
	vector<StaticObstacle*> obstacles = ne->getStaticObstacles();

	int count = 0;
	for (int i = 0; i < obstacles.size(); i++) {
		//vector<Cell*> cells = obstacles[i]->getObstacleCells();
		//for (int j = 0; j < cells.size(); j++) {
			//for (EdgeIterator edgeIt = cells[j]->getShape()->edges_begin(); edgeIt < cells[j]->getShape()->edges_end(); edgeIt++) {
		for (EdgeIterator edgeIt = obstacles[i]->getShape()->edges_begin(); edgeIt <  obstacles[i]->getShape()->edges_end(); edgeIt++) {
				Segment_2 edge = *edgeIt;
				Point_2 samplePoint = Point_2{ sample->getX(), sample->getY() };
				//double squareDistance = CGAL::squared_distance(edge, samplePoint).approx().inf();
				double squareDistance = CGAL::squared_distance(edge, samplePoint);
				double distance = sqrt(squareDistance);
				if (distance < minimalDistanceToObstacle) {
					minimalDistanceToObstacle = distance;
				}
				//samplePoint.~Point_2();
				count++;
			//}
		}
	}

	return minimalDistanceToObstacle;
}

bool BialkowskiCollisionChecker::testFreeSegment(Node* initNode, Node* endNode) {
	//return true;
	//Cell* celllInit = this->ne->getCellByXYCoordinate(endNode->getX(), endNode->getY());
	////int value = this->ne->getValueFromBinaryGrid(celllInit->getRowIndex(), celllInit->getColumnIndex());
	//if (celllInit->getBinaryGridValue() == 1) {
	//	return false;
	//}
	
	this->collisionsDetected++;
	if (!((initNode->getX() <= ne->getInitialCoordinate()->getX() || endNode->getX() <= ne->getInitialCoordinate()->getX()) ||
		(initNode->getX() >= ne->getFinalCoordinate()->getX() || endNode->getX() >= ne->getFinalCoordinate()->getX()) ||
		(initNode->getY() <= ne->getInitialCoordinate()->getY() || endNode->getY() <= ne->getInitialCoordinate()->getY()) ||
		(initNode->getY() >= ne->getFinalCoordinate()->getY() || endNode->getY() >= ne->getFinalCoordinate()->getY()))){
	//	double xInit, yInit, xEnd, yEnd;
		Point_2 *begin = new Point_2(initNode->getX(), initNode->getY());
		Point_2 *end = new Point_2(endNode->getX(), endNode->getY());
		Segment_2* segmentTest = new Segment_2(*begin, *end);

		//Verifica colisão com obstáculos estáticos
		vector<StaticObstacle*> obstacles = this->ne->getStaticObstacles();
		for (int i = 0; i < obstacles.size(); i++) {
			if (obstacles[i]->intersection(*segmentTest) == true) {
				obstacles.clear();
				return false;
			}
		}

		delete begin;
		delete end;
		delete segmentTest;

		//obstacles.clear();
		//segmentTest.~Segment_2();
		return true;
	}
	return false;
}

int BialkowskiCollisionChecker::getCollisionsDetected() {
	return this->collisionsDetected;
}