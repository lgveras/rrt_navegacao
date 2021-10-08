#include "StaticObstacle.h"
#include "Node.h"
#include "MathUtils.h"

//
//typedef CGAL::Polygon_with_holes_2<CGALKernel> Polygon_with_holes_2;
//typedef CGALKernel::Segment_2 Segment_2;
//typedef CGALKernel::Intersect_2 Intersect_2;
//typedef Polygon_2::Edge_const_iterator Edge_const_iterator;

StaticObstacle::StaticObstacle(vector<Cell*> obstacleCells, Polygon_2 obstacleShape, Node* startLimit, Node* endLimit) {
	Polygon_2 result;
	Polygon_2::Vertex_circulator circulator = obstacleShape.vertices_circulator();
	Polygon_2::Vertex_iterator begin = obstacleShape.vertices_begin();
	environmentLimits[0] = startLimit;
	environmentLimits[1] = endLimit;

	int count = 0;
	/*cout << "Orientation: " << obstacleShape.orientation() << endl;
	cout << "Quantidade vértices" << obstacleShape.size() << endl;*/
	double countv = 0;
	while (count < obstacleShape.size()+1) {
		//Para determinar se um vértice é convexo, foi utilizado produto vetorial
		if (count >= 1) {
			Point_2 v1 = *(circulator-1);
			Point_2 v2 = *(circulator);
			Point_2 v3 = *(circulator + 1);

			//Calcular o determinante da matrix formada por v1, v2 e v3 define se a curva feita pelas arestas é para a direita ou esquerda
			//Se a geometria é antihorária, então uma curva para a esquerda define um vértice convexo (det > 0)
			//Se a geometria é horário, então uma curva para a direita define um vértice convexo (det < 0)
			double det = (v2.x() - v1.x()) * (v3.y() - v2.y()) - (v3.x() - v2.x())*(v2.y() - v1.y());

			if(det > 0){
				if (environmentLimits[0] != nullptr) {
					if (v2.x() != environmentLimits[0]->getX() && v2.y() != environmentLimits[0]->getY()
						&& v2.x() != environmentLimits[1]->getX() && v2.y() != environmentLimits[1]->getY()) {
						this->convexVertices.push_back(*(circulator));
						countv++;
					}
				} else {
					this->convexVertices.push_back(*(circulator));
					countv++;
				}
			}

			//põe todo mundo na origem e rotaciona
			//double x2 = v2.x() - v1.x(); 
			////x2 = v2.x() * cos(omega);
			//double y2 = v2.y() - v1.y();  
			//v2.y() * sin(omega);

		/*	double x3 = v3.x() - v1.x() * cos(omega);*/
			//double y3 = v3.y() - v2.y() * sin(omega);

		/*	double x2 = v2.x() - v2.x() * cos(omega);
			double y2 = v2.y() - v2.y() * sin(omega);

			double x3 = v3.x() - v3.x() * cos(omega);
			double y3 = v3.y() - v3.y() * sin(omega);*/

		/*	double modulo_vetorB_linha = MathUtils::euclidianDistance(x2, y2, x3, y3);
			cout << "vetor v2 linha para v3 linha: " << modulo_vetorB_linha << endl;

			double sin_theta = (y3) / modulo_vetorB_linha;
			double sin = asin(sin_theta);
			if (sin > 3.14) {
				this->convexVertices.push_back(*(circulator));
			}*/
			//cout << "v1 - x: " << v1.x() << " y: " << v1.y() << endl;
			//cout << "v2 - x: " << v2.x() << " y: " << v2.y() << endl;
			//cout << "v3 - x: " << v3.x() << " y: " << v3.y() << endl;

			//double modulo_vetorA = MathUtils::euclidianDistancePoint(&v1, &v2);
			//double modulo_vetorB = MathUtils::euclidianDistancePoint(&v1, &v3);

			//cout << "vetor v1 para v2: " << modulo_vetorA << endl;
			//cout << "vetor v2 para v3: " << modulo_vetorB << endl;

			//Point_2 v2_linha(v2.x() - v1.x(), v2.y() - v1.y());
			//Point_2 v3_linha(v3.x() - v1.x(), v3.y() - v1.y());

			//cout << "v2' - x: " << v2_linha.x() << " y: " << v2_linha.y() << endl;
			//cout << "v3' - x: " << v3_linha.x() << " y: " << v3_linha.y() << endl;

			////double internProductAB = (v2.x()-v1.x())*(v3.x() - v2.x()) + (v2.y() - v1.y())*(v3.y() - v2.y());
			//double internProductAB = (v2_linha.x())*(v3_linha.x()) + (v2_linha.y())*(v3_linha.y());
			//cout << "Produto interno: " << internProductAB << endl;

			//double cosValue = internProductAB / (modulo_vetorA*modulo_vetorB);
			//double angle = acos(cosValue);
			//cout << "Angulo: " << angle << endl;
			//cout << "Cosseno: " << cosValue << endl;

			//double sinValue = sin(angle);
			//cout << "Seno: " << sinValue << endl;

			//double vetorialProductAB = modulo_vetorA*modulo_vetorB*sinValue;

			//cout << "Produto vetorial: " << vetorialProductAB << endl;

			//if (vetorialProductAB > 0) {
			//	this->convexVertices.push_back(*(circulator));
			//}
		}

		count++;
		circulator++;
	}
	//cout << "Quantidade vertices convexos: " << countv << endl;
	//for (circulator; circulator <= obstacleShape.vertices_begin() + 1; circulator++){
		
	//int timesIdentified;
	//for (int i = 0; i < obstacleCells.size(); i++) {
	//	for (int j = 0; j < obstacleCells[i]->getShape()->size(); j++) {
	//		Point_2 vertex = (*obstacleCells[i]->getShape()).vertex(j);
	//		timesIdentified = 0;
	//		//TODO: Fazer por vizinhança. Criar ponteiros para os vizinhos de cada célula.
	//		for (int k = 0; k < obstacleCells.size(); k++) {
	//			if (i != k) {
	//				for (int w = 0; w < obstacleCells[k]->getShape()->size(); w++) {
	//					Point_2 vertexToCompare = (*obstacleCells[k]->getShape()).vertex(w);
	//					if (vertex.x() == vertexToCompare.x() &&
	//						vertex.y() == vertexToCompare.y()) {
	//						timesIdentified++;
	//					}
	//				}
	//			}
	//		}

	//		//se não identificou nenhuma vez, é porque é um vértice na ponta do obstáculo.
	//		/*se identificou 2 vezes, é porque o vértice está intersecionado por 2 células. em uma estrutura de grade
	//		isso significa que o vértice está em uma dobra do obstáculo.*/
	//		if (timesIdentified == 0 || timesIdentified == 2) {
	//			bool isInResult = false;
	//			for (int t = 0; t < result.size(); t++) {
	//				if (result[t].x() == vertex.x() && result[t].y() == vertex.y()) {
	//					isInResult = true;
	//				}
	//			}

	//			if (!isInResult) {
	//				result.push_back(vertex);
	//				if (timesIdentified==0) {
	//					/*std::shared_ptr<Point_2*> p(new Point_2{ to_double(obstacleCells[i]->getShape()->vertex(j).x()),
	//											  to_double(obstacleCells[i]->getShape()->vertex(j).y()) });*/
	//				/*
	//					this->convexVertices.push_back(std::make_shared<Point_2>(to_double(obstacleCells[i]->getShape()->vertex(j).x()),
	//																			 to_double(obstacleCells[i]->getShape()->vertex(j).y()))
	//												 );*/
	//					this->convexVertices.push_back(vertex);
	//				}
	//			}
	//		}
	//	}
	//}

	this->shape = obstacleShape;
	this->obstacleCells = obstacleCells;
}

StaticObstacle::StaticObstacle(vector<Cell*> obstacleCells, Polygon_2 obstacleShape) {
	Polygon_2::Vertex_circulator circulator = obstacleShape.vertices_circulator();

	int count = 0;
	/*cout << "Orientation: " << obstacleShape.orientation() << endl;
	cout << "Quantidade vértices" << obstacleShape.size() << endl;*/
	double countv = 0;
	while (count < obstacleShape.size() + 1) {
		//Para determinar se um vértice é convexo, foi utilizado produto vetorial
		if (count >= 1) {
			Point_2 v1 = *(circulator - 1);
			Point_2 v2 = *(circulator);
			Point_2 v3 = *(circulator + 1);

			//Calcular o determinante da matrix formada por v1, v2 e v3 define se a curva feita pelas arestas é para a direita ou esquerda
			//Se a geometria é antihorária, então uma curva para a esquerda define um vértice convexo (det > 0)
			//Se a geometria é horário, então uma curva para a direita define um vértice convexo (det < 0)
			double det = (v2.x() - v1.x()) * (v3.y() - v2.y()) - (v3.x() - v2.x())*(v2.y() - v1.y());

			if (det > 0) {			
				this->convexVertices.push_back(*(circulator));
				countv++;			
			}
		}

		count++;
		circulator++;
	}

	this->shape = obstacleShape;
	this->obstacleCells = obstacleCells;
}

StaticObstacle::~StaticObstacle() {
}

Polygon_2* StaticObstacle::getShape() {
	Polygon_2* shape = &(this->shape);
	return shape;
}

Polygon_2 * StaticObstacle::getOffSet(){
	Polygon_2* offSet = &(this->offSet);
	return offSet;
}

vector<Cell*> StaticObstacle::getObstacleCells() {
	return this->obstacleCells;
}

vector<Point_2> StaticObstacle::getConvexVertices() {
	return this->convexVertices;
}

vector<Point_2> StaticObstacle::getOffSetConvexVertices(){
	vector<Point_2> offSetConvexVertices;
	Polygon_2* offset = this->getOffSet();
	if(offset->size() != 0 && offset->is_clockwise_oriented()) offset->reverse_orientation();
	Polygon_2::Vertex_circulator circulator = offset->vertices_circulator();
	Polygon_2::Vertex_iterator begin = offset->vertices_begin();

	int count = 0;
	/*cout << "Orientation: " << obstacleShape.orientation() << endl;
	cout << "Quantidade vértices" << obstacleShape.size() << endl;*/
	double countv = 0;
	while (count <  offset->size() + 1) {
		//Para determinar se um vértice é convexo, foi utilizado produto vetorial
		if (count >= 1) {
			Point_2 v1 = *(circulator - 1);
			Point_2 v2 = *(circulator);
			Point_2 v3 = *(circulator + 1);

			//Calcular o determinante da matrix formada por v1, v2 e v3 define se a curva feita pelas arestas é para a direita ou esquerda
			//Se a geometria é antihorária, então uma curva para a esquerda define um vértice convexo (det > 0)
			//Se a geometria é horário, então uma curva para a direita define um vértice convexo (det < 0)
			double det = (v2.x() - v1.x()) * (v3.y() - v2.y()) - (v3.x() - v2.x())*(v2.y() - v1.y());

			if (det > 0) {
				offSetConvexVertices.push_back(*(circulator));
				countv++;
			}
		}

		count++;
		circulator++;
	}

	return offSetConvexVertices;
}

void StaticObstacle::generateOffSet(double d){
	vector<boost::shared_ptr<Polygon_2>> polygons = CGAL::create_exterior_skeleton_and_offset_polygons_2(d, this->shape);
	this->offSet = *(polygons[polygons.size()-1]);
}

void StaticObstacle::setNavigationEnvironmentLimits(Node * startLimit, Node * endLimit){
	environmentLimits[0] = startLimit;
	environmentLimits[1] = endLimit;

}

bool StaticObstacle::intersection(Segment_2 segment) {
	EdgeIterator edgesIt;
	int  countIntersection = 0, countVerticeIntersection = 0;
	//bool intersect = false;
	/*vector<Point_2> collinearPoint;
	random_collinear_points_2(segment.start(), segment.end(), 2, std::back_inserter(collinearPoint));*/

	//double initX = to_double(segment.start().x());
	//double initY = to_double(segment.start().y());
	//double endX = to_double(segment.end().x());
	//double endY = to_double(segment.end().y());

	double initX = segment.start().x();
	double initY = segment.start().y();
	double endX = segment.end().x();
	double endY = segment.end().y();

	Segment_2 edgeObstacle;
	Polygon_2 obsShape;

	if (this->offSet.is_empty()) {
		obsShape = this->shape;
	} else {
		obsShape = this->offSet;
	}

	for (edgesIt = obsShape.edges_begin(); edgesIt < obsShape.edges_end(); edgesIt++) {
		edgeObstacle = *edgesIt;
		CGAL::cpp11::result_of<Intersect_2(Segment_2, Segment_2)>::type result = CGAL::intersection(edgeObstacle, segment);
		//CGAL::cpp11::result_of<Intersect_2(Segment_2, Segment_2)>::type result;
		bool startVerticeEqual = false, endVerticeEqual = false;
		if (result) {
			countIntersection++;

			//Test with segment start node is equal of some of the vertices of the obstacle edge
		/*	if ((to_double(edgeObstacle.start().x()) == initX &&
				to_double(edgeObstacle.start().y()) == initY) ||
				(to_double(edgeObstacle.end().x()) == initX &&
					to_double(edgeObstacle.end().y()) == initY)) {*/
			if ((edgeObstacle.start().x() == initX &&
				edgeObstacle.start().y() == initY) ||
				(edgeObstacle.end().x() == initX &&
				edgeObstacle.end().y() == initY)) {
				startVerticeEqual = true;
				countVerticeIntersection++;
			}

			//Test with segment end node is equal of some of the vertices of the obstacle edge
			//if ((to_double(edgeObstacle.start().x()) == endX &&
			//	to_double(edgeObstacle.start().y()) == endY) ||
			//	(to_double(edgeObstacle.end().x()) == endX &&
			//		to_double(edgeObstacle.end().y()) == endY)) {
			if ((edgeObstacle.start().x() == endX &&
				edgeObstacle.start().y() == endY) ||
				(edgeObstacle.end().x() == endX &&
					edgeObstacle.end().y() == endY)) {
				endVerticeEqual = true;
				countVerticeIntersection++;
			}

			//When the segment lies on one of the sides of the obstacle.
			if (startVerticeEqual && endVerticeEqual) {
				return false;
			}
		}
	}

	if (countVerticeIntersection  > 0) {
		if (countVerticeIntersection > countIntersection) {
			//if (!this->shape.has_on_bounded_side(segment.start()) && !this->shape.has_on_bounded_side(segment.end())) {
				return false;
			//}
		} else if (countVerticeIntersection < countIntersection) {
			return true;
		}else{
			if (initX > endX) {
				double tempX = initX;
				double tempY = initY;
				initX = endX;
				initY = endY;
				endX = tempX;
				endY = tempY;
			}
			//cálcula vetor b-a e multiplica por constante verificando se cai dentro do obstáculo
			double constant = 0.001;
			double initDeltaX = initX + (endX - initX)*constant;
			double initDeltaY = initY + (endY - initY)*constant;

			double endDeltaX = endX - (endX - initX)*constant;
			double endDeltaY = endY - (endY - initY)*constant;

			Point_2 initDeltaPoint = Point_2(initDeltaX, initDeltaY);
			Point_2 endDeltaPoint = Point_2(endDeltaX, endDeltaY);

			if (obsShape.has_on_bounded_side(initDeltaPoint) || obsShape.has_on_bounded_side(endDeltaPoint)) {
				return true;
			}
		}

		return false;
	}

	if (countIntersection > 0) {
		return true;
	}
	
	return false;
}


bool StaticObstacle::intersectionNode(Node * node){
	Polygon_2 obsShape;

	if (this->offSet.is_empty()) {
		obsShape = this->shape;
	} else {
		obsShape = this->offSet;
	}

	Point_2* point = new Point_2(node->getX(), node->getY());
	bool result = !obsShape.has_on_unbounded_side(*point);
	delete point;

	return	result;
}