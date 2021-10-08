/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Node.cpp
 * Author: LuizGustavo
 * 
 * Created on 8 de Agosto de 2016, 14:03
 */

#include "Node.h"
#include <cstddef>

Node::Node() {
     this->nodeWaypoint = new Waypoint{};
     this->antecessor = nullptr;
}

Node::Node(double latitude, double longitude) {
    this->nodeWaypoint = new Waypoint{latitude, longitude};
    this->antecessor = nullptr;
}

Node::Node(double latitude, double longitude, G2PlanarPH* curve){
	this->nodeWaypoint = new Waypoint{ latitude, longitude };
	this->antecessor = nullptr;
	this->curve = curve;
}

Node::Node(const Node& orig) {
}

Node::~Node() {
	delete this->nodeWaypoint;
}

Node* Node::getAntecessor(){
	if(this == nullptr){
		return nullptr;
	}else{
		return this->antecessor;
	}
}

void Node::setAntecessor(Node* node){
    this->antecessor = node;
}

Waypoint* Node::getNodeWaypoint(){
    return nodeWaypoint;
}

const vector<Node*>& Node::getSucessors() const{
    return this->sucessores;
}

void Node::addSucessor(Node* newSucessor){
    this->sucessores.push_back(newSucessor);
}

bool Node::isSucessorNode(Node* potentialSucessor) {
	for (int i = 0; i < this->sucessores.size(); i++) {
		if (potentialSucessor->getX() == this->sucessores[i]->getX() && potentialSucessor->getY() == this->sucessores[i]->getY()) {
			return true;
		}
	}
	
	return false;
}

bool Node::isAntecessor(Node* potentialAntecessor) {
	if (this->antecessor != nullptr) {
		if (potentialAntecessor->getX() == this->antecessor->getX() && potentialAntecessor->getY() == this->antecessor->getY()) {
			return true;
		}
	}
	return false;
}

bool Node::isInRoute(Node* potentialRouteMember) {
	Node* currentNode = this->antecessor;
	while (currentNode != nullptr) {
		if (currentNode->getX() == potentialRouteMember->getX() && 
			currentNode->getY() == potentialRouteMember->getY()) {
			return true;
		}
		currentNode = currentNode->getAntecessor();
	}

	return false;
}

G2PlanarPH * Node::getCurve(){
	return this->curve;
}

short int Node::removeSucessor(Node * sucessorToRemove){
	for (int i = 0; i < this->sucessores.size(); i++) {
		if (this->sucessores[i]->getX() == sucessorToRemove->getX() && this->sucessores[i]->getY() == sucessorToRemove->getY()) {
			this->sucessores.erase(this->sucessores.begin() + i);
			return 1;
		}
	}
	//0 significa que nada foi removido;
	return 0;
}

Node* Node::createCopy(){
	return new Node(this->getX(), this->getY());
}

Node* Node::getReferenceCurveNode(){
	return this->referenceCurveNode;
}

void Node::setReferenceCurveNode(Node* referenceNode){
	this->referenceCurveNode = referenceNode;
}

 bool Node::isVisited(){
     return this->visited;
 }
 
 void Node::visite(){
     this->visited = true;
 }

 double Node::getX() const{
	 return this->nodeWaypoint->getLatitude();
 }

 double Node::getY() const{
	 return this->nodeWaypoint->getLongitude();
 }

 inline bool Node::operator==(const Node & nodeToCompare) {
	 return this->getX() == nodeToCompare.getX() && this->getY() == nodeToCompare.getY();
 }

 void Node::incrementTestsValue(int increment) {
	 this->testsValue += increment;
 }

 int Node::getTestsValue() {
	 return this->testsValue;
 }

 void Node::setStartPosition(bool value) {
	 this->startPosition = true;
 }

 bool Node::isStartPosition() {
	 return this->startPosition;
 }
