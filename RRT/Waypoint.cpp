/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Waypoint.cpp
 * Author: LuizGustavo
 * 
 * Created on 8 de Agosto de 2016, 15:04
 */

#include "Waypoint.h"

Waypoint::Waypoint() {
    this->latitude = 0;
    this->longitude = 0;
}

Waypoint::Waypoint(const Waypoint& orig) {
    this->latitude = 0;
    this->longitude = 0;
}

Waypoint::Waypoint(double latitude, double longitude){
    this->latitude = latitude;
    this->longitude = longitude;
}

Waypoint::~Waypoint() {
}

const double & Waypoint::getLatitude() const{
    return this->latitude;
}

void Waypoint::setLatitude(const double &lat){
    latitude = lat;
}

const double & Waypoint::getLongitude() const{
    return this->longitude;
}

void Waypoint::setLongitude(const double &lon){
    longitude = lon;
}