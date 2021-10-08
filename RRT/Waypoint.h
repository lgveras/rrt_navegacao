/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Waypoint.h
 * Author: LuizGustavo
 *
 * Created on 8 de Agosto de 2016, 15:04
 */

#ifndef WAYPOINT_H
#define WAYPOINT_H

//TODO: Criar uma superclasse para representar um ponto e fazer
//o waypoint 
class Waypoint {
public:
    Waypoint();
    Waypoint(const Waypoint& orig);
    Waypoint(double latitude, double longitude);
    virtual ~Waypoint();
    const double & getLatitude() const;
    void setLatitude(const double &lat);
    const double & getLongitude() const;
    void setLongitude(const double &lon);
private:
    char nome[20];
    double lAlt; //pés
    double latitude;
    double longitude;
    int bSolo; //no chao
    double dVel;
    int bEndPoint;
    int l;
    int c;
};

#endif /* WAYPOINT_H */