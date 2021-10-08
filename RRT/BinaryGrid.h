/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GradeBinaria.h
 * Author: LuizGustavo
 *
 * Created on 8 de Agosto de 2016, 10:24
 */

#ifndef GRADEBINARIA_H
#define GRADEBINARIA_H

#include<string>

using namespace std;

class BinaryGrid {
public:
    BinaryGrid();
    virtual ~BinaryGrid();
    void GravacaoGradeBinaria( int n );
    void Exibir_Grade_Binaria();
    void GerarArquivodaMatriz( void );
    void LerConfiguracoes( void );
    void binaryMatrixRead(const string name);
	int getColumnQuantity();
	int getRowQuantity();
	int getValueInMatriz(int i, int j);
private:
	int** matriz;
	int columnQuantity, rowQuantity;
};

#endif /* UTILS_H */