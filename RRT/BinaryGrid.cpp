/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GradeBinaria.cpp
 * Author: LuizGustavo
 * 
 * Created on 8 de Agosto de 2016, 10:24
 */

#include "BinaryGrid.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

BinaryGrid::BinaryGrid() {
}

BinaryGrid::~BinaryGrid() {
}

void BinaryGrid::GravacaoGradeBinaria( int n ){
//  FILE       *arq;
//  int        lin, col;
//
//  arq = fopen( "grade_binaria.txt", "w+" );
//
//  for( lin = 0; lin < n; lin++ )
//  {
//      for( col = 0; col < n; col++ )
//      {
//        //fprintf( arq, "%d ", BinaryGrid[((n-1)-lin)][col]);
//      }
//      fprintf(arq,"\n");
//  }
//
//  fclose(arq);
}

void BinaryGrid::Exibir_Grade_Binaria()
{
//     int lin,col, nlin, ncol;
//     int BinariaM[nlin][ncol];
//     for(lin=0;lin<nlin;lin++)
//     {
//         for(col=0;col<ncol;col++)
//         {
//             printf("%d ",BinariaM[nlin-1-(lin)][col]);
//         }
//         printf("\n");
//     }
//
//     getchar();
//     getchar();
}

void BinaryGrid::GerarArquivodaMatriz( void )
{
//  int lin, col;
//  FILE *arq1; 
//  int nlin, ncol;
//  int BinariaM[nlin][ncol];
//  
//  arq1 = fopen( "matrizbinaria.txt", "w+" );
//
//  for( lin = 0; lin < nlin; lin++ )
//  {
//    for( col = 0; col < ncol; col++ )
//    {
//      fprintf( arq1, "%d ", BinariaM[(nlin-1)-lin][col] );
//    }
//
//    fprintf( arq1, "\n" );
//  }
//
//  fclose(arq1);
}

void MontarMatriz(int no){
//  int  posl,posc;
//  int lin, col, i;  
//  int nlin, ncol;
//  int BinariaM[nlin][ncol];
//
//  //BinariaM = (int**) malloc( sizeof( int ) * nlin );
//  for( i = 0; i < nlin; i++ )
//    //BinariaM[i] = (int*) malloc( sizeof( int ) * ncol );
//
//  for( lin=0;lin<nlin;lin++ )
//    for( col=0;col<ncol;col++ )
//    {
//      BinariaM[lin][col] = 0;
//    }
//
//  for(i=0;i<no;i++)
//  {
//    posl=rand()%(nlin-1);
//    posc=rand()%(ncol-1);
//
//    BinariaM[posl][posc] = 1;
//    BinariaM[posl][posc+1] = 1;
//    BinariaM[posl+1][posc] = 1;
//    BinariaM[posl+1][posc+1] = 1;
//  }
}

void BinaryGrid::LerConfiguracoes( void ){
//  FILE *arq1;
//  arq1 = fopen( "configuracoes.txt", "r" );
//  
//  int lin,col, nlin, ncol;
//  int BinariaM[nlin][ncol];
//  char nome_arquivo_matriz_binaria[15];
//  float C1,C2;
//
//  fscanf(arq1, "%d\n", &nlin);
//  fscanf(arq1, "%d\n", &ncol);
//  fscanf(arq1, "%lf\n", &C1);
//  fscanf(arq1, "%lf\n", &C2);
//  fscanf(arq1, "%s\n",&nome_arquivo_matriz_binaria);
//  fclose(arq1);
}

int BinaryGrid::getColumnQuantity() {
	return this->columnQuantity;
}

int BinaryGrid::getRowQuantity() {
	return this->rowQuantity;
}

int BinaryGrid::getValueInMatriz(int i, int j){
	return this->matriz[i][j];
}

void BinaryGrid::binaryMatrixRead(const string name){
	//string path = __FILE__; //gets source code path, include file name
	//path = path.substr(0, 1 + path.find_last_of('\\')); //removes file name
	//path += name; //adds input file to path
	cout << "Reading binary grid ..." << endl;

	ifstream infile;
	infile.open(name, ios::in);

	if (infile.is_open()) {
		string line;
		int linesQuantity = count(istreambuf_iterator<char>(infile),
								  istreambuf_iterator<char>(), '\n') + 1;
		infile.clear(); // clear bad state after eof
		infile.seekg(0);
		int i = 0;
		string s;
		string lineNoWhitespace;

		while (getline(infile, line)) {
			stringstream lineStream(line);
			lineNoWhitespace = "";
			// lê a string ignorando os espaços em branco
			while (lineStream >> s) {
				lineNoWhitespace.append(s);
			}

			if (this->matriz == NULL) {
				const int columnsQuantity = line.size();
				this->matriz = (int**) malloc(sizeof(int**)*lineNoWhitespace.size());
				for (int t = 0; t < lineNoWhitespace.size(); t ++) {
					this->matriz[t] = (int*)malloc(sizeof(int*)*linesQuantity);
				}
			}

			for (int j = 0; j < lineNoWhitespace.size(); j++) {
				this->matriz[i][j] = lineNoWhitespace[j] - '0';
			}
			i++;
		}
		this->columnQuantity = lineNoWhitespace.size();
		this->rowQuantity = linesQuantity;
	} else {
		cout << "File not encountered!!!" << endl;
	}

	infile.close();

	cout << "Binary grid readed!!!" << endl;
}