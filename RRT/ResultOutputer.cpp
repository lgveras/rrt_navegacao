#include "ResultOutputer.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <iomanip>

using namespace std;

ResultOutputer::ResultOutputer(string name, string *columnsBegin, string *columnsEnd, char separator){
	this->columnNames = columnsBegin;
	int count = 0;
	while (columnsBegin!=columnsEnd) {
		columnsBegin++;
		count++;
	}
	this->fileName = name;
	this->qtdColumns = count;
	this->separator = separator;

	struct stat buffer;
	if (stat(name.c_str(), &buffer) != 0) {
		try {
			outputFile.exceptions(ifstream::failbit);
			outputFile.open(name, ofstream::out);
			for (int i = 0; i <= this->qtdColumns; i++) {
				outputFile << this->columnNames[i];
				if (i + 1 <= this->qtdColumns) {
					outputFile << separator;
				}
			}
			outputFile << endl;
		} catch (const ifstream::failure& e) {
			cout << "Exception opening file: " << e.what();
		}
		outputFile.close();
	}
}

ResultOutputer::~ResultOutputer(){
	this->closeFile();
}

bool ResultOutputer::pushData(float * tupleBegin, float * tupleEnd) {
	if (!outputFile.is_open()) {
		outputFile.open(fileName, fstream::out | ofstream::app);
	}

	for (float* i = tupleBegin; i <= tupleEnd; i++){
		outputFile << std::setprecision(6) << *i;
		if (i + 1 <= tupleEnd) {
			outputFile << separator;
		}
	}
	outputFile << endl;	
	//outputFile.close();
	return true;
}

void ResultOutputer::closeFile() {
	if (outputFile.is_open()){
		outputFile.close();
	}
}