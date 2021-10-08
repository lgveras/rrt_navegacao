#pragma once
#include <string>
#include <fstream>

using namespace std;

class ResultOutputer{
public:
	ResultOutputer(string name, string *columnsBegin, string *columnsEnd, char separator);
	~ResultOutputer();
	bool pushData(float * tupleBegin, float * tupleEnd);
	void closeFile();
private:
	ofstream outputFile;
	string* columnNames;
	char separator;
	int qtdColumns;
	string fileName;
};