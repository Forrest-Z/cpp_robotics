
#pragma once

#include "common.h"

struct MapPoint{
	long long id;
	float x;
	float y;
};

class Mapper {

public:
	Mapper(){}
	void initialize(const string& filename);
	vector<MapPoint> data;
};

void Mapper::initialize(const string& filename) {

	ifstream in_file(filename, ifstream::in);
	if (!in_file.is_open()) {
		cerr << "Cannot open input file: " << filename << endl;
    	exit(EXIT_FAILURE);
	}

	string line;
	while(getline(in_file, line)) {
		istringstream ss(line);
		MapPoint mp;
		ss>>mp.id;
		ss>>mp.x;
		ss>>mp.y;
		data.push_back(mp);
		if (debug)
		cout << data.back().id << ": " << data.back().x << ": " << data.back().y << endl;
	}

	if (in_file.is_open()) {
		in_file.close();
	}
}