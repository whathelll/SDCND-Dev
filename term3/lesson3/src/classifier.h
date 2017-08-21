#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class GNB {
public:

	vector<string> possible_labels = {"left","keep","right"};

	vector<double> label_prob = vector<double>(3);

	vector<double> left_sum = vector<double>(4);
	vector<double> left_mean = vector<double>(4);
	vector<double> left_variance = vector<double>(4);
	vector<double> left_stddev = vector<double>(4);

	vector<double> keep_mean = vector<double>(4);
	vector<double> keep_variance = vector<double>(4);
	vector<double> keep_stddev = vector<double>(4);

	vector<double> right_mean = vector<double>(4);
	vector<double> right_variance = vector<double>(4);
	vector<double> right_stddev = vector<double>(4);

	/**
  	* Constructor
  	*/
 	GNB();

	/**
 	* Destructor
 	*/
 	virtual ~GNB();

 	void train(vector<vector<double> > data, vector<string>  labels);

  	string predict(vector<double>);

};

#endif



