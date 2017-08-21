#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "classifier.h"

/**
 * Initializes GNB
 */
GNB::GNB() {

}

GNB::~GNB() {}

void GNB::train(vector<vector<double>> data, vector<string> labels)
{

	/*
		Trains the classifier with N data points and labels.

		INPUTS
		data - array of N observations
		  - Each observation is a tuple with 4 values: s, d,
		    s_dot and d_dot.
		  - Example : [
			  	[3.5, 0.1, 5.9, -0.02],
			  	[8.0, -0.3, 3.0, 2.2],
			  	...
		  	]

		labels - array of N labels
		  - Each label is one of "left", "keep", or "right".
	*/
  // P(A|B) = P(A) *  P(B|A) / P(B)
  int left_count = 0;
  int keep_count = 0;
  int right_count = 0;

  for(int i=0; i < data.size(); i++) {
    if(labels[i] == "left") left_count++;
    if(labels[i] == "keep") keep_count++;
    if(labels[i] == "right") right_count++;
    // But d % lane_width might be helpful since it gives the relative position of a vehicle
    // in it's lane regardless of which lane the vehicle is in
//    data[i][1] = fmod(data[i][1], 4);  // this actually made it worse
  }
  label_prob[0] = left_count / data.size();
  label_prob[1] = keep_count / data.size();
  label_prob[2] = right_count / data.size();
  std::cout << "Left | Keep | Right -- " << left_count << " | " << keep_count << " | " << right_count << std::endl;

  //accumlate the mean
  for(int i=0; i < data.size(); i++) {
    if(labels[i] == "left") {
      for(int j=0; j<4; j++) {
	  left_mean[j] += data[i][j]/left_count;
      }
    }
    if(labels[i] == "keep") {
      for(int j=0; j<4; j++) {
	  keep_mean[j] += data[i][j]/keep_count;
      }
    }
    if(labels[i] == "right") {
      for(int j=0; j<4; j++) {
	  right_mean[j] += data[i][j]/right_count;
      }
    }
  }
  //calc the variance
  for(int i=0; i < data.size(); i++) {
    if(labels[i] == "left") {
      for(int j=0; j<4; j++) {
	left_variance[j] += pow(data[i][j]-left_mean[j], 2)/left_count;
      }
    }
    if(labels[i] == "keep") {
      for(int j=0; j<4; j++) {
	  keep_variance[j] += pow(data[i][j]-keep_mean[j], 2)/left_count;
      }
    }
    if(labels[i] == "right") {
      for(int j=0; j<4; j++) {
	  right_variance[j] += pow(data[i][j]-right_mean[j], 2)/left_count;
      }
    }
  }

  cout << "test output:";
  for(auto& item: right_variance) {
      cout << item << ", ";
  }
  cout << endl;


}

double gaussianProb(double obs, double mean, double variance) {
  return 1 / sqrt(2*M_PI*variance) * exp(-pow(obs-mean, 2) / (2*variance));
}

string GNB::predict(vector<double> sample)
{
	/*
		Once trained, this method is called and expected to return
		a predicted behavior for the given observation.

		INPUTS

		observation - a 4 tuple with s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
		# TODO - complete this
	*/
  // P(left|X) = P(left) * P(X|left) / P(X)
  // Since P(X) is a constant we're just dropping it for convenience, as we're only interested in the highest value
  // P(left|X) = P(left) * P(X|left)
  // P(left|X) = P(left) * P(s|left) * P(d|left) * P(s_dot|left) * P(d_dot|left)
  // where each P(x|left) is given by function gaussianProb()
  double PXleft = 1;
  double PXkeep = 1;
  double PXright = 1;
  for(int i=0; i<4; i++) {
//    double PXileft = 1 / sqrt(2*M_PI*left_variance[i]) * exp(-pow(sample[i]-left_mean[i], 2) / (2*left_variance[i]));
    double PXileft = gaussianProb(sample[i], left_mean[i], left_variance[i]);
    PXleft *= PXileft;
    double PXikeep = gaussianProb(sample[i], keep_mean[i], keep_variance[i]);
    PXkeep *= PXikeep;
    double PXiright = gaussianProb(sample[i], right_mean[i], right_variance[i]);
    PXright *= PXiright;
  }



//  std::cout << PXleft << " | " << PXkeep << " | " << PXright << std::endl;
  if(PXleft > PXkeep && PXleft > PXright){
      return this->possible_labels[0];
  }
  if(PXkeep > PXleft && PXkeep > PXright){
      return this->possible_labels[1];
  }
  if(PXright > PXleft && PXright > PXkeep){
      return this->possible_labels[2];
  }
  std::cout << "this should rarely happen" << std::endl;
  return this->possible_labels[1];

}
