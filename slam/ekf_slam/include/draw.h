#pragma once

#include "common.h"
#include "mapper.h"
#include "Eigen/Dense"
#include "measurement_package.h"
#include "chisquare.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class Draw {

private:

map<string, string> l_settings;
map<string, string> o_settings;

void Drawellipse(const Eigen::VectorXd& X, float a, float b, string color) {

	assert(X.size() == 3);
	const int NPOINTS = 100;
	Eigen::VectorXd vec = Eigen::VectorXd::LinSpaced(NPOINTS, 0, 2 * M_PI);
	int N = vec.size();

	Eigen::MatrixXd p = Eigen::MatrixXd::Ones(2,N);
	p.row(0) = a * vec.array().cos();
	p.row(1) = b * vec.array().sin();

	// //handle the rotation and translation
	float x0 = X(0), y0 = X(1), angle = X(2);
	Eigen::MatrixXd R = Eigen::MatrixXd(2,2);
	R << cos(angle), -sin(angle), sin(angle), cos(angle);

	Eigen::MatrixXd T = Eigen::MatrixXd::Ones(2,N);
	T.row(0) = x0*T.row(0);
	T.row(1) = y0*T.row(1);
	p = R*p + T;

	//for plotting purpuse only
	vector<float> Px, Py;
	for (int i = 0; i < N; i++) {
		Px.push_back(p(0,i));
		Py.push_back(p(1,i));
	}
	plt::plot(Px, Py, color);
 }

 void  Drawprobellipse(VectorXd x, const MatrixXd& C, float alpha, string color) {
	      
	float sxx = C(0, 0), syy = C(1, 1), sxy = C(0, 1);
	float a = sqrt(0.5*(sxx+syy+sqrt(pow(sxx-syy, 2)+4*pow(sxy,2))));   // always greater
	float b = sqrt(0.5*(sxx+syy-sqrt(pow(sxx-syy, 2)+4*pow(sxy,2))));   // always smaller

	//% Scaling in order to reflect specified probability
	a = a*sqrt(chi2invtable(alpha,2));
	b = b*sqrt(chi2invtable(alpha,2));	

	if (sxx < syy) {
	 float temp = a;
	 a = b;
	 b = temp;	
	}
	
	float angle = 0;
	//% Calculate inclination (numerically stable)
	if (sxx != syy)
	  angle = 0.5*atan(2*sxy/(sxx-syy));	
	else if (sxy == 0)
	  angle = 0;     //angle doesn't matter 
	else if (sxy > 0)
	  angle =  M_PI/4;
	else if (sxy < 0)
	  angle = -M_PI/4;
	x(2) = angle;
	Drawellipse(x,a,b,color);
 }

 void DrawLandMarks(const Mapper& mapper) {
	vector<float> X, Y;
	for (auto& landmark : mapper.data) {
	X.push_back(landmark.x);
	Y.push_back(landmark.y);
	}
	plt::plot(X, Y, l_settings);
 }
public:

 Draw() {
	//landmarking drawing setting
	l_settings["color"] = "red";
	l_settings["linestyle"] = " ";
	l_settings["marker"] = "+";
	l_settings["markersize"] = "10";
	l_settings["linewidth"] = "5";
	//observation drawing settings;
	o_settings["color"] = "blue";
	o_settings["linestyle"] = " ";
	o_settings["marker"] = "o";
	o_settings["markersize"] = "10";
	o_settings["linewidth"] = "5";
 }
 
 void Plot_state(const VectorXd& mu, const MatrixXd& sigma, const Mapper& mapper, const vector<bool>&observedLandmarks, const vector<RadarReading>& Z) {
	
	//initia rob pos and landmarks
	VectorXd rob = VectorXd(3);
	rob << mu(0), mu(1), mu(2);
	Drawprobellipse(rob, sigma, 0.6, "r");
	DrawLandMarks(mapper);

	//DrawLandMarks(landmarks) with probability;
	for(int i = 0; i < observedLandmarks.size(); i++) {
	if(observedLandmarks[i]) {
	    vector<float> X, Y;
	    X.push_back(mu(2*i + 3));
	    Y.push_back(mu(2*i + 4));
	    plt::plot(X, Y, o_settings);
	    VectorXd m = VectorXd(3);
	    m << mu(2*i+3), mu(2*i+4), 0;
	    MatrixXd sig = MatrixXd(2,2);
	    sig << sigma(2*i+3,2*i+3),sigma(2*i+3, 2*i+4),sigma(2*i+4, 2*i+3),sigma(2*i+4, 2*i+4);
	    Drawprobellipse(m, sig, 0.6, "b");
	}
	}
	
	//draw observation lines
	for(int i= 0; i < Z.size(); i++) {
	vector<float> X, Y;
	X.push_back(mu(0));Y.push_back(mu(1));
	X.push_back(mu(2*Z[i].id+1));
	Y.push_back(mu(2*Z[i].id+2));
	plt::plot(X, Y, "k");
	}   

    	//drawrobot(mu(1:3), 'r', 3, 0.3, 0.3);
	Drawellipse(rob, 0.15, 0.15, "r");
  }

  void Pause() {
      plt::pause(0.01);
  }
  void Clear() {
      plt::clf();
      plt::xlim(-2,12);
      plt::ylim(-2,12);
  }
   void Show() {
	plt::show();
   }

   void Save(string path) {
   		plt::save(path);
   }
	
};
