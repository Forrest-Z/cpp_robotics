#pragma once

#include "common.h"
#include "mapper.h"
#include "Eigen/Dense"
#include "measurement_package.h"
#include "chisquare.h"
#include "matplotlibcpp.h"
#include "tools.h"
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
 
 void DrawParticles(const std::vector<Particle>& particles) {
 	
 	std::vector<float> X;
 	std::vector<float> Y;

 	for (auto& p : particles) {
 		X.push_back(p.pose(0));
 		Y.push_back(p.pose(1));
 	}
	plt::plot(X, Y, "g."); 	

 }

 void Plot_state(const vector<Particle>& particles, const Mapper& mapper, const vector<RadarReading>& Z) {
	
	DrawLandMarks(mapper);
	DrawParticles(particles);

	int best_index = 0, max_w = 0.0;
	for (int i = 1; i < particles.size();i++) {
		if (max_w < particles[i].weight) {
		max_w = particles[i].weight;
		best_index = i;
		}
	}

	//DrawLandMarks(landmarks) with probability;
	for(int i = 0; i < particles[best_index].landmarks.size(); i++) {
		auto& landmark = particles[best_index].landmarks[i];
		if (landmark.observed) {
		    vector<float> X, Y;
		    X.push_back(landmark.mu(0));
		    Y.push_back(landmark.mu(1));
		    plt::plot(X, Y, o_settings);
	    	    VectorXd m = VectorXd(3);
	            m << landmark.mu(0), landmark.mu(1), 0;
		    Drawprobellipse(m, landmark.sigma, 0.95, "b");
		}
	}
	
	//draw observation lines

	for(int i= 0; i < Z.size(); i++) {
	   vector<float> X, Y;
	   X.push_back(particles[best_index].pose(0));
	   Y.push_back(particles[best_index].pose(1));
	   X.push_back(particles[best_index].landmarks[Z[i].id-1].mu(0));
	   Y.push_back(particles[best_index].landmarks[Z[i].id-1].mu(1));
	   plt::plot(X, Y, "k");
	}  

	//draw trajectory
	vector<float> X, Y;
	for (auto& ele : particles[best_index].history) {
		X.push_back(ele(0));
		Y.push_back(ele(1));
	}
	plt::plot(X, Y, "r");

	Drawellipse(particles[best_index].pose, 0.15, 0.15, "r");
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
