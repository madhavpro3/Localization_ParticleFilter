/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using namespace std;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  num_particles = 10;  // TODO: Set the number of particles

  std::default_random_engine gen;
  normal_distribution<double> posx(x,std[0]);
  normal_distribution<double> posy(y,std[1]);
  normal_distribution<double> ang(theta,std[2]);

  for(int pindex=1;pindex<=num_particles;++pindex){
    Particle newp;
    newp.id=pindex;
    newp.x=posx(gen);
    newp.y=posy(gen);
    newp.theta=ang(gen);
    newp.weight=1;
    particles.push_back(newp);
  }

  is_initialized=true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
   // Bicycle model
   std::default_random_engine gen;
   // cout << "Numparticles = "<< particles.size() << endl;
   for(Particle& p:particles){

     double theta_f = p.theta;
     double x_f = p.x + velocity*sin(p.theta)*delta_t;
     double y_f = p.y + velocity*cos(p.theta)*delta_t;
     if(abs(yaw_rate) > 1e-05){
       theta_f = p.theta + (yaw_rate*delta_t);
       x_f = p.x + (velocity/yaw_rate)*(sin(theta_f) - sin(p.theta));
       y_f = p.y + (velocity/yaw_rate)*(cos(p.theta) - cos(theta_f));
     }

     normal_distribution<double> posx(x_f,std_pos[0]);
     normal_distribution<double> posy(y_f,std_pos[1]);
     normal_distribution<double> ang(theta_f,std_pos[2]);

     p.x = posx(gen);
     p.y = posy(gen);
     p.theta = ang(gen);
   }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
   weights.clear();
   // int particleinx=0;
   for(Particle& p:particles){
     p.associations.clear();
     // convert observations of landmarks from vehicle coords to Map coords
     vector<LandmarkObs> v_lmark_mapcoords;
     for(LandmarkObs lmark_vehcoords:observations){
       LandmarkObs lmark_mapcoords;
       lmark_mapcoords.id=lmark_vehcoords.id;
       lmark_mapcoords.x = p.x + cos(p.theta)*lmark_vehcoords.x - sin(p.theta)*lmark_vehcoords.y;
       lmark_mapcoords.y = p.y + sin(p.theta)*lmark_vehcoords.x + cos(p.theta)*lmark_vehcoords.y;
       v_lmark_mapcoords.push_back(lmark_mapcoords);
     }

     double particleweight=1;
     // associate lmark_mapcoords with true landmarks
     for(LandmarkObs& lmark_mapcoords: v_lmark_mapcoords){

       int closestlmarkid=map_landmarks.landmark_list[0].id_i;
       double closestlmarkdist = dist(lmark_mapcoords.x,lmark_mapcoords.y,
                    map_landmarks.landmark_list[0].x_f,map_landmarks.landmark_list[0].y_f);


       // for(int i=1;i<map_landmarks.landmark_list.size();++i){
       for(int i=1;i<42;++i){

         double lmarkdist = dist(lmark_mapcoords.x,lmark_mapcoords.y,
                      map_landmarks.landmark_list[i].x_f,map_landmarks.landmark_list[i].y_f);


          if(lmarkdist < closestlmarkdist){
            closestlmarkid=map_landmarks.landmark_list[i].id_i;
            closestlmarkdist=lmarkdist;
          }
       }

       p.associations.push_back(closestlmarkid);

       double power = pow((lmark_mapcoords.x - map_landmarks.landmark_list[closestlmarkid-1].x_f)/std_landmark[0],2) +
        pow((lmark_mapcoords.y - map_landmarks.landmark_list[closestlmarkid-1].y_f)/std_landmark[1],2);
       power/=2;

       double prob = exp(-power)/(2*M_PI*std_landmark[0]*std_landmark[1]);
       particleweight*=prob;
     }

     // find weight of the particle based on the associations
     // find the prob of observations in map coords given mean is the nearest landmark poistion
      p.weight=particleweight;
      // weights[particleinx]=particleweight;
      weights.push_back(particleweight);
      // ++particleinx;
   }


}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
   // cout << "NWeights = " << weights.size() << endl;
   std::default_random_engine gen;
   std::discrete_distribution<int> distr(weights.begin(),weights.end());
   // cout << "Weights # = "<< weights.size() << endl;
   vector<Particle> resampled;
   for(int i=0;i<num_particles;++i){
     resampled.push_back(particles[distr(gen)]);
   }

   particles=resampled;
}

void ParticleFilter::SetAssociations(Particle& particle,
                                     const vector<int>& associations,
                                     const vector<double>& sense_x,
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
