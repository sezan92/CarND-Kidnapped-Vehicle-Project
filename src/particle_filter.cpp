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



void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  std::cout << "INFO: Initializing Particle Filter" << std::endl;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  std::default_random_engine gen;
  num_particles = 100;  // TODO: Set the number of particles
  Particle particle;
  for (int i=0; i< num_particles; i++){
    double sample_x, sample_y, sample_theta;
    
    sample_x = dist_x(gen);
    sample_y = dist_y(gen);
    sample_theta = dist_theta(gen);
    
    particle.x = sample_x;
    particle.y = sample_y;
    particle.theta = sample_theta;
    std::cout << "INFO: Sample " << i + 1 << " " << particle.x << " " << particle.y << " " << particle.theta << std::endl; 

    particles.push_back(particle);
  }

  std::cout<< "INFO: initialized Particles of length: "<< particles.size() << std::endl;
  is_initialized = true;

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
  std::cout << "INFO: prediction step" << std::endl;
  for (int i=0; i < num_particles; i++){
    double x0 = particles[i].x;
    double y0 = particles[i].y;
    double theta0 = particles[i].theta;
    double new_x = x0 + velocity / yaw_rate * (sin( theta0 + yaw_rate * delta_t) - sin(theta0));
    double new_y = y0 + velocity / yaw_rate * (cos( theta0 + yaw_rate * delta_t) - cos(theta0));
    double new_theta = theta0 + yaw_rate * delta_t ; 

    normal_distribution<double> dist_x(new_x, std_pos[0]);
    normal_distribution<double> dist_y(new_y, std_pos[1]);
    normal_distribution<double> dist_theta(new_theta, std_pos[2]);
    std::default_random_engine gen; 

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen) ; 
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

  std::cout << "INFO: Data association" << std::endl;
  double distance, min_distance;
  int min_iter_id;
  
  for (unsigned int i=0;i < observations.size(); i++){
    min_distance = 10000;
    for (unsigned int j=0; j<predicted.size(); j++){
      distance =  dist(predicted[i].x, observations[j].x, predicted[i].y, observations[j].y);
      if (distance < min_distance){
        min_distance = distance;
        min_iter_id = predicted[j].id;
            }
    }
  observations[i].id = min_iter_id;
  std::cout << "INFO: nearest landmark id for "<< observations[i].x << " "<< observations[i].y <<" is "<< observations[i].id << std::endl;
  }
  
 

}

vector<LandmarkObs> ParticleFilter::transform_observations(const vector<LandmarkObs> &observations,  double x, double y, double theta){
  vector<LandmarkObs> transformed_observations;
  for (unsigned observe_iter=0; observe_iter < observations.size(); observe_iter++){
    double x_new = cos(theta) * observations[observe_iter].x - sin(theta) * observations[observe_iter].y + x;
    double y_new = sin(theta) * observations[observe_iter].y - cos(theta) * observations[observe_iter].x + y;
    transformed_observations.push_back(LandmarkObs {observations[observe_iter].id, x_new, y_new});
  }
  return transformed_observations;
}

vector<LandmarkObs> ParticleFilter::predict_landmark(std::vector<Map::single_landmark_s> landmark_list, double x, double y, double sensor_range)
{
  vector<LandmarkObs> predicted;
  for(unsigned int map_iter=0; map_iter<landmark_list.size(); map_iter++ )
    {
      int id = landmark_list[map_iter].id_i;
      double dx = x - landmark_list[map_iter].x_f;
      double dy = y - landmark_list[map_iter].y_f;
      if(dx * dx + dy * dy <= sensor_range * sensor_range) predicted.push_back(LandmarkObs {id, dx, dy});
    }
  return predicted;

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
  vector<Map::single_landmark_s> landmark_list = map_landmarks.landmark_list;
  for(unsigned int i = 0; i< particles.size(); i++)
  {
    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;
    vector<LandmarkObs> predicted = predict_landmark(landmark_list, x, y, sensor_range);
    vector<LandmarkObs> transformed_observations = transform_observations(observations, x, y, theta);
    dataAssociation(predicted, transformed_observations);
    //TODO: update weights using gaussian distribution
  }


}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

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