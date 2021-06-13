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
#include <cstdlib>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

#define EPS 0.00001
void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  //std::cout << "INFO: Initializing Particle Filter" << std::endl;
  std::default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  num_particles = 100;  // TODO: Set the number of particles
  Particle particle;
  for (int i=0; i< num_particles; i++){
    double sample_x, sample_y, sample_theta;
    
    sample_x = dist_x(gen);
    sample_y = dist_y(gen);
    sample_theta = dist_theta(gen);
    particle.id = i ;
    particle.x = sample_x;
    particle.y = sample_y;
    particle.theta = sample_theta;
    particle.weight = 1;
    weights.push_back(particle.weight);
    //std::cout << "INFO: Sample " << i + 1 << " " << particle.x << " " << particle.y << " " << particle.theta << std::endl; 

    particles.push_back(particle);
  }

  //std::cout<< "INFO: initialized Particles of length: "<< num_particles << std::endl;
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
  //std::cout << "INFO: prediction step" << std::endl;
  std::default_random_engine gen;
  for (int i=0; i < num_particles; i++){
    double x0 = particles[i].x;
    double y0 = particles[i].y;
    double theta0 = particles[i].theta;
    double new_x = x0 + velocity / yaw_rate * (sin( theta0 + yaw_rate * delta_t) - sin(theta0));
    double new_y = y0 + velocity / yaw_rate * (-cos( theta0 + yaw_rate * delta_t) + cos(theta0));
    double new_theta = theta0 + yaw_rate * delta_t ; 

    normal_distribution<double> dist_x(new_x, std_pos[0]);
    normal_distribution<double> dist_y(new_y, std_pos[1]);
    normal_distribution<double> dist_theta(new_theta, std_pos[2]);
     

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

  //std::cout << "INFO: Data association" << std::endl;
  double distance, min_distance;
  int min_iter_id = -1;
  
  for (unsigned int i=0;i < observations.size(); i++){
    min_distance = __DBL_MAX__;
;
    for (unsigned int j=0; j<predicted.size(); j++){
      distance =  dist(predicted[i].x, predicted[i].y, observations[j].x, observations[j].y);
      if (distance < min_distance){
        min_distance = distance;
        min_iter_id = predicted[j].id;
        //std::cout<<"INFO:got minimum distance at "<< min_iter_id <<std::endl;  
            }
    observations[i].id = min_iter_id;
      //std::cout << "INFO: nearest landmark id for "<< observations[i].x << " "<< observations[i].y <<" is "<< observations[i].id << std::endl;  
    }
    
  
 
  }
  
 

}

vector<LandmarkObs> ParticleFilter::transform_observations(const vector<LandmarkObs> &observations,  Particle particle){
  double x = particle.x;
  double y = particle.y;
  double theta = particle.theta;
  vector<LandmarkObs> transformed_observations;
  for (unsigned observe_iter=0; observe_iter < observations.size(); observe_iter++){
    double x_new = cos(theta) * observations[observe_iter].x - sin(theta) * observations[observe_iter].y + x;
    double y_new = sin(theta) * observations[observe_iter].x + cos(theta) * observations[observe_iter].y + y;
    transformed_observations.push_back(LandmarkObs {observations[observe_iter].id, x_new, y_new});
  }
  return transformed_observations;
}

vector<LandmarkObs> ParticleFilter::predict_landmark(std::vector<Map::single_landmark_s> landmark_list, Particle particle, double sensor_range)
{
  vector<LandmarkObs> predicted;
  double x = particle.x;
  double y = particle.y;
  
  for(unsigned int map_iter=0; map_iter<landmark_list.size(); map_iter++ )
    {
      Map::single_landmark_s landmark = landmark_list[map_iter];
      int id = landmark.id_i;
      double dx = abs(x - landmark.x_f);
      double dy = abs(y - landmark.y_f);
      if(dx <= sensor_range && dy <= sensor_range) predicted.push_back(LandmarkObs {id, landmark.x_f, landmark.y_f});
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
  double weight_sum =0;
  for(int i = 0; i< num_particles; i++)
    {
      vector<LandmarkObs> transformed_observations = transform_observations(observations, particles[i]);
      vector<LandmarkObs> predicted = predict_landmark(landmark_list, particles[i], sensor_range);      
      dataAssociation(predicted, transformed_observations);
      particles[i].weight=1.0;
      
      double dx = 0;
      double dy = 0; 
      for (unsigned int j=0; j< transformed_observations.size(); j++)
      {
        unsigned int k = 0;
        bool found = false;
      
        while ( !found && k<predicted.size() ){
           
          if (predicted[k].id == transformed_observations[j].id){
            found = true;
            //std::cout << "INFO: found the nearest landmark at id " << transformed_observations[j].id <<std::endl;
          
        dx = predicted[k].x - transformed_observations[j].x;
        dy = predicted[k].y - transformed_observations[j].y;
          }
        
        


      k++;
      }
        double weight = (1/(pow(sqrt(2 * M_PI * pow(std_landmark[0], 2) * pow(std_landmark[1], 2)), 2) )) * exp(-pow(dx, 2)/ (2 * pow(std_landmark[0], 2)) + (-pow(dy, 2)/ (2 * pow(std_landmark[1], 2))));
        if (weight == 0)
        {
          particles[i].weight *= EPS;
        }
        else
        {
          particles[i].weight *= weight;
        
        }

      weight_sum += weight;
      
    
    //std::cout<<"particle weight"<<particles[i].weight<<std::endl;
      }
    }
  for(int i =0; i<num_particles; i++){
    particles[i].weight /= weight_sum;
    weights[i] = particles[i].weight;
  }

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(), weights.end());
  
  std::vector<Particle> old_particles = particles;
  for (int i = 0; i< num_particles; i++){
    unsigned int idx = d(gen);
    //std::cout<<"INFO: sampled particle of index: "<<idx<<"with weight: "<< weights[idx] <<std::endl;
    particles[i] = old_particles[idx];


  }
  
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