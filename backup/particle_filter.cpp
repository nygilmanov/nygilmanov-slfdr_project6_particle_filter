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

using namespace std 
  
default_random_engine gen;


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  
  
  num_particles = 300;
	weights.resize(num_particles, 1.0);

	// This line creates a normal (Gaussian) distribution for x
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < num_particles; ++i) {
		
		double sample_x, sample_y, sample_theta;
		
		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_theta = dist_theta(gen);

		Particle particle;
		particle.id = i;		
		particle.x = sample_x;
		particle.y = sample_y;
		particle.theta = sample_theta;
		particle.weight = 1.0;

		particles.push_back(particle);

	}

	is_initialized = true;

	return;
  
  

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
  
 // Extracting standard deviations
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  // Creating normal distributions
  normal_distribution<double> dist_x(0, std_x);
  normal_distribution<double> dist_y(0, std_y);
  normal_distribution<double> dist_theta(0, std_theta);

  // Calculate new state.
  for (int i = 0; i < num_particles; i++) {

  	double theta = particles[i].theta;

    if ( fabs(yaw_rate) < EPS ) { // When yaw is not changing.
      particles[i].x += velocity * delta_t * cos( theta );
      particles[i].y += velocity * delta_t * sin( theta );
      // yaw continue to be the same.
    } else {
      particles[i].x += velocity / yaw_rate * ( sin( theta + yaw_rate * delta_t ) - sin( theta ) );
      particles[i].y += velocity / yaw_rate * ( cos( theta ) - cos( theta + yaw_rate * delta_t ) );
      particles[i].theta += yaw_rate * delta_t;
    }

    // Adding noise.
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
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
  unsigned int nObservations = observations.size();
  unsigned int nPredictions = predicted.size();

  for (unsigned int i = 0; i < nObservations; i++) { // For each observation

    // Initialize min distance as a really big number.
    double minDistance = numeric_limits<double>::max();

    // Initialize the found map in something not possible.
    int mapId = -1;

    for (unsigned j = 0; j < nPredictions; j++ ) { // For each predition.

      double xDistance = observations[i].x - predicted[j].x;
      double yDistance = observations[i].y - predicted[j].y;

      double distance = xDistance * xDistance + yDistance * yDistance;

      // If the "distance" is less than min, stored the id and update min.
      if ( distance < minDistance ) {
        minDistance = distance;
        mapId = predicted[j].id;
      }
    }

    // Update the observation identifier.
    observations[i].id = mapId;
  }
  

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
  double stdLandmarkRange = std_landmark[0];
  double stdLandmarkBearing = std_landmark[1];

  for (int i = 0; i < num_particles; i++) {

    double x = particles[i].x;
    double y = particles[i].y;
    double theta = particles[i].theta;
    // Find landmarks in particle's range.
    double sensor_range_2 = sensor_range * sensor_range;
    vector<LandmarkObs> inRangeLandmarks;
    for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      float landmarkX = map_landmarks.landmark_list[j].x_f;
      float landmarkY = map_landmarks.landmark_list[j].y_f;
      int id = map_landmarks.landmark_list[j].id_i;
      double dX = x - landmarkX;
      double dY = y - landmarkY;
      if ( dX*dX + dY*dY <= sensor_range_2 ) {
        inRangeLandmarks.push_back(LandmarkObs{ id, landmarkX, landmarkY });
      }
    }

    // Transform observation coordinates.
    vector<LandmarkObs> mappedObservations;
    for(unsigned int j = 0; j < observations.size(); j++) {
      double xx = cos(theta)*observations[j].x - sin(theta)*observations[j].y + x;
      double yy = sin(theta)*observations[j].x + cos(theta)*observations[j].y + y;
      mappedObservations.push_back(LandmarkObs{ observations[j].id, xx, yy });
    }

    // Observation association to landmark.
    dataAssociation(inRangeLandmarks, mappedObservations);

    // Reseting weight.
    particles[i].weight = 1.0;
    // Calculate weights.
    for(unsigned int j = 0; j < mappedObservations.size(); j++) {
      double observationX = mappedObservations[j].x;
      double observationY = mappedObservations[j].y;

      int landmarkId = mappedObservations[j].id;

      double landmarkX, landmarkY;
      unsigned int k = 0;
      unsigned int nLandmarks = inRangeLandmarks.size();
      bool found = false;
      while( !found && k < nLandmarks ) {
        if ( inRangeLandmarks[k].id == landmarkId) {
          found = true;
          landmarkX = inRangeLandmarks[k].x;
          landmarkY = inRangeLandmarks[k].y;
        }
        k++;
      }

      // Calculating weight.
      double dX = observationX - landmarkX;
      double dY = observationY - landmarkY;

      double weight = ( 1/(2*M_PI*stdLandmarkRange*stdLandmarkBearing)) * exp( -( dX*dX/(2*stdLandmarkRange*stdLandmarkRange) + (dY*dY/(2*stdLandmarkBearing*stdLandmarkBearing)) ) );
      if (weight == 0) {
        particles[i].weight *= EPS;
      } else {
        particles[i].weight *= weight;
      }
    }
  }

   

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
   
	std::discrete_distribution<int> d(weights.begin(), weights.end());
	std::vector<Particle> weighted_sample(num_particles);

	for(int i = 0; i < num_particles; ++i){
		int j = d(gen);
		weighted_sample.at(i) = particles.at(j);
	}

	particles = weighted_sample;

	return;
  
 
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