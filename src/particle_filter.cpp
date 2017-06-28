/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  
  num_particles = 10;
  
  default_random_engine generator;
  
  normal_distribution<double> x_dist(x, std[0]);
  normal_distribution<double> y_dist(y, std[1]);
  normal_distribution<double> theta_dist(theta, std[2]);
  
  for (int i=0; i < num_particles; i++) {
    Particle p;
    p.id = i+1;
    p.x = x_dist(generator);
    p.y = y_dist(generator);
    p.theta = theta_dist(generator);
    p.weight = 1;
    weights.push_back(1);
    
    particles.push_back(p);
  }

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  cout << "5 \n";
  default_random_engine generator;
  for (int i=0; i < num_particles; i++) {
    Particle particle = particles[i];
    double x_mean, y_mean, theta_mean;
    if (yaw_rate != 0) {
      x_mean = particle.x + ((velocity/yaw_rate) * (sin(particle.theta + yaw_rate*delta_t) - sin(particle.theta)));
      
      y_mean = particle.y + ((velocity/yaw_rate) * (cos(particle.theta) - cos(particle.theta + yaw_rate*delta_t)));
      
      theta_mean = particle.theta + yaw_rate*delta_t;
      cout << "ABC \n";
    }
    
    else {
      x_mean = particle.x + (velocity * cos(particle.theta) * delta_t);
      
      y_mean = particle.y + (velocity * sin(particle.theta) * delta_t);
      
      theta_mean = particle.theta;
      cout << "PQR \n";
    };
    
    normal_distribution<double> x_d(x_mean, std_pos[0]);
    normal_distribution<double> y_d(y_mean, std_pos[1]);
    normal_distribution<double> theta_d(theta_mean, std_pos[2]);
    
    particles[i].x = x_d(generator);
    particles[i].y = y_d(generator);
    particles[i].theta = theta_d(generator);
    
  };
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  
}

    void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
        std::vector<LandmarkObs> observations, Map map_landmarks) {
      // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
      //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
      // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
      //   according to the MAP'S coordinate system. You will need to transform between the two systems.
      //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
      //   The following is a good resource for the theory:
      //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
      //   and the following is a good resource for the actual equation to implement (look at equation 
      //   3.33
      //   http://planning.cs.uiuc.edu/node99.html
      
      //default_random_engine generator;
      //For each particle
      for (int i=0; i < num_particles; i++) {
        //set weight to 1. It will be updated below
        double mult = 1.0;
        double sigmax = std_landmark[0];
        double sigmay = std_landmark[1];
        double theta = particles[i].theta;
        //We need to sample theta from a random distribution?
        //normal_distribution<double> theta_d(theta, std_landmark[1]);
        
        
        // For each measurement
        for (int j=0; j < observations.size(); j++) {
          //double angle = theta_d(generator);
          double angle = theta;
          //Take x,y of the observation
          double a = observations[j].x;
          double b = observations[j].y;
          int obs_id = observations[j].id;
          //Transform coordinates to global space
          double x = particles[i].x + (a * cos(angle)) - (b * sin(angle));
          double y = particles[i].y + (a * sin(angle)) + (b * cos(angle));
          //cout << "X = " << x << " Y = " << y;
          //cout << "114 \n";
          //find corresponding landmark using this id
          
          float mu_x, mu_y;
          Map::single_landmark_s best;
          double least_dist = 1000000;
          
          //This loop picks the landmark which has lowest distance from the current measurement
          for (int k=0; k < map_landmarks.landmark_list.size(); k++) {
            //cout << "115 \n";
            Map::single_landmark_s land = map_landmarks.landmark_list[k];
            
            double distance = dist(x, y, land.x_f, land.y_f);
            
            if (distance < least_dist) {
              best = land;
              least_dist = distance;
            }
          }
          
          //cout << "    Xf = " << best.x_f << " Yf = " << best.y_f << "\n";
          
          particles[i].associations.push_back(best.id_i);
          particles[i].sense_x.push_back(best.x_f);
          particles[i].sense_y.push_back(best.y_f);
          
          // calculating the multivariate gaussian in a few smaller steps
          mu_x = (x - best.x_f);
          mu_x = (mu_x * mu_x);
          mu_y = y - best.y_f;
          mu_y = (mu_y * mu_y);
          //cout << "MU X = " << mu_x << "\n";
          //cout << "MU Y = " << mu_y << "\n";
          
          // Updating the overall weight by multiplying it to new weight
          double gaussian = (exp(-1 * (mu_x/(2*sigmax*sigmax) + mu_y/(2*sigmay*sigmay)))) / (2 * M_PI * sigmax * sigmay);
          mult = mult * gaussian;
          //cout << gaussian << "\n";
        }
        cout << "Mult = " << mult << "\n";
        particles[i].weight = mult;
        weights[i] = mult;
      }
    }

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  default_random_engine gen;
  
  discrete_distribution<int> dist(weights.begin(), weights.end());
  
  vector<Particle> resampled;
  
  for (int i=0; i < num_particles; i++) {
    resampled.push_back(particles[dist(gen)]);
  };
  
  particles = resampled;
  
  for (int i=0; i<num_particles; i++) {
    cout << "X = " << particles[i].x << "  Y = " << particles[i].y << "\n";
  }
  
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
