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
	
  num_particles = 10;
  
  random_device rand;
  default_random_engine generator(rand());
  
  // Create normal distributions centered around initial measurements
  normal_distribution<double> x_dist(x, std[0]);
  normal_distribution<double> y_dist(y, std[1]);
  normal_distribution<double> theta_dist(theta, std[2]);
  
  cout << "Initial X = " << x << "  Y = " << y << "\n";
  
  // Generate particles from normal distributions created above
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
  
  is_initialized = true;

}


// Move each particle to update its position
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	
  default_random_engine generator;
  
  for (int i=0; i < num_particles; i++) {
    Particle particle = particles[i];
    double x_mean, y_mean, theta_mean;

    if (yaw_rate != 0) {
      x_mean = particle.x + ((velocity/yaw_rate) * (sin(particle.theta + yaw_rate*delta_t) - sin(particle.theta)));
      
      y_mean = particle.y + ((velocity/yaw_rate) * (cos(particle.theta) - cos(particle.theta + yaw_rate*delta_t)));
      
      theta_mean = particle.theta + yaw_rate*delta_t;
    }
    // Numerical stability matters!
    else {
      x_mean = particle.x + (velocity * cos(particle.theta) * delta_t);
      
      y_mean = particle.y + (velocity * sin(particle.theta) * delta_t);
      
      theta_mean = particle.theta;
    };
    
    // We again sample the movement from a gaussian distribution centered around the position calculated above
    // This is to add some process noise
    normal_distribution<double> x_d(x_mean, std_pos[0]);
    normal_distribution<double> y_d(y_mean, std_pos[1]);
    normal_distribution<double> theta_d(theta_mean, std_pos[2]);
    
    particles[i].x = x_d(generator);
    particles[i].y = y_d(generator);
    particles[i].theta = theta_d(generator);
    
  };
}


// Now for the measurement update
// We update the weight of each particle based on the realistic-ness of observed landmarks
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
    std::vector<LandmarkObs> observations, Map map_landmarks) {
  
  //For each particle
  for (int i=0; i < num_particles; i++) {
    
    particles[i].associations.clear();
    particles[i].sense_x.clear();
    particles[i].sense_y.clear();

    //set weight to 1. It will be updated below
    double mult = 1.0;
    double sigmax = std_landmark[0];
    double sigmay = std_landmark[1];
    double theta = particles[i].theta;
    
    // For each measurement
    for (int j=0; j < observations.size(); j++) {
      double angle = theta;
      
      //Take x,y of the observation
      double a = observations[j].x;
      double b = observations[j].y;
      int obs_id = observations[j].id;
      
      //Transform coordinates to global space
      double x = particles[i].x + (a * cos(angle)) - (b * sin(angle));
      double y = particles[i].y + (a * sin(angle)) + (b * cos(angle));
      
      float mu_x, mu_y;
      Map::single_landmark_s best;
      double least_dist = INFINITY;
      
      //This loop picks the landmark which has lowest distance from the current measurement
      for (int k=0; k < map_landmarks.landmark_list.size(); k++) {
        Map::single_landmark_s land = map_landmarks.landmark_list[k];
        
        double distance = dist(x, y, land.x_f, land.y_f);
        
        if (distance < least_dist) {
          best = land;
          least_dist = distance;
        }
      }
      
      particles[i].associations.push_back(best.id_i);
      particles[i].sense_x.push_back(best.x_f);
      particles[i].sense_y.push_back(best.y_f);
      
      // calculating the multivariate gaussian in a few smaller steps
      mu_x = (x - best.x_f);
      mu_x = (mu_x * mu_x)/(2*sigmax*sigmax);
      mu_y = y - best.y_f;
      mu_y = (mu_y * mu_y)/(2*sigmay*sigmay);
      
      // Updating the overall weight by multiplying it to new weight
      double gaussian = (exp(-1*mu_x - mu_y)) / (2 * M_PI * sigmax * sigmay);
      mult = mult * gaussian;

    }
    particles[i].weight = mult;
    weights[i] = mult;
  }
}


// Resampling time! Well this is pretty much boilerplate code
void ParticleFilter::resample() {
	
  default_random_engine gen;
  
  discrete_distribution<int> dist(weights.begin(), weights.end());
  
  vector<Particle> resampled;
  
  for (int i=0; i < num_particles; i++) {
    resampled.push_back(particles[dist(gen)]);
  };
  
  particles = resampled;
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
