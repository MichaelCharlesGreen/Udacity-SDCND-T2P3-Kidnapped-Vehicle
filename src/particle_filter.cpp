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
  // TODO:
  // Set the number of particles.
  //
  // Initialize:
  // All particles to first position (based on estimates of x, y, theta and their uncertainties from GPS).
  // All weights to 1.
  //
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  
  // L14M2:
  // Initialization takes initial estimate (GPS) and sensor noise.
  // L14M3:
  // Initialize by sampling from a Gaussian distribution taking into account Gaussian sensor noise around the initial GPS position estimate
  // and the initial heading estimate.
  // Use the C++ standard library, normal distribution function to sample positions around a GPS measurement.
  
  // Initialize random engine.
  default_random_engine gen;
  
  // Set the number of particles; should probably be a constant.
  num_particles = 30;
  
  // Size particles and weights vectors to the number of particles.
  particles.resize(num_particles);
  weights.resize(num_particles);
  
  // Standard deviations for x, y, and theta.
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];
  
  // Create a normal (Gaussian) distribution for x.
  normal_distribution<double> dist_x(x, std_x);
  // Create normal distributions for y and theta.
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  
  for (int i = 0; i < num_particles; i++)
  {
	// Initialize particle to random x, y, and theta that corresponds to our uncertainty.
	// The uncertainty is given to us at the top of main.cpp
	// Go through each particle and generate a new random x, y, theta that move along the sigma
	// and the weights will start off as 1.
	
	Particle particle;
	particle.id = i;
	particle.weight = 1;
	
	particle.x = dist_x(gen);
	particle.y = dist_y(gen);
	particle.theta = dist_theta(gen);
	
	particles[i] = particle;
  }
  
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  // L14M2
  // Prediction step takes yaw rate, velocity and sensor noise and is called again after resampling.
  // L14M6
  // Used what we learned in the motion models lesson to predict where the car will be at the next time step.
  // For each particle, you will have to update the particle's location based on velocity and yaw rate measurements.
  // To account for the uncertainty in the control input, add Gaussian noise to the velocity and yaw rate.
  // L14M14
  // Update each particle's position estimates and account for sensor noise by adding Gaussian noise.
  // You can add Gaussian noise by sampling from a Gaussian distribution with mean equal to the updated particle position, and standard
  // deviation equal to the standard deviation of the measurements.
  
  // Initialize random engine.
  default_random_engine gen;
  
  // Standard deviations for x, y, and theta.
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];
  
  double factor = velocity / yaw_rate; // theta_dot is the yaw_rate
  
  for (int i = 0; i < num_particles; i++)
  {
	// Predicted particle position.
	double pred_part_x, pred_part_y, pred_part_theta;
	
	if (fabs(yaw_rate) < 0.001) // should be a constant
	{
	  pred_part_x = particles[i].x + (velocity * delta_t * cos(particles[i].theta));
	  pred_part_y = particles[i].y + (velocity * delta_t * sin(particles[i].theta));
	  pred_part_theta = particles[i].theta;
	}
	else
	{
	  // The equations for updating x, y, and the yaw angle when the yaw rate is not equal to 0:
	  // xf = x0 + velocity/theta_dot[sin(theta_0 + theta_dot*dt) - sin(theta_0)]
	  // yf = y0 + velocity/theta_dot[cos(theta0) - cos(theta0 + theta_dot*dt))]
	  // thetaF = theta0 + theta_dot*delta_t
	  pred_part_x = particles[i].x + (factor * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta)));
	  pred_part_y = particles[i].y + (factor * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t))));
	  pred_part_theta = particles[i].theta + (yaw_rate * delta_t);
	}
	
	// Normal distributions for x, y, and theta with sensor noise.
	normal_distribution<double> dist_x(pred_part_x, std_x);
	normal_distribution<double> dist_y(pred_part_y, std_y);
	normal_distribution<double> dist_theta(pred_part_theta, std_theta);
	
	// Add noise to results to add variation to the selected particles so we don't get the same results
	// as before.
	particles[i].x = dist_x(gen);
	particles[i].y = dist_y(gen);
	particles[i].theta = dist_theta(gen);
  }
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
  
  // TODO: put this in a helper function
  // For each particle:
  for (int i = 0; i < num_particles; i++)
  {
	// Coordinate transformation and rotation: transform observations from car to map coordinate system.
	// https://en.wikipedia.org/wiki/Rotation_of_axes for derivation
	vector<LandmarkObs> observations_mcs(observations.size()); // _mcs == mapcoordinatesystem
	for (int j = 0; j < observations.size(); j++)
	{
	  observations_mcs[j].x = particles[i].x + observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[j].theta);
	  observations_mcs[j].y = particles[i].y + observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta);
	  observations_mcs[j].id = observations[j].id;
	}
	
	// Data association: associate map landmarks with the nearest observation (measurement).
	vector<double> distances; // distances (using Pythagorean theorem) from a particle to each landmark within sensor range
	vector<int> associations;
	vector<double> sense_x;
	vector<double> sense_y;
	
	// TODO: Research and implement a more efficient search for the nearest neighbor.
	// Find the nearest observation (within sensor range) to the map landmark.
	distances.resize(observations_mcs.size());
	int min_distance_id = 0;
	// For each map landmark...
	for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
	{
	  // Calculate the distance between the particle under investigation and this map landmark.
	  double dist_part_land = sqrt(pow((particles[i].x - map_landmarks.landmark_list[j].x_f), 2) + pow((particles[i].y - map_landmarks.landmark_list[j].y_f), 2));
	  // If the distance between the particle and the landmark is greater than the sensor's range, it will not be appearing as a valid
	  // observation from the sensor data so exclude it from further analysis.
	  // However, if it is within sensor range, calculate the distance from the landmark to the observations and select the nearest
	  // observation.
	  // If the distance from the particle to the landmark is within sensor range...
	  if (dist_part_land < sensor_range)
	  {
		// Calculate the distance between the map landmark and the first observation.
		distances[0] = sqrt(pow((observations_mcs[0].x - map_landmarks.landmark_list[j].x_f), 2) + pow((observations_mcs[0].y - map_landmarks.landmark_list[j].y_f), 2));
		min_distance_id = 0; // This index is a starting point; the index will be updated as closer observations are discovered.
		
		// Process all observations to find the one nearest to the landmark.
		for (int k = 1; k < observations_mcs.size(); k++) // beginning with the second observation and moving through all observations
		{
		  // Calculate the distance between the map landmark and the subsequent observation.
		  distances[k] = sqrt(pow((observations_mcs[k].x - map_landmarks.landmark_list[j].x_f), 2) + pow((observations_mcs[k].y - map_landmarks.landmark_list[j].y_f), 2));
		  // If any subsequent observation is closer to the landmark...
		  if (distances[k] < distances[min_distance_id])
		  {
			min_distance_id = k; // set index to that of the new nearest observation
		  }
		  // process next observation
		}
		
		// Assign the landmark to the nearest observation.
		observations_mcs[min_distance_id].id = j + 1;
		associations.push_back(j + 1);
		sense_x.push_back(observations_mcs[min_distance_id].x);
		sense_y.push_back(observations_mcs[min_distance_id].y);
	  }
	  // process next map landmark
	}
	
	// Save associations for this specific particle
	SetAssociations(particles[i], associations, sense_x, sense_y);
	// TODO: above could be put in helper function
	
	
	// Update weight for each particle.
	double prob = 1.0;
	// For each observaton...
	for (int j = 0; j < observations.size(); j++)
	{
	  int associated_landmark_id = observations_mcs[j].id;
	  if (associated_landmark_id > 0)
	  {
		double x = observations_mcs[j].x;
		double y = observations_mcs[j].y;
		double x_pred = map_landmarks.landmark_list[associated_landmark_id - 1].x_f;
		double y_pred = map_landmarks.landmark_list[associated_landmark_id - 1].y_f;
		double sigma_x = std_landmark[0];
		double sigma_y = std_landmark[1];
		double sigma_x_squared = pow(sigma_x,2);
		double sigma_y_squared = pow(sigma_y,2);
		double delta_x = x - x_pred;
		double delta_y = y - y_pred;
		// L11M24
		// L14M11
		// L14M13
		// Multivariate-Gaussian Probability
		prob *= exp(-0.5*((pow(delta_x,2)/sigma_x_squared) + (pow(delta_y,2)/sigma_y_squared))) / (2*M_PI*sigma_x*sigma_y);
	  }
	  // process next observation
	}
	particles[i].weight = prob;
	weights[i] = prob;
  }
  // process next particle
}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  // Use the weights of the particles in your particle filter and C++ standard librarie's discrete distribution function to update your
  // particles to the Bayesian posterior distribution.
  // L14M11
  // After you update the weights for each particle, you will have to resample the particle with probability proportional to these weights.
  // L13M14
  // Let the particles with high weights survive somewhat at random, but the probability of survival will be proportional to their weights.
  // Which means, after what's called resampling, which is just a technical term for randomly drawning N new particles from these old ones
  // with replacement in proportion to their importance weight.
  // The particles that are very consistent with the sensor measurement survive with a higher probability, and teh ones with lower
  // importance weight tend to die out. So we get the effect that the particles cluster around regions of higher posterior probability.
  // L13M15
  // With replacement, there can be multiple copies of teh same particle, and in the end those particles that have a high-normalized
  // weight will occur likely more frequently in the new set. That's called resampling.
  // L13M20
  // Very initially let's guess a particle index uniformly from the set of all indices of the resampling wheel.
  // This is denoted as a uniform sample at U from the discrete set of choices of index 1 all the way to N.
  // Determine a value beta that you initialize to 0 and to which you add--when you construct these particles--a uniformly drawn
  // continuous value that sits between 0 and 2 times W max, which is the largest of the importance weights in the important set.
  // In other words, add a random value that might be as large as twice the largest weight.
  
  // It's easy to see that each particle is now picked in proportion to the total circumference it spans in this wheel of particles.
  
  // Implementation
  // Create a new set of particles called new_particles; it's an empty set in the beginning.
  // With every resample, add a particle from the previous particle set with the index, index.
  // At the end, assign new_particles back to particles.
  // The very first index is drawn at random - a uniform random sampler of all the indices.
  // beta is a running variable set to 0.0
  // Cache the max of W just to be slightly faster.
  // Produce num_particles particles. Do this by adding to beta a uniform random that is twice as large as the maximum weight. Two times
  // max weight will be a very large step, but by adding a random variable that sits between 0 and 1
  
  // The result is a set of particles that are all co-located. So instead of having a complete random set of particles, like before, the
  // resampling step gives particles of very similar x and y postions.
  // It turns out, the orientations are not very similar. Orientation plays no role in the projected measurement and therefore has no role
  // in the selection.
  
  // Rather than using the Resampling Wheel method Sebastian Thrun presents in L13M20, a discrete distribution methodology will be
  // implemented.
  // https://stackoverflow.com/questions/22108836/define-and-use-a-discrete-probability-density-function-in-c
  vector<Particle> new_particles(num_particles);
  // Initialize random engine.
  default_random_engine gen;
  // std::discrete_distribution produces random integers on the interval [0, n), where the probability of each individual integer i is
  // defined as the weight of the ith integer divided by the sum of all n weights.
  // The weights of the particles sum to one. For example, five particles may have weights of 0.1, 0.2, 0.1, 0.4, 0.2
  // This means that the probablity of selecting a particular particle is proportionate to its weight.
  discrete_distribution<> dis_dist(weights.begin(), weights.end());
  
  for (int i = 0; i < num_particles; i++)
  {
	int index = dis_dist(gen);
	new_particles[i] = particles[index];
	new_particles[i].id = i;
  }
  
  particles = new_particles;
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
