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

static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 100;  // TODO: Set the number of particles

	double std_x, std_y, std_theta;
	std_x = std[0];
	std_y = std[1];
	std_theta =std[2];

	normal_distribution<double> N_x(x, std_x);
	normal_distribution<double> N_y(y, std_y);
	normal_distribution<double> N_theta(theta, std_theta);

	// Initilize Particles
	for(int i = 0; i < num_particles; i++)
	{
		Particle particle;
		particle.id = i;
		particle.x = N_x(gen);
		particle.y = N_y(gen);
		particle.theta = N_theta(gen);
		particle.weight = 1.0;

		particles.push_back(particle);
		weights.push_back(1.0);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	double std_x, std_y, std_theta;

	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];

	//Predict Particles
	for(int i = 0; i < num_particles; i++)
	{
		double new_x, new_y, new_theta;
		double old_x, old_y, old_theta;

		old_x = particles[i].x;
		old_y = particles[i].y;
		old_theta = particles[i].theta;

		if(fabs(yaw_rate) < 0.0001)
		{
			new_x = old_x + velocity*delta_t*cos(old_theta);
			new_y = old_y + velocity*delta_t*sin(old_theta);
			new_theta = old_theta;
		}
		else
		{
			new_x = old_x + (velocity/yaw_rate)*(sin(old_theta + yaw_rate * delta_t)-sin(old_theta));
			new_y = old_y + (velocity/yaw_rate)*(cos(old_theta) - cos(old_theta + yaw_rate * delta_t));
			new_theta = old_theta + yaw_rate * delta_t;
		}
		// Add noise on prediction
		normal_distribution<double> N_x(new_x, std_x);
		normal_distribution<double> N_y(new_y, std_y);
		normal_distribution<double> N_theta(new_theta, std_theta);

		particles[i].x = N_x(gen);
		particles[i].y = N_y(gen);
		particles[i].theta = N_theta(gen);

	}




}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	//Associate the closest landmark with observed measurement
	for(int i = 0; i < observations.size() ;i++)
	{
		double min_dist = numeric_limits<double>::max();
		LandmarkObs obs = observations[i];

		int map_id = -1;

		for(int j =0; j < predicted.size(); j++)
		{
			LandmarkObs pred = predicted[j];

			double curr_dist = dist(obs.x, obs.y, pred.x, pred.y);

			if(curr_dist < min_dist)
			{
				min_dist = curr_dist;
				map_id = pred.id;
			}

		}
		// Assign the closest landmark ID to observed measurement
		observations[i].id = map_id;
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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

	double std_x = std_landmark[0];
	double std_y = std_landmark[1];
	double std_x_2 = pow(std_x,2);
	double std_y_2 = pow(std_y,2);
	double normalizer = (1.0/(2.0*M_PI*std_x*std_y));
	double weight_normalizer = 0.0;

	for(int i = 0; i < num_particles; i++)
	{
		double x_part = particles[i].x;
		double y_part = particles[i].y;
		double theta = particles[i].theta;
		std::vector<int> associations;
		std::vector<double> sense_x;
		std::vector<double> sense_y;

		std::vector<LandmarkObs> predictions;

		//Filter landmarks that is outside sensor range
		for(int m = 0; m < map_landmarks.landmark_list.size(); m++)
		{
			float landmark_x = map_landmarks.landmark_list[m].x_f;
			float landmark_y = map_landmarks.landmark_list[m].y_f;
			int landmark_id = map_landmarks.landmark_list[m].id_i;

			if((fabs(landmark_x - x_part) <= sensor_range) && (fabs(landmark_y - y_part) <= sensor_range ))
			{
				predictions.push_back(LandmarkObs{landmark_id, landmark_x, landmark_y});
			}
		}

		//Create a new list of observations transformed from Vehicle coordinates to map coordinates
		vector<LandmarkObs> trans_observations;
		for (int j = 0; j < observations.size(); j++)
		{
			LandmarkObs trans_obs;
			trans_obs.x = x_part + cos(theta) * observations[j].x - sin(theta) * observations[j].y;
			trans_obs.y = y_part + sin(theta) * observations[j].x + cos(theta) * observations[j].y;
			trans_obs.id = observations[j].id;
			trans_observations.push_back(trans_obs);			
		}

		dataAssociation(predictions, trans_observations);

		particles[i].weight = 1.0;

		associations.clear();
		sense_x.clear();
		sense_y.clear();

		//Get the x, y coordinates of the prediction associated with transformed observation
		for(int k = 0; k < trans_observations.size(); k++)
		{  

			int association_id = trans_observations[k].id;
			double meas_x = trans_observations[k].x;
			double meas_y = trans_observations[k].y;
			double mu_x, mu_y;

			for(int l = 0; l < predictions.size(); l++)
			{
				if(predictions[l].id == association_id)
				{
					mu_x = predictions[l].x;
					mu_y = predictions[l].y;
					associations.push_back(association_id);
					sense_x.push_back(meas_x);
					sense_y.push_back(meas_y);
				}

			}
			//Calculate paritcle Multivariate-Gaussian Probability
			long double w = normalizer*exp(-((pow(meas_x-mu_x,2)/(2.0*std_x_2)) + (pow(meas_y-mu_y,2)/(2.0*std_y_2))));
			if(fabs(w) < 0.0001)
			{	
				particles[i].weight *= 0.0001;
			}
			else
			{
				particles[i].weight *= w;
			}
			

		}
		weight_normalizer += particles[i].weight;

		//weights[i] = particles[i].weight;
		// Use for debugging
		//SetAssociations(particles[i], associations, sense_x, sense_y);
	}
	for(int i = 0; i < num_particles; i++)
	{
		particles[i].weight /= weight_normalizer;
		weights[i] = particles[i].weight;
	}
	

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	uniform_int_distribution<int> uniintdist(0, num_particles-1);
	
	int index = uniintdist(gen);
	vector<Particle> new_particles;

	// Get the maximum value of weight in weights list
	double max_weight = *std::max_element(weights.begin(),weights.end());

	uniform_real_distribution<double> unirealdist(0.0, max_weight*2.0);

	double beta = 0.0;

	//Spin the resampling wheel
	for(int i = 0; i < num_particles; i++)
	{
		beta += unirealdist(gen);
		while( weights[index] < beta)
		{
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		new_particles.push_back(particles[index]);
	}
	particles = new_particles;

}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
