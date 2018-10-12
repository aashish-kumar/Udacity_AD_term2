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
#include <climits>
#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  // This line creates a normal (Gaussian) distribution for x
  num_particles = 100;
  default_random_engine gen;
  normal_distribution<double> dist_x(0, std[0]);

  // TODO: Create normal distributions for y and theta
  normal_distribution<double> dist_y(0, std[1]);
  normal_distribution<double> dist_theta(0, std[2]);

  
  for (int i = 0; i < num_particles; ++i) {
    Particle p;
    
    // TODO: Sample  and from these normal distrubtions like this: 
    //   sample_x = dist_x(gen);
    //   where "gen" is the random engine initialized earlier.
    
     p.x = x+dist_x(gen);
     p.y = y+dist_y(gen);
     p.theta = theta+dist_theta(gen);  
     p.weight = 1; 
     particles.push_back(p);

  }
  weights = std::vector<double>(num_particles,1.0);
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine
  default_random_engine gen;
  normal_distribution<double> dist_x(0, std_pos[0]);
 // TODO: Create normal distributions for y and theta
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  for(int i=0;i<particles.size();i++)
  {
    Particle *p =&particles[i];


    double yaw_change = yaw_rate*delta_t;
    if(yaw_rate>0.000001)
    {
      p->x = p->x + (velocity/yaw_rate )*(sin(p->theta + yaw_change) - sin(p->theta))+dist_x(gen);
      p->y = p->y + (velocity/yaw_rate)*(cos(p->theta) - cos(p->theta + yaw_change))+dist_y(gen);
      p->theta = p->theta + yaw_change + dist_theta(gen);
    }
    else
    {
      p->x = p->x + cos(p->theta)*velocity*delta_t+dist_x(gen);
      p->y = p->y + sin(p->theta)*velocity*delta_t+dist_y(gen);
      p->theta = p->theta + yaw_change + dist_theta(gen);
    }
  }

}
 
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.



  //process all the particles

  for(int i=0;i<observations.size();i++)
  {
    double mindist = INT_MAX;
    int predict_id = -1;
    for(int j=0;j<predicted.size();j++)
    {
      double distance = dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y);
      if(distance < mindist)
      {
        mindist = distance;
        predict_id = predicted[j].id;
      }
    }
    observations[i].id = predict_id;
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

  for(int i=0;i<num_particles;i++)
  {
    double px = particles[i].x;
    double py = particles[i].y;
    double ptheta = particles[i].theta;

    //find the landmarks in sensor_range
    vector<LandmarkObs> inRangepredict;
    for(int j=0;j<map_landmarks.landmark_list.size();j++)
    {
      Map::single_landmark_s lm = map_landmarks.landmark_list[j];
      if(fabs(lm.x_f-px)<=sensor_range && fabs(lm.y_f-py)<=sensor_range){
        LandmarkObs temp;
        temp.id = lm.id_i;
        temp.x = lm.x_f;
        temp.y = lm.y_f;
        inRangepredict.push_back(temp);
      }
    }

    //transform the coordinates for observations
    std::vector<LandmarkObs> observations_map_coordinate;
    for(int j=0;j<observations.size();j++)
    {
      LandmarkObs tmp = observations[j];
      LandmarkObs lmark;
      lmark.x = px + (cos(ptheta)*tmp.x) - (sin(ptheta)*tmp.y);
      lmark.y = py + (sin(ptheta)*tmp.x) + (cos(ptheta)*tmp.y);
      lmark.id = tmp.id;
      observations_map_coordinate.push_back(lmark);
    }


    //lets associate the observations with landmarks in range
    dataAssociation(inRangepredict,observations_map_coordinate);

    //initialize the weight = 1 as the probability will be multiplied for all observations
    particles[i].weight = 1;

    //lets process each observation 
    double lm_corres_x,lm_corres_y;
    for(int j=0;j<observations_map_coordinate.size();j++)
    {
      //picking an observtion
      LandmarkObs lm = observations_map_coordinate[j];

      //find x y for the correct landmark for this observation
      for(int k=0;k<inRangepredict.size();k++){
        if(inRangepredict[k].id == lm.id)
        {
          lm_corres_x = inRangepredict[k].x;
          lm_corres_y = inRangepredict[k].y;
          break;
        }
      }
      //lets use multi-variate gaussian distribution to estimate the closeness of particle to a suitable position 
      double std_lmarkx= std_landmark[0],std_lmarky = std_landmark[1];
      double norm = (1/(2*M_PI*std_lmarky*std_lmarkx));
      double expo =  pow(lm.x - lm_corres_x,2)/(2*pow(std_lmarkx,2)) + pow(lm.y - lm_corres_y,2)/(2*pow(std_lmarky,2));
      double weight = norm*exp(-expo);
      particles[i].weight*=weight;

    }
    weights[i] = particles[i].weight;
  }

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  std::default_random_engine generator;
  std::discrete_distribution<int> indexer(weights.begin(),weights.end());
 
  std::vector<Particle> tempparticle;
  for(int i=0;i<num_particles;i++)
  {
    int ix = indexer(generator);
    tempparticle.push_back(particles[ix]);
  }
  particles = tempparticle;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
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
