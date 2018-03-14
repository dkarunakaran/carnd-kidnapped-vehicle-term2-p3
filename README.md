This is the particle filter project. If you want to know the localisation from scratch please visit my article in medium.

```
//Each particle for loop
for(int i = 0; i < num_particles; i++) {
	double paricle_x = particles[i].x;
	double paricle_y = particles[i].y;
	double paricle_theta = particles[i].theta;

	//Create a vector to hold the map landmark locations predicted to be within sensor range of the particle
	vector<LandmarkObs> predictions;

	//Each map landmark for loop
	for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {

		//Get id and x,y coordinates
		float lm_x = map_landmarks.landmark_list[j].x_f;
		float lm_y = map_landmarks.landmark_list[j].y_f;
		int lm_id = map_landmarks.landmark_list[j].id_i;

		//Only consider landmarks within sensor range of the particle (rather than using the "dist" method considering a circular region around the particle, this considers a rectangular region but is computationally faster)
		if(fabs(lm_x - paricle_x) <= sensor_range && fabs(lm_y - paricle_y) <= sensor_range) {
			predictions.push_back(LandmarkObs{ lm_id, lm_x, lm_y });
		}
	}

	//Create and populate a copy of the list of observations transformed from vehicle coordinates to map coordinates
	vector<LandmarkObs> trans_os;
	for(unsigned int j = 0; j < observations.size(); j++) {
		double t_x = cos(paricle_theta)*observations[j].x - sin(paricle_theta)*observations[j].y + paricle_x;
		double t_y = sin(paricle_theta)*observations[j].x + cos(paricle_theta)*observations[j].y + paricle_y;
		trans_os.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
	}

	//Data association for the predictions and transformed observations on current particle
	dataAssociation(predictions, trans_os);
	particles[i].weight = 1.0;
	for(unsigned int j = 0; j < trans_os.size(); j++) {
		double o_x, o_y, pr_x, pr_y;
		o_x = trans_os[j].x;
		o_y = trans_os[j].y;
		int asso_prediction = trans_os[j].id;

		//x,y coordinates of the prediction associated with the current observation
		for(unsigned int k = 0; k < predictions.size(); k++) {
			if(predictions[k].id == asso_prediction) {
				pr_x = predictions[k].x;
				pr_y = predictions[k].y;
			}
		}

		//Weight for this observation with multivariate Gaussian
		double s_x = std_landmark[0];
		double s_y = std_landmark[1];
		double obs_w = ( 1/(2*M_PI*s_x*s_y)) * exp( -( pow(pr_x-o_x,2)/(2*pow(s_x, 2)) + (pow(pr_y-o_y,2)/(2*pow(s_y, 2))) ) );

		//Product of this obersvation weight with total observations weight
		particles[i].weight *= obs_w;
	}
}
```
