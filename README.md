This is the particle filter project. If you want to know the localisation from scratch please visit my article in medium.

```
if(is_initialized) {
		return;
	} 

	//Number of particles
	num_particles = 100;

	//SD
	double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];

	//Normal distributions
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	//Generate particles with normal distribution with mean on GPS values.
	for(int i = 0; i < num_particles; i++) {
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1.0;
		particles.push_back(p);
	}
	is_initialized = true;
  ```
