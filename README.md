This is the particle filter project. If you want to know the localisation from scratch please visit my article in medium.

```
//Get weights and max weight.
vector<double> weights;
double maxWeight = numeric_limits<double>::min();
for(int i = 0; i < num_particles; i++) {
	weights.push_back(particles[i].weight);
	if(particles[i].weight > maxWeight) {
		maxWeight = particles[i].weight;
	}
}

uniform_real_distribution<double> distDouble(0.0, maxWeight);
uniform_int_distribution<int> distInt(0, num_particles - 1);
int index = distInt(gen);
double beta = 0.0;
vector<Particle> resampledParticles;
for(int i = 0; i < num_particles; i++) {
	beta += distDouble(gen) * 2.0;
	while(beta > weights[index]) {
		beta -= weights[index];
		index = (index + 1) % num_particles;
	}
	resampledParticles.push_back(particles[index]);
}

particles = resampledParticles;
```
