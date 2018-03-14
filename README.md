This is the particle filter project. If you want to know the localisation from scratch please visit my article in medium.

```
//Normal distributions for sensor noise
normal_distribution<double> dist_x(0, std_pos[0]);
normal_distribution<double> dist_y(0, std_pos[1]);
normal_distribution<double> dist_theta(0, std_pos[2]);

for(int i = 0; i < num_particles; i++) {
	if(fabs(yaw_rate) < 0.00001) {  
		particles[i].x += velocity * delta_t * cos(particles[i].theta);
		particles[i].y += velocity * delta_t * sin(particles[i].theta);
	} 
	else{
		particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
		particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
		particles[i].theta += yaw_rate * delta_t;
	}

	//Noise
	particles[i].x += dist_x(gen);
	particles[i].y += dist_y(gen);
	particles[i].theta += dist_theta(gen);
}
```
