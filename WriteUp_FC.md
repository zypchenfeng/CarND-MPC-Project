# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

#### The Model
#####The states:
The states of the car include: location(x, y), velocity(v), direction(psi), off-center error(cte) and angle error(epsi), also two control states which are turning (delta) and acceleration (a).
The state t+1 is depending on the state t:
```
    x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v_[t+1] = v[t] + a[t] * dt
    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
The states are fitted with 3rd order polynomia equation.

#####The cost function:
The cost function including:
1. location 
2. turning direction -- No abrupt turning
3. Speed -- not stay at zero
4. The change between actuation -- reduce actuator usuage
5. A special one 'fg[0] += 100*CppAD::pow(vars[delta_start + t] * vars[v_start+t], 2);' (from github https://github.com/jeremy-shannon/CarND-MPC-Project) was used to avoid fast response of wheel turning
```
// cost base on reference start
for (unsigned int  t = 0; t < N; ++t) {
    fg[0] += 100*CppAD::pow(vars[cte_start + t], 2);
    fg[0] += 100*CppAD::pow(vars[epsi_start + t], 2);
    fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2); // to prevent vehicle stop in the middle of path
}

// minimize use of the actuators
for (unsigned int  t = 0; t < N-1; ++t) {
    fg[0] += 10*CppAD::pow(vars[delta_start + t], 2);
    fg[0] += 20*CppAD::pow(vars[a_start + t], 2);      
}

// Minimize the value gap between sequential actuations
for (unsigned int  t = 0; t < N - 2; ++t) {
fg[0] += 100*CppAD::pow(vars[delta_start + t] * vars[v_start+t], 2);
    fg[0] += 500*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2); // for smoother action such as lane change
    fg[0] += 500*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2); // for smoother moving forward
}
```

#####Constrains
Constrains are used to solve the linear equation


##### Timestep
In the project I used N=25 and dt = 0.05, which gives about 1.25 seconds, or about 20yards if running at 45miles/hour, this gives relatively good response.
I have tried N=10 and dt = 0.15, not much difference. 

##### Polynomial Fitting and MPC Preprocessing
The waypoints need to be transformed to the car coordinate system using `dx = dX*cos(psi) + dY*sin(psi), dy = dY*cos(psi) - dX*sin(psi)`.

####Latency
A 100 millisecond latency was added to the beginning of the model, which means move the model forward by 0.1 seconds by calculating the state base on equations in "The states" using dt=0.1.
