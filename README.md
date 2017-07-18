# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
### Model Predictive Control (MPC)

The main goal of the project is to implement in C++ Model Predictive Control to drive the car around the track. MPC uses an approximate Global Kinematic Model to predict the motion of the vehicle and an optimization algorithm is used to calculate the required control inputs to move along the desired trajectory.

You can view the performance of the controller at the link below:
https://youtu.be/uSz2TF9mhtM

The controller used a kinematic model to describe the state of the car. The state of the car was determined by its x position, y position, orientation, velocity, crosstrack error, and orientation error. The optimal steering angle and throttle required for navigation of the car was calculated/predicted using these current state, waypoints/trajectory, kinematic equations and practical constraints on throttle/steering angle. The model used for the project was very similar to the one provided in the solution for the mpc quiz, except for certain specifics described in the subsection below. Because we are looking for an optimal path, calculations are posed as an optimization problem with proper constraints.
We used IpOpt C++ library to solve the optimization equation.

## Kinematic Model

A simple Kinematic model (ignores tire forces, gravity, mass, etc) was used for the Controller. 

Position (_x,y_), heading (_ψ_) and velocity (_v_) form the vehicle state vector:

State: _[x,y,ψ,v]_


There are two actuators. Stearing angle (_δ_) is the first one, it should be in range [-25,25] deg. For simplicity the throttle and brake represented as a singular actuator (_a_), with negative values signifying braking and positive values signifying acceleration. It should be in range [-1,1].

Actuators: _[δ,a]_

Errors: cross track error (_cte_) and _ψ_ error (_eψ_) were used to build the cost function for the MPC. They could be updated on a new time step using the following equa

The kinematic model can predict the state on the next time step by taking into account the current state and actuators as follows:

![equations](/equations.png)

where _Lf_ measures the distance between the front of the vehicle and its center of gravity. 

These state equations are defined in MPC.cpp as constraints for optimization:

## MPC Parameter Tuning

Very similar to PID, MPC also requires tuning  of controller parameters to get optimal performance. In this section we will discuss more about the optimization equation and tunning of weights in the optimization problem to obtain optimal result.
First of all, data about waypoints was transformed into the vehicle space and a 3d order polynomial was fitted to the data.

##### A. Objective Function

The objective function for the model/optimization problem has three parts:

**State Cost:** This incorporated cost of changes in cross-track error (cte), heading error and speed error. The three costs were weighted differently to assessing their relative importance. The main goal of the model is to drive at the center of the track. 

**Actuation cost:** This cost is used to minimize the use of actuators, probably because we want fuel efficiency is concern.

**Cost of changing actuation:** To prevent the car from suddently accelerating and changing the steering angle abruptly, the cost dependent on changes in actuation was added. This is included because we want the transitions in situations such as changing from one lane to another smooth. A high weight of 500 was used for steering angle change to force the car to make smoother turns and prevent it from swinging on the track.
[To do: Add Snapshot of Objective Function]

##### B. Constraints
The constraints on the model:
1. One set of constraints was state equations (kinematic model equations):
At each step, the model has to follow the state equations.
```
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```
2. Actual constraints on throttle and steer angle:
 These were kept the same as the ones in the quiz solution. The one change that was made was that the steering angle was constrained between -1 and 1, instead of -deg2Rads(25) and deg2Rads(25). By doing so, no conversion was needed while passing values to the simulator.
```
  // The upper and lower limits of delta are set to -25 and 25 degrees
  for (unsigned int i = delta_start; i < a_start; i++) 
  {
    vars_lowerbound[i] = -DEG25_RAD;
    vars_upperbound[i] = DEG25_RAD;
  }
 // Acceleration/decceleration upper and lower limits.
  for (unsigned int i = a_start; i < n_vars; i++) 
  {
    vars_lowerbound[i] = -MAX_ACCEL;
    vars_upperbound[i] = MAX_ACCEL;
  }
 ```
First of all, data about waypoints was transformed into the vehicle space and a 3d order polynomial was fitted to the data. Actual state of the vehicle was "predicted" into the future by 100 ms latency. This improves performance of the controller as errors and state were calculated more accurately.

##### C. Duration of Estimating Horizon:
The MPC is used to predict the state of the system in the following "N" steps based on the current state. The first step of actuation is performed and the prediction is carried out again for the next "N" steps. Thus the horizon of the controller is changing. After trying different combination of number of steps (N) and time differnece between states (dt), I settled on N = 10 and dt = 0.1s.

At speed of 50mph, 1 sec horizon was useful in fitting the way points around a curved section of the track an optimal amount of time before the car had to actually turn. I tried smaller dt values and found the car behaving erractily at curves and then swinging from one end of the track to another.

##### D. Cost Function Weights
The cost function weights were tuned by trial-and-error method. All these parameters are stored in the `src/MPC.h` file.
To tune the MPC following parameters were used in MPC.h
#define ref_v 50
#define NUM_STEPS 20
#define DELTA_TIME 0.1

#define A_WEIGHT 50 
#define DELTA_WEIGHT 1000
#define A_DIFF_WEIGHT 1
#define DELTA_DIFF_WEIGHT 0.1
#define VREF_WEIGHT 1
#define CTE_WEIGHT 1
#define PSI_WEIGHT 1

Tuning the parameters:
a) A_WEIGHT:
Increasing a_weight decreased the speed at which vehicle can navigate, while decreasing a_weight increases the speed but it also introduced more deviation for the reference trajectory

b) DELTA_WEIGHT:
Increasing delta weight too high caused the vehicle to not steer quickly at turns and was getting off track sometimes.
However if the delta weight is too low, vehicle seems steer too much even on straight path.

c) VREF_WEIGHT also contributed to increase/decrease in speed of vehicle


I also have tried the tuning the model with and without state prediction before optimizing the values for MPC solver.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

