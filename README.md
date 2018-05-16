# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Project Goal

The goal of the project is to model a self-driving using a Model Predictive Controller. The model is tested on a simulated car in the Udacity simulator. The following are the expectations of the project:

- The car should stay within the lane and not veer too far off the center.
- The car should drive smoothly minimizing any jerky movements.
- The car should drive at or below the desired speed.

### Vehicle Model

The model used for the vehicle is the kinematic bicycle model. This model takes into account the dynamics of the system, like velocity, heading and ignores other factors like friction, mass and various forces. The following set of equations describe the state of a vehicle at any given time:

<p align="center"><img src="svgs/61e4158173dc6895470988dbecf2b63d.svg?invert_in_darkmode&sanitize=true" align=middle width=244.71315pt height=175.31415pt/></p>

Here <img src="svgs/c47305a36647762ef14f697f4092fca2.svg?invert_in_darkmode&sanitize=true" align=middle width=145.24521pt height=22.74591pt/> represents the current state of the vehicle at time <img src="svgs/4f4f4e395762a3af4575de74c019ebb5.svg?invert_in_darkmode&sanitize=true" align=middle width=5.913963pt height=20.1465pt/>. <img src="svgs/1c2c8c7d47260addae719b977f037491.svg?invert_in_darkmode&sanitize=true" align=middle width=25.57401pt height=20.1465pt/> is the cross-track error, the distance of the vehicle's center from the trajectory. <img src="svgs/ad2e92cff6ef93ef8b058b56577f32f0.svg?invert_in_darkmode&sanitize=true" align=middle width=23.24058pt height=22.74591pt/> is the difference in the heading and desired heading <img src="svgs/a57b5ff01a8873c4f692ef7a73c96c45.svg?invert_in_darkmode&sanitize=true" align=middle width=40.031145pt height=22.74591pt/>. The above set of equations will estimate the state of vehicle at time <img src="svgs/628783099380408a32610228991619a8.svg?invert_in_darkmode&sanitize=true" align=middle width=34.145925pt height=21.10812pt/>.  Also, <img src="svgs/a37302359718eac43a8aec1b5310a2be.svg?invert_in_darkmode&sanitize=true" align=middle width=18.8166pt height=22.38192pt/> is a constant that represents the distance between the center of mass of the vehicle and it's front wheels. Without it, the above model is only true for a point particle, which is not a realistic reflection of our vehicle. <img src="svgs/a37302359718eac43a8aec1b5310a2be.svg?invert_in_darkmode&sanitize=true" align=middle width=18.8166pt height=22.38192pt/> is defined in the code [here](src/MPC.cpp#L22).

Furthermore, <img src="svgs/10ea9eec57d7dd8109d5e58e9baf6620.svg?invert_in_darkmode&sanitize=true" align=middle width=12.226005pt height=22.74591pt/> and <img src="svgs/9789555e5d8fa5de21171cc40c86d2cd.svg?invert_in_darkmode&sanitize=true" align=middle width=13.60392pt height=14.10255pt/> represent the actuators of the system. <img src="svgs/10ea9eec57d7dd8109d5e58e9baf6620.svg?invert_in_darkmode&sanitize=true" align=middle width=12.226005pt height=22.74591pt/> is the change in heading, which can be assumed as the amount of steering to apply. <img src="svgs/9789555e5d8fa5de21171cc40c86d2cd.svg?invert_in_darkmode&sanitize=true" align=middle width=13.60392pt height=14.10255pt/> is the acceleration and is used as an approximation of the throttle to be applied.

### Model Predictive Control

Once we have the kinematic model of our vehicle, we can use MPC to estimate our future trajectory. In MPC, an optimal control problem is "solved" for a certain number of steps, called the horizon, based on certain frequency. The horizon and frequency are represented in the code by variables `N` and `dt` in file [MPC.cpp](src/MPC.cpp#L9-10) . 

The optimal control problem referred above is a nonlinear optimization problem that tries to minimize a certain cost given certain constraints. 

The cost in our case is given by:
<p align="center"><img src="svgs/76999a1fb9f84ea2d11a73514a63238d.svg?invert_in_darkmode&sanitize=true" align=middle width=488.70195pt height=162.51906pt/></p>

The code for the costs is at [TODO: MPC.cpp lines for cost ](src/MPC.cpp#L58-86)

[TODO: Explain the above variables here]

The constraints in our case are given by:

<p align="center"><img src="svgs/2759068c07eea3bbddfcba9d2369dec5.svg?invert_in_darkmode&sanitize=true" align=middle width=105.73233pt height=41.03451pt/></p>

The code for these constraints is at [TODO: MPC.cpp lines for constraints ](src/MPC.cpp#L191-212)

#### Reference frame

The computations for the model are done in the reference frame of the vehicle. Since the waypoints are received in global coordinates, they are converted into vehicle coordinates at [TODO: main.cpp transform lines](src/main.cpp#L106-114)

#### Trajectory representation

The trajectory in our case consists of waypoints that are pre-defined along the route of travel. These waypoints are represented by a third degree polynomial <img src="svgs/d53011c4c7824fee58ecd4cecbff13ee.svg?invert_in_darkmode&sanitize=true" align=middle width=46.24521pt height=24.56553pt/>. It is computed in the code at [TODO: main.cpp polyfit call](src/main.cpp#L119)

#### Latency

The simulation adds a latency of about 100ms. This is to simulate the delay between actuation and effect. We need to account for this latency properly otherwise the computed and reference trajectories will keep diverging. The code for this is at [TODO: main.cpp lines for updated state equations]

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
