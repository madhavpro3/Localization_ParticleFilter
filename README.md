**Localization** is the problem of an autonomous vehicle identifying its position with-in a map. Once localized, the robot may be able to plan its motion towards its destination. Commonly used Global Positioning System(GPS) measurements are not accurate enough for a self-driving car and are also unreliable.
 
**Particle filter** is a type of Bayesian filter used for localization of a robot within a known map; The map is assumed to be fixed in this project. The localization is achieved by the robot sensing landmarks in the map. Bayesian filters are iterative techniques i.e they have repeated operations: prediction and update. 

**The iterative structure** is achieved by incorporating Markov assumptions which simplifies prediction of future state by solely basing it on the current state without the need for information of any other previous states. The knowledge of the robot’s position at the start of an iteration is called a priori and the estimations made after are called posterior.

**The prediction** step uses a motion model to calculate the probabilities of being at a future state from the current state. In this project for the autonomous vehicle, we are using a *‘Bicycle model’* of the car. 

**The update** step involves taking in sensor measurements and adjusting the predictions.

Particle filter utilizes particles representing a hypothetical car. Initially, the particles are spread out, by iterating through the update and prediction steps, the particles converge to the real car position.

Pseudo code
* Initialize ‘n’ particles based on the GPS information and noises
* Predict the next position of the particle using the motion model with noises added
* Calculate a weight for each particle
* Transform the sensor observations from vehicle coordinates to map coordinates
* Associate each observation to the closest landmark
* Weight = Prob of landmark position in a multivariate normal distribution with mean as the observation.
* Sample ‘n’ particles by replacement from the current pool based on their weights
* Go to step 2

Details on running the code are explained below.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

