This directory runs a simulation environment with a number of robots and luggages.

Before running, please pip-install the following libraries:
	'Simpy'
	'Arcade'
	'Pathfinding'

To execute, simply run the Simulation.py file. 

In the beginning of the Simulation.py file, there are 3 configurable paramaters: 
	NUM_ROBOTS (amount of robots, must be <= NUM_LUGGAGE) 
	NUM_LUGGAGE (amount of luggages, must be <= 65 because of the grid layout) 
	AVG_STEP_TIME (time taken per robot step)

These parameters can be freely adjusted for the wanted modelling

Additionaly, in case of being unable to run the code for whatever reason, an executable file 'Simulation.exe' has 
been created in the \dist folder, with a pre-set configuration of:
	- NUM_ROBOTS = 10
	- NUM_LUGGAGE = 30
	- AVG_STEP_TIME = 1 / 120

The 'Simulation.exe' program should give the viewer an overview of how the visualization looks like.