# 1DOF sprang mass with damper model
Model of sprang mass with damper with control based on the CHRONOproject
The model is represented as a rolling wheel with a mass attached through a spring with a damper.

![alt text](Scheme.png?raw=true "Scheme of the model")

Project structure:

		├── build                   		# build of current CHRONO project (has to be build locally wrt CMakeLists)
		|	├── Release
		|	|	├── SDM.exe 				# Execution file of CHRONO build
		|	|	├── example.txt 			# Text file with output of the simulation
		├── SDM_model_Matlab        		# MATLAB/Simulink files
		|	├── supportFiles 				# Functions for control synth, plot figures and metrics measurement
		|	├── main.m 						# MATLAB executable file to synth controllers and defifnition all elements for Simulinc
		|	├── SDM_CoSim.slx 				# Simulink model with CHRONOcoSimulation module and chosen controller
		|	├── MatlabDriven_SDM_CoSim.slx 	# Simulink model with CHRONOcoSimulation module for OpenLoop response verification
		├── sourceFiles             		# Source files
		|	├── configuratoin.txt 			# Configuration file with definition of the system and simulation parameters
		|	├── MySystem.cpp 				# Class file with code to ascemble the system based on the given configuration
		|	├── MySystem.h 					# Header file for MySystem class
		├── main.cpp 						# main fileof the c++ project 
		├── CMakeLists.txt    				# CMake file

The project is based on the Matlab Simulink control with coSimulation on CHRONO.

CHRONO PROJECT version 9.0 
https://api.projectchrono.org/9.0.0/