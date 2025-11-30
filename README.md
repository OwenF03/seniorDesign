# Implementing MUSIC Algorithm in C++
Based on Matlab musicdoa function

## How to Build

Ensure that Eigen is installed and on your g++ include path. 
Also ensure that Matlab is installed and the MATLAB_ROOT environment variable is set correctly (so that readMatFile can be compiled)

run <em> make debugSingle </em> in the root directory to compile the single incoming frequency version of the program

## Structure


### musTest.cpp

Main testing program. Reads in input data from .mat files (generated from Matlab scripts) and performs calculations on them.

Currently setup to test estimateDOA_cov() and or estimateDOA(). Input data for each can be set by setting MUSIC_SINGLE_COV_FILE 
and or MUSIC_SINGLE_FILE environment variables to the .mat file name or path. 

### MUSIC_single

Perform MUSIC DOA algorithm on signals arriving at a single carrier frequency. 

### MUSIC

Main implementation, will filter input signals before performing DOA calculation in order to improve accuracy when there are two sources at
different frequencies (WIP)

### readMatFile

Read in variables from .mat file. Used for reading in data from Matlab. 

### params.h

Store parameters used by other parts of the project

### Matlab scripts

What was used to generate input/simulation data 
