# Implementing MUSIC Algorithm in C++
Based on Matlab musicdoa function. Project will work on most UNIX systems, was tested on Ubuntu 24.04

## How to Build

Ensure that Eigen is installed on g++ include path. 
Also ensure that Matlab is installed and the MATLAB_ROOT environment variable is set correctly (so that readMatFile can be compiled)

run <em> make debug </em> in the root directory to build the program (with debug information enabled)

## Structure of Project 

### matscripts

Contains matlab scripts used to generate test values for main code

### scripts

Contains Python script(s) used for unit testing functionality of code

### src

Contains C++ source code 

## Description of Each File

### MUSIC.h

Contains DOA class declaration, definitions in corresponding .cpp file

### MUSIC.cpp

Matlab musicdoa implementation in C++. estimateDOA() operates on float input data (from either ADC on a MCU or test data). estimateDOA_cov() is a debug function that accepts a covariance matrix and will calculate the DOA (used to verify some fo the codes functionality)

If DEBUG is asserted when compiled, content of variables will be displayed to stdout, and TODO variables will be stored into individual text files for unit testing purposes.

### params.h

Contains some parameters for the program. 

### readMatFile.h

Declares functions used to read .mat files into a C++ program for testing

### readMatFile.cpp 

Definition of functions used to read .mat files into a C++ program for testing. Designed to be used exclusively by test.cpp to read in test values. 

### test.cpp

Main testing program. Reads in input data from .mat files (generated from Matlab scripts) and performs calculations on them.

Currently setup to test estimateDOA_cov() and or estimateDOA(). Input data for each can be set by setting MUSIC_SINGLE_COV_FILE 
and or MUSIC_SINGLE_FILE environment variables to the .mat file name or path. 
