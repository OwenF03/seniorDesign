#!/bin/bash

# Clean previous run
bash clean_tests.sh
# Create directories if repo was just cloned
mkdir ./matscripts/test_data > /dev/null
mkdir ./matscripts/vars > /dev/null
mkdir ./src/vars > /dev/null

#Run each program to generate output 
#Matlab script
cd ./matscripts
matlab -nodesktop -nosplash -r "$1;exit;"
cd ..

# Copy Resulting data into scripts folder 
mkdir -p ./scripts/vars/matlab/ > /dev/null
cp ./matscripts/vars/* ./scripts/vars/matlab/
clear
# Build C++ program 
make clean
export MUSIC_SINGLE_FILE="./matscripts/test_data/input_data.mat"
export MUSIC_SINGLE_COV_FILE="./matscripts/test_data/input_data.mat"
make test

./music_test.out  # supress stdout output 
# Copy resulting data into scripts folder 
mkdir -p ./scripts/vars/c/ > /dev/null
cp ./src/vars/* ./scripts/vars/c/

# Run python script
cd ./scripts
clear
python3 unit_test.py 




