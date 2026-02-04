#!/bin/bash

# Remove files generated during test process
make clean
rm -r ./src/vars/
rm -r ./matscripts/test_data/
rm -r ./matscripts/vars/
rm -r ./scripts/vars/
rm ./scripts/*.out
