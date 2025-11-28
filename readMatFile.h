#ifndef READMATFILE_H
#define READMATFILE_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <complex>
#include <Eigen/Dense>
#include <unordered_map>
#include "mat.h" 
#include "matrix.h"

//Function to open .mat file
MATFile * openMatFile(const char *f);
//Function to read headers from .mat file
void readHeaders(MATFile * f, std::unordered_map<std::string, mxArray *> &vars);
//File to convert array into complex valued eigen matrix
Eigen::MatrixXcf mxArrayToEigenMatrixcf(const mxArray* pa);
//Extract variableName stored in file fn into a complex valued eigen matrix
Eigen::MatrixXcf extractVarToEigen(const char * fn, const std::string & variableName);

#endif
