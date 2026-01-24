#ifndef MUSIC_H
#define MUSIC_H

#include <Eigen/Dense> 
#include <unsupported/Eigen/FFT>
#include <array> //Using for convenient bounds checking 
#include <vector> 
#include <cmath>
#include <iostream>
#include <algorithm> 
#include <float.h>
#include "params.h"

//For sorting results 
struct Peak{
    float val; //  Value being sorted
    int idx;   // Index in spectrum list (DOA)
}; 

//Class designed to implement the MUSIC direction finding algorithm
class DOA{
    private: 
        int fs; // Sampling frequency
        std::array<double, M> positions; // Transducer positions in meters
        Eigen::MatrixXcf sv; // Steering vector 
        Eigen::VectorXd scanAngles; //Array containing scan angles (in increments of 1)
        
    public: 

        //Constructors
        DOA();
        DOA(int fs, std::array<double, M> positions);
        DOA(int fs);

        // Make real valued signal analytical so it will work with MUSIC algo
        Eigen::VectorXcf realToAnalytic(const Eigen::VectorXf& real_signal);  

        //Estimate DOA for incoming siganls 
        // input data is a real valued signal
        std::vector<Peak> estimateDOA(float inputData[]);

        //Estimate DOA for incoming siganls 
        // input is the covariance matrix of the adc data
        // For debug purposes
        std::vector<struct Peak> estimateDOA_cov(Eigen::Matrix4cf cov);

        //Take in noise subspace, calculate MUSIC pseudo spectrum
        void genPseudoSpectrum(Eigen::MatrixXcf & noiseSub, double fc, std::vector<struct Peak> & result);

        // Find local maxima (peaks) in a 1D signal
        // Returns indices of peaks sorted by peak height in descending order
        // Generated with claude
        std::vector<int> findPeaks(const Eigen::VectorXf& signal, int max_peaks = -1);  

        // Helper function to compute element delays (tau)
        // pos: 3xN matrix of element positions [x;y;z] in meters
        // c: propagation speed (m/s)
        // ang: 2xM matrix of angles [azimuth;elevation] in degrees
        // Generated with claude
        Eigen::MatrixXd computeElementDelay(const Eigen::MatrixXd& pos, double c,const Eigen::MatrixXd& ang);

        // Main steering vector function (scalar frequency version)
        // pos: 3xN matrix of element positions [x;y;z] in meters
        // freq: frequency in Hz
        // c: propagation speed (m/s)
        // ang: 2xM matrix of angles [azimuth;elevation] in degrees
        // generated with claude
        Eigen::MatrixXcf steeringVector(const Eigen::MatrixXd& pos,double freq,double c,const Eigen::MatrixXd& ang);

        // Simplified version for ULA (Uniform Linear Array) with broadside angles only
        // This is what musicdoa uses internally
        // elementSpacing: element spacing in wavelengths (typically 0.5)
        // N_elements: number of array elements
        // scanAngles: 1xM vector of broadside angles (in degrees)
        // Generated with calude
        Eigen::MatrixXcf steeringVectorULA(double elementSpacing,int N_elements,const Eigen::VectorXd& scanAngles);

        // For use with actual physical parameters 
        // generated with Calude
        Eigen::MatrixXcf steeringVectorPhysical(int N_elements, double freq,double c,const std::vector<double>& doa_degrees);

};

#endif
