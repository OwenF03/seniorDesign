#ifndef PARAMS_H
#define PARAMS_H

constexpr int M = 4; // Number of array elements (transducers). This is fixed
constexpr int N_signals = 2; // Number of sources 
constexpr int numSnapshots = 128; // Number of measurments from each transducer
constexpr int distance = 1; //Distance in meters 
constexpr float fc[N_signals] = {30000.0, 30000.0}; 
constexpr float cSpeed = 1500; // in m/s
constexpr float lambda = cSpeed / 40000; 
constexpr float d = lambda / 2; 
constexpr double elSpacing = 0.5; //Element spacing in wavelengths, used for steering vector


#endif
