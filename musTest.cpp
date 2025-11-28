#ifdef DEBUG
#include "music_single_debug.h"
#else 
#include "music_single.h"
#endif
#include "readMatFile.h"
#include <string>
#include <iostream>
#include <stdio.h>
#include <complex>
#include <array>

#define NUM_SNAPSHOTS 128

void readSignalData(std::string & fn, std::array<float, 4 * NUM_SNAPSHOTS> & sig); 


int main(){
    DOA estimator = DOA(400000); 

    Eigen::MatrixXcf cov = extractVarToEigen("R_xx.mat", std::string("R_xx"));

    auto res = estimator.estimateDOA_cov(cov);

    //Search for peaks 
    // Print top N peaks
    std::cout << "\n=== TOP " << N_signals << " PEAKS ===\n";
    for (int i = 0; i < std::min(N_signals, (int)res.size()); i++) {
        std::cout << "Peak " << i+1 << ": Angle = " << (res[i].idx - 90) 
                  << " degrees, Value = " << res[i].val << "\n";
    }  
    return 0; 
}
