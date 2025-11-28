// https://www.wavewalkerdsp.com/2021/12/15/efficient-real-to-complex-conversion-with-a-half-band-filter/
// Based on : https://github.com/msamsami/doa-estimation-music
#include <Eigen/Dense> 
#include <unsupported/Eigen/FFT>
#include <array> //Using for convenient bounds checking 
#include <vector> 
#include <cmath>
#include <iostream>
#include <algorithm> 
#include <float.h>

//Paremters 
constexpr int M = 4; // Number of array elements (transducers). This is fixed
constexpr int N_signals = 3; // Number of sources 
constexpr int numSnapshots = 128; 
constexpr int distance = 1; //Distance in meters 
constexpr float fc[N_signals] = {25000.0, 30000.0}; 
constexpr float cSpeed = 1500; // in m/s
constexpr float lambda = cSpeed / 40000; 
constexpr float d = lambda / 2; 
constexpr double elSpacing = 0.5; //Element spacing in wavelengths, used for steering vector

//For sorting results 
struct Peak{
    float val; 
    int idx; 
}; 

//Class designed to implement the MUSIC direction finding algorithm
class DOA{
    private: 
        int fs; // Sampling frequency
        std::array<double, M> positions; // Transducer positions in meters
        Eigen::MatrixXcf sv; 
        Eigen::VectorXd scanAngles;
        
    public: 

        //Default constructor
        DOA(){
            fs = 400000; //400 KSPS 
            positions = {0, d, 2 * d, 3 * d}; //In meters, will be much smaller
            scanAngles = Eigen::VectorXd::LinSpaced(181, -90, 90); 
            sv = steeringVectorULA(elSpacing, M, scanAngles); 
        }
        
        //Constructor with arguments
        DOA(int fs, std::array<double, M> positions) : fs(fs), positions(positions){
            scanAngles = Eigen::VectorXd::LinSpaced(181, -90, 90); 
            sv = steeringVectorULA(elSpacing, M, scanAngles); 
        }

        DOA(int fs) : fs(fs){
            positions = {0, d, 2 * d, 3 * d}; //In meters, will be much smaller
            scanAngles = Eigen::VectorXd::LinSpaced(181, -90, 90); 
            sv = steeringVectorULA(elSpacing, M, scanAngles); 
        }

        // Make real valued signal analytical so it will work with MUSIC algo
        Eigen::VectorXcf realToAnalytic(const Eigen::VectorXf& real_signal) {
            Eigen::FFT<float> fft;
            
            // Forward FFT
            Eigen::VectorXcf freq_domain;
            fft.fwd(freq_domain, real_signal);
            
            int N_signals = real_signal.size();
            
            // Zero out negative frequencies (Hilbert transform)
            for(int i = N_signals/2 + 1; i < N_signals; i++) {
                freq_domain(i) = 0;
            }
            
            // Double positive frequencies (except DC and Nyquist)
            for(int i = 1; i < N_signals/2; i++) {
                freq_domain(i) *= 2.0;
            }
            
            // Inverse FFT
            Eigen::VectorXcf analytic;
            fft.inv(analytic, freq_domain);
            
            return analytic;
        }

        //Estimate DOA for incoming siganls 
        // input data is a real valued signal
        std::vector<double> estimateDOA(std::array<float, M * numSnapshots> & inputData){
            
            //Create Eigen matrix from ADC data
            // ADC data is of the form [ch0, ch1, ch2, ch3, ch0, ch1, ch2, ch3 ...], repeats for numSnapshots
            Eigen::MatrixXf input_mat_real(Eigen::Map<Eigen::MatrixXf>(inputData.data(), M, numSnapshots)); 

            // *** NOTE ***
            // Likely need a software bandpass filter before the below call so that the input for the MUSIC algorithm are in fact 
            // narrow band

            //Convert to analytical signal 
            // Convert each channel to analytic signal
            Eigen::MatrixXcf input_mat;
            for(int i = 0; i < M; i++) {
                input_mat.row(i) = realToAnalytic(input_mat_real.row(i));
            }

            // Compute Covariance of transducer data
            Eigen::Matrix4cf R = (input_mat * input_mat.adjoint()) / numSnapshots; 
            
            //Perform eigen value decomposition on the covariance matrix R
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix4cf> eig(R); 

            if(eig.info() != Eigen::Success){
                throw std::runtime_error("Eigen Value Decomposition Failed"); 
            }

            Eigen::MatrixXcf eV = eig.eigenvectors(); //Get eigen vectors of 
                                                     //co-variance matrix

            Eigen::MatrixXcf noiseSub = eV.leftCols(M - N_signals); //Extract noise subspace
            
            
            auto DOA_angles = std::vector<double>(N_signals); //Create vector to hold DOA_angles
            
            //Perform peak search
            // NOTE : needs to be expanded so that input signal is filtered before each fc run
            for(int i = 0; i < 1; i++){
                std::vector<struct Peak> res; 
                genPseudoSpectrum(noiseSub, fc[i], res);  
                
                //Search for peaks 
                auto sortFunc = [](const Peak & a, const Peak & b){
                    return a.val > b.val; //Sort in reverse order 
                };

                std::sort(res.begin(), res.end(), sortFunc);
                DOA_angles[i] = res[0].idx; //Index is DOA 
            
            }

            return DOA_angles;
        }

        //Estimate DOA for incoming siganls 
        // input is the covariance matrix of the adc data
        std::vector<struct Peak> estimateDOA_cov(Eigen::Matrix4cf cov){
            

            //Perform eigen value decomposition on the covariance matrix R
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix4cf> eig(cov); 

            if(eig.info() != Eigen::Success){
                throw std::runtime_error("Eigen Value Decomposition Failed"); 
            }
            

            Eigen::VectorXf eigenvalues = eig.eigenvalues().real().cast<float>();
            Eigen::MatrixXcf eigenvectors = eig.eigenvectors();

            std::cout << "Eigen Vectors Pre Sort\n"; 
            std::cout << eigenvectors<< "\n"; 

            
            std::cout << "Eigen Values Pre\n"; 
            std::cout << eigenvalues << "\n"; 

            // Sort eigenvalues in DESCENDING order (like MATLAB)
            std::vector<std::pair<float, int>> eigen_pairs;
            for (int i = 0; i < M; i++) {
                eigen_pairs.push_back(std::make_pair(eigenvalues(i), i));
            }
            
            // Sort descending by eigenvalue
            std::sort(eigen_pairs.begin(), eigen_pairs.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });
            
            // Reorder eigenvectors according to sorted eigenvalues
            Eigen::MatrixXcf eigenvectors_sorted(M, M);
            Eigen::VectorXf eigenvalues_sorted(M);
            
            for (int i = 0; i < M; i++) {
                int original_idx = eigen_pairs[i].second;
                eigenvectors_sorted.col(i) = eigenvectors.col(original_idx);
                eigenvalues_sorted(i) = eigen_pairs[i].first;
            }
    
            std::cout << "Eigenvectors (sorted):\n" << eigenvectors_sorted << "\n";
    
            // Extract noise subspace: LAST (M-Nsig) columns after descending sort
            // These correspond to the smallest eigenvalues
            int num_noise = M - N_signals;
            Eigen::MatrixXcf noise_subspace = eigenvectors_sorted.rightCols(num_noise);
            
            std::cout << "Noise Subspace\n"; 
            std::cout << noise_subspace << "\n"; 
            
            auto DOA_angles = std::vector<double>(N_signals); //Create vector to hold DOA_angles
            
            //Perform peak search
            // NOTE : needs to be expanded so that input signal is filtered before each fc run
            for(int i = 0; i < 1; i++){
                std::vector<struct Peak> res; 
                genPseudoSpectrum(noise_subspace, fc[i], res);  
                 
                //Search for peaks 
                // Print top N peaks
                std::cout << "\n=== TOP " << N_signals << " PEAKS ===\n";
                for (int i = 0; i < std::min(N_signals, (int)res.size()); i++) {
                    std::cout << "Peak " << i+1 << ": Angle = " << (res[i].idx - 90) 
                              << " degrees, Value = " << res[i].val << "\n";
                }
                
                return res;            
            }

            return std::vector<struct Peak>(); 
            //return DOA_angles;
        }

        //Take in noise subspace, calculate MUSIC pseudo spectrum
        void genPseudoSpectrum(Eigen::MatrixXcf & noiseSub, double fc, std::vector<struct Peak> & result){

            std::cout << "Steering Vectors \n"; 
            std::cout  << sv; 
            result = std::vector<struct Peak>();  
            
            // sv' * noise_eigenvects
            // Results in angles x noise eigenvectors matrix
            Eigen::MatrixXcf projection = sv.adjoint() * noiseSub;

            std::cout << "\nProjection (sv' * noise_eigenvects) : \n" << projection; 
        
            // abs(...)^2
            // Element wise square each element
            Eigen::MatrixXf projection_abs_squared = projection.cwiseAbs2();  

            std::cout << "\nProjection abs squared  : \n" << projection_abs_squared; 
            
            // sum(..., 2) - sum across columns 
            // rowwise().sum() results in Mx1 vector where each element is sum along the row
            Eigen::VectorXf D = projection_abs_squared.rowwise().sum(); 

            std::cout << "\nD  : \n" << D; 

            //Add small number to prevent division by zero
            D.array() += FLT_MIN; 

            // spec = sqrt(1./D)
            // .array() allows for element wise operations
            // 1.0f / D.array() computes element wise division
            // .sqrt() element wise sqrt()
            Eigen::VectorXf spec = (1.0f / D.array()).sqrt(); 
            std::cout << "\n Spectrum  : \n" << spec; 

            std::vector<int> peak_indicies = findPeaks(spec, N_signals);  

            for(int i : peak_indicies){
                struct Peak p = {}; 
                p.idx = i; 
                p.val = spec[i];  
                result.push_back(p); 
            }
        }

        // Find local maxima (peaks) in a 1D signal
        // Returns indices of peaks sorted by peak height in descending order
        // Generated with claude
        std::vector<int> findPeaks(const Eigen::VectorXf& signal, int max_peaks = -1) {
            std::vector<std::pair<float, int>> peaks;  // (height, index)
            
            int N = signal.size();
            
            // Find local maxima: signal[i] > signal[i-1] AND signal[i] > signal[i+1]
            for (int i = 1; i < N - 1; i++) {
                if (signal(i) > signal(i-1) && signal(i) > signal(i+1)) {
                    peaks.push_back(std::make_pair(signal(i), i));
                }
            }
            
            // Check boundaries (first and last elements can also be peaks)
            if (N > 1) {
                if (signal(0) > signal(1)) {
                    peaks.push_back(std::make_pair(signal(0), 0));
                }
                if (signal(N-1) > signal(N-2)) {
                    peaks.push_back(std::make_pair(signal(N-1), N-1));
                }
            }
            
            // Sort peaks by height in descending order
            std::sort(peaks.begin(), peaks.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });
            
            // Extract indices
            std::vector<int> peak_indices;
            int num_peaks = (max_peaks > 0) ? std::min(max_peaks, (int)peaks.size()) : peaks.size();
            for (int i = 0; i < num_peaks; i++) {
                peak_indices.push_back(peaks[i].second);
            }
            
            std::cout << "\n=== PEAK FINDING ===\n";
            std::cout << "Total peaks found: " << peaks.size() << "\n";
            std::cout << "Top peaks (index, angle, value):\n";
            for (int i = 0; i < std::min(5, (int)peaks.size()); i++) {
                int idx = peaks[i].second;
                float val = peaks[i].first;
                std::cout << "  Peak " << (i+1) << ": idx=" << idx 
                          << ", angle=" << (idx - 90) << "°, value=" << val << "\n";
            }
            
            return peak_indices;
        }
 

        // Helper function to compute element delays (tau)
        // pos: 3xN matrix of element positions [x;y;z] in meters
        // c: propagation speed (m/s)
        // ang: 2xM matrix of angles [azimuth;elevation] in degrees
        // Generated with claude
        Eigen::MatrixXd computeElementDelay(const Eigen::MatrixXd& pos, 
                                             double c,
                                             const Eigen::MatrixXd& ang) {
            const double DEG2RAD = M_PI / 180.0;
            int N = pos.cols();  // number of elements
            int M = ang.cols();  // number of angles
            
            Eigen::MatrixXd tau(N, M);
            
            for (int m = 0; m < M; m++) {
                double az_rad = ang(0, m) * DEG2RAD;
                double el_rad = ang(1, m) * DEG2RAD;
                
                // Unit vector in direction of arrival
                // In spherical coordinates: [cos(el)*cos(az), cos(el)*sin(az), sin(el)]
                Eigen::Vector3d k_hat;
                k_hat << std::cos(el_rad) * std::cos(az_rad),
                         std::cos(el_rad) * std::sin(az_rad),
                         std::sin(el_rad);
                
                // Compute delay for each element
                for (int n = 0; n < N; n++) {
                    Eigen::Vector3d elem_pos = pos.col(n);
                    // tau = -k_hat' * pos / c
                    tau(n, m) = -k_hat.dot(elem_pos) / c;
                }
            }
            
            return tau;
        }

        // Main steering vector function (scalar frequency version)
        // pos: 3xN matrix of element positions [x;y;z] in meters
        // freq: frequency in Hz
        // c: propagation speed (m/s)
        // ang: 2xM matrix of angles [azimuth;elevation] in degrees
        // generated with claude
        Eigen::MatrixXcf steeringVector(const Eigen::MatrixXd& pos,
                                        double freq,
                                        double c,
                                        const Eigen::MatrixXd& ang) {
            // Compute delays
            Eigen::MatrixXd tau = computeElementDelay(pos, c, ang);
            
            int N = pos.cols();  // number of elements
            int M = ang.cols();  // number of angles
            
            Eigen::MatrixXcf sv(N, M);
            
            // sv = exp(-1j * 2*pi * freq * tau)
            const std::complex<float> j(0.0f, 1.0f);
            const float two_pi_freq = 2.0f * M_PI * static_cast<float>(freq);
            
            for (int m = 0; m < M; m++) {
                for (int n = 0; n < N; n++) {
                    float phase = two_pi_freq * static_cast<float>(tau(n, m));
                    sv(n, m) = std::exp(j * phase);
                }
            }
            
            return sv;
        }

        // Simplified version for ULA (Uniform Linear Array) with broadside angles only
        // This is what musicdoa uses internally
        // elementSpacing: element spacing in wavelengths (typically 0.5)
        // N_elements: number of array elements
        // scanAngles: 1xM vector of broadside angles (in degrees)
        // Generated with calude
        Eigen::MatrixXcf steeringVectorULA(double elementSpacing,
                                            int N_elements,
                                            const Eigen::VectorXd& scanAngles) {
            const double DEG2RAD = M_PI / 180.0;
            int M = scanAngles.size();  // number of angles
            
            Eigen::MatrixXcf sv(N_elements, M);
            
            const std::complex<float> j(0.0f, 1.0f);
            const float two_pi = 2.0f * M_PI;
            
            // For broadside (ULA), steering vector is:
            // sv = exp(-1j * 2*pi * elementPos * sin(angle))
            // where elementPos[n] = n * elementSpacing
            
            for (int m = 0; m < M; m++) {
                float sin_angle = std::sin(static_cast<float>(scanAngles(m) * DEG2RAD));
                
                for (int n = 0; n < N_elements; n++) {
                    float elementPos = n * elementSpacing;
                    float phase = two_pi * elementPos * sin_angle;
                    sv(n, m) = std::exp(j * phase);
                }
            }
            
            return sv;
        }

        // For use with actual physical parameters 
        // generated with Calude
        Eigen::MatrixXcf steeringVectorPhysical(int N_elements,
                                                double freq,
                                                double c,
                                                const std::vector<double>& doa_degrees) {
            // Compute wavelength
            double lambda = c / freq;
            double d = lambda / 2.0;  // half-wavelength spacing
            
            // Create 3xN position matrix (ULA along x-axis)
            Eigen::MatrixXd pos(3, N_elements);
            pos.setZero();
            for (int i = 0; i < N_elements; i++) {
                pos(0, i) = i * d;  // x position
                // y and z are zero
            }
            
            // Create 2xM angle matrix [azimuth; elevation]
            int M = doa_degrees.size();
            Eigen::MatrixXd ang(2, M);
            for (int i = 0; i < M; i++) {
                ang(0, i) = doa_degrees[i];  // azimuth
                ang(1, i) = 0.0;              // elevation (broadside)
            }
            
            return steeringVector(pos, freq, c, ang);
        }

       

};
