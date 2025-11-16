// https://www.wavewalkerdsp.com/2021/12/15/efficient-real-to-complex-conversion-with-a-half-band-filter/
// Based on : https://github.com/msamsami/doa-estimation-music
#include <Eigen/Dense> 
#include <unsupported/Eigen/FFT>
#include <array> //Using for convenient bounds checking 
#include <vector> 
#include <cmath>
#include <algorithm> 



//Paremters 
constexpr int M = 4; // Number of array elements (transducers)
constexpr int N = 2; // Number of sources 
constexpr int numSnapshots = 128; 
constexpr int distance = 1; //Distance in meters 
constexpr float fc[N] = {25000.0, 30000.0}; 
constexpr float cSpeed = 300000000; 

//For sorting results 
struct Peak{
    double val; 
    int idx; 
}; 

//Class designed to implement the MUSIC direction finding algorithm
class DOA{
    private: 
        int fs; // Sampling frequency
        std::array<int, M> positions; // Transducer positions in meters
        
    public: 

        //Default constructor
        DOA(){
            fs = 400000; //400 KHz 
            positions = {0, 1, 2, 3}; //In meters, will be much smaller
        }

        //Constructor with arguments  
        DOA(int fs, std::array<int, M> positions) : fs(fs), positions(positions){}

        // Make real valued signal analytical so it will work with MUSIC algo
        Eigen::VectorXcf realToAnalytic(const Eigen::VectorXf& real_signal) {
            Eigen::FFT<float> fft;
            
            // Forward FFT
            Eigen::VectorXcf freq_domain;
            fft.fwd(freq_domain, real_signal);
            
            int N = real_signal.size();
            
            // Zero out negative frequencies (Hilbert transform)
            for(int i = N/2 + 1; i < N; i++) {
                freq_domain(i) = 0;
            }
            
            // Double positive frequencies (except DC and Nyquist)
            for(int i = 1; i < N/2; i++) {
                freq_domain(i) *= 2.0;
            }
            
            // Inverse FFT
            Eigen::VectorXcf analytic;
            fft.inv(analytic, freq_domain);
            
            return analytic;
        }

        //Estimate DOA for incoming siganls 
        std::vector<float> estimateDOA(std::array<float, M * numSnapshots> inputData){
            
            //Create Eigen matrix from ADC data
            // ADC data is of the form [ch0, ch1, ch2, ch3, ch0, ch1, ch2, ch3 ...], repeats for numSnapshots
            Eigen::MatrixXf input_mat_real(Eigen::Map<Eigen::MatrixXf>(inputData.data(), M, numSnapshots).cast<std::complex<float>>()); 

            // *** NOTE ***
            // Likely need a software bandpass filter before the below call so that the input for the MUSIC algorithm are in fact 
            // narrow band

            //Convert to analytical signal 
            // Convert each channel to analytic signal
            Eigen::MatrixXcf input_mat(M, numSnapshots);
            for(int i = 0; i < M; i++) {
                input_mat.row(i) = realToAnalytic(input_mat_real.row(i));
            }

            // Compute Covariance of transducer data
            Eigen::MatrixXcf R = (input_mat * input_mat.adjoint()) / numSnapshots; 
            
            //Perform eigen value decomposition on the covariance matrix R
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXcf> eig(R); 

            if(eig.info() != Eigen::Success){
                throw std::runtime_error("Eigen Value Decomposition Failed"); 
            }

            Eigen::MatrixXcf eV = eig.eigenvectors(); //Get eigen vectors of 
                                                     //co-variance matrix

            Eigen::MatrixXcf noiseSub = eV.leftCols(M - N); //Extract noise subspace
            
            
            auto DOA_angles = std::vector<float>(N); //Create vector to hold DOA_angles
            
            //Perform peak search
            // NOTE : needs to be expanded so that input signal is filtered before each fc run
            for(int i = 0; i < N; i++){
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

        
        //Take in noise subspace, calculate MUSIC pseudo spectrum
        void genPseudoSpectrum(Eigen::MatrixXcf & noiseSub, double fc, std::vector<struct Peak> & result){
            result = std::vector<struct Peak>();  
            //Create array to use for inner loop calculations
            Eigen::VectorXcf sensor_indicies = Eigen::VectorXcf::LinSpaced(M, 0.0, M - 1.0); 

            float t0 = -2.0 * M_PI * distance * (1.0 / cSpeed); //Intermediate calculation (can be done out of loops)
            float t1 = t0 * fc; // Second intermediate value 
            // Iterate through 0 to 180 degrees
            for(int i = 0; i < 180; i++){
                struct Peak p = {}; 
                p.idx = i; 
                float deg_rad = i * (M_PI / 180.0); 
                float K = t1 * std::cos(deg_rad); 

                Eigen::VectorXcf a = (sensor_indicies.array() * std::complex<float>(0.0, K)).exp(); // line 51 of reference matlab script

                // a(:,i)' * noiseSub
                Eigen::RowVectorXcf projection = a.adjoint() * noiseSub; 
                //norm(a(:i)' * noiseSub)^2
                float denominator = projection.squaredNorm(); 
                //Get sudo spectrum value
                p.val = (1.0 / denominator);  
                result.push_back(p); 
            }
        }
        

};
