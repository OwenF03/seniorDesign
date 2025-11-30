#include "readMatFile.h"

//Function to open .mat file
MATFile * openMatFile(const char *f){
    
    MATFile * file = matOpen(f, "r"); 
    if(file == NULL){
        printf("Error opening .mat file : %s", file); 
        exit(1); 
    }

    return file;
}

void readHeaders(MATFile * f, std::unordered_map<std::string, mxArray *> &vars){
    const char * name;  
    mxArray * dat; 
    //Read variables from file
    while(dat = matGetNextVariableInfo(f, &name)){
        vars.emplace(std::pair<std::string, mxArray*>(std::string(name), dat)); //Add element to map
    }   
}

std::vector<float> mxArrayToFloatVector(const mxArray* pa){
    mwSize num_dims = mxGetNumberOfDimensions(pa); 
    const mwSize* dims = mxGetDimensions(pa); 

    mwSize rows = dims[0]; 
    mwSize cols = (num_dims > 1) ? dims[1] : 1; 

    std::vector<float> vec; 

    if (mxIsComplex(pa)) {
        // Handle complex data based on actual data type
        if (mxIsSingle(pa)) {
            // Single precision complex
            mxComplexSingle* complex_data = mxGetComplexSingles(pa);
            if (complex_data == nullptr) {
                throw std::runtime_error("Failed to get complex single data");
            }
            for (mwSize row = 0; row < rows; row++) {
                for (mwSize col = 0; col < cols; col++) {
                    mwSize idx = row + col * rows;
                    vec.emplace_back(complex_data[idx].real); 
                }
            }
        } else if (mxIsDouble(pa)) {
            // Double precision complex (most common case)
            mxComplexDouble* complex_data = mxGetComplexDoubles(pa);
            if (complex_data == nullptr) {
                throw std::runtime_error("Failed to get complex double data");
            }
            for (mwSize row = 0; row < rows; row++) {
                for (mwSize col = 0; col < cols; col++) {
                    mwSize idx = row + col * rows;
                    vec.emplace_back(complex_data[idx].real); 
                }
            }
        } else {
            throw std::runtime_error("Unsupported complex data type: " + std::string(mxGetClassName(pa)));
        }
    } else {
        // Handle real data
        if (mxIsSingle(pa)) {
            float* real_data = mxGetSingles(pa);
            if (real_data == nullptr) {
                throw std::runtime_error("Failed to get single precision data");
            }
            for (mwSize row = 0; row < rows; row++) {
                for (mwSize col = 0; col < cols; col++) {
                    mwSize idx = row + col * rows;
                    vec.emplace_back(real_data[idx]);
                }
            }
        } else if (mxIsDouble(pa)) {
            double* real_data = mxGetDoubles(pa);
            if (real_data == nullptr) {
                throw std::runtime_error("Failed to get double precision data");
            }
            for (mwSize row = 0; row < rows; row++) {
                for (mwSize col = 0; col < cols; col++) {
                    mwSize idx = row + col * rows;
                    vec.emplace_back(real_data[idx]);
                }
            }
        } else {
            throw std::runtime_error("Unsupported real data type: " + std::string(mxGetClassName(pa)));
        }
    }

    return vec; 
}

Eigen::MatrixXcf mxArrayToEigenMatrixcf(const mxArray* pa){
    mwSize num_dims = mxGetNumberOfDimensions(pa); 
    const mwSize* dims = mxGetDimensions(pa); 

    mwSize rows = dims[0]; 
    mwSize cols = (num_dims > 1) ? dims[1] : 1; 

    Eigen::MatrixXcf eigen_mat(rows, cols); 

    if (mxIsComplex(pa)) {
        // Handle complex data based on actual data type
        if (mxIsSingle(pa)) {
            // Single precision complex
            mxComplexSingle* complex_data = mxGetComplexSingles(pa);
            if (complex_data == nullptr) {
                throw std::runtime_error("Failed to get complex single data");
            }
            for (mwSize col = 0; col < cols; col++) {
                for (mwSize row = 0; row < rows; row++) {
                    mwSize idx = row + col * rows;
                    eigen_mat(row, col) = std::complex<float>(complex_data[idx].real, 
                                                              complex_data[idx].imag);
                }
            }
        } else if (mxIsDouble(pa)) {
            // Double precision complex (most common case)
            mxComplexDouble* complex_data = mxGetComplexDoubles(pa);
            if (complex_data == nullptr) {
                throw std::runtime_error("Failed to get complex double data");
            }
            for (mwSize col = 0; col < cols; col++) {
                for (mwSize row = 0; row < rows; row++) {
                    mwSize idx = row + col * rows;
                    eigen_mat(row, col) = std::complex<float>(
                        static_cast<float>(complex_data[idx].real), 
                        static_cast<float>(complex_data[idx].imag)
                    );
                }
            }
        } else {
            throw std::runtime_error("Unsupported complex data type: " + std::string(mxGetClassName(pa)));
        }
    } else {
        // Handle real data
        if (mxIsSingle(pa)) {
            float* real_data = mxGetSingles(pa);
            if (real_data == nullptr) {
                throw std::runtime_error("Failed to get single precision data");
            }
            for (mwSize col = 0; col < cols; col++) {
                for (mwSize row = 0; row < rows; row++) {
                    mwSize idx = row + col * rows;
                    eigen_mat(row, col) = std::complex<float>(real_data[idx], 0.0f);
                }
            }
        } else if (mxIsDouble(pa)) {
            double* real_data = mxGetDoubles(pa);
            if (real_data == nullptr) {
                throw std::runtime_error("Failed to get double precision data");
            }
            for (mwSize col = 0; col < cols; col++) {
                for (mwSize row = 0; row < rows; row++) {
                    mwSize idx = row + col * rows;
                    eigen_mat(row, col) = std::complex<float>(
                        static_cast<float>(real_data[idx]), 0.0f
                    );
                }
            }
        } else {
            throw std::runtime_error("Unsupported real data type: " + std::string(mxGetClassName(pa)));
        }
    }

    return eigen_mat; 
}


Eigen::MatrixXcf extractVarToEigen(const char * fn, const std::string & variableName){

    MATFile * matfile = openMatFile(fn); 

    mxArray* data = matGetVariable(matfile, variableName.c_str()); 
    if(data == NULL){
        matClose(matfile); 
        printf("Variable Not found\n");
        exit(-1); 
    }

    auto res = mxArrayToEigenMatrixcf(data); 

    mxDestroyArray(data);
    matClose(matfile); 

    return res; 

}

std::vector<float> extractVarToVec(const char * fn, const std::string & variableName){
    MATFile * matfile = openMatFile(fn); 

    mxArray* data = matGetVariable(matfile, variableName.c_str()); 
    if(data == NULL){
        matClose(matfile); 
        printf("Variable Not found\n");
        exit(-1); 
    }

    auto res = mxArrayToFloatVector(data); 

    mxDestroyArray(data);
    matClose(matfile); 

    return res; 


}
