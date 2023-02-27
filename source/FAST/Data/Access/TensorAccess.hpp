#pragma once

#include <eigen3/unsupported/Eigen/CXX11/Tensor>
#include <FAST/Object.hpp>
#include <FAST/Data/TensorShape.hpp>

namespace fast {

// Rename the Eigen float tensor for simplicity
template<int NumDimensions>
using TensorData = Eigen::TensorMap<Eigen::Tensor<float, NumDimensions, Eigen::RowMajor, int>>;

class Tensor;

class FAST_EXPORT TensorAccess {
    public:
        typedef std::unique_ptr<TensorAccess> pointer;
        TensorAccess(float* data, TensorShape shape, std::shared_ptr<Tensor> tensor);
        float * getRawData();
        TensorShape getShape() const;
        ~TensorAccess();
        void release();
        template <int NumDimensions>
        TensorData<NumDimensions> getData() const;
    private:
        std::shared_ptr<Tensor> m_tensor;
        TensorShape m_shape;
        float* m_data;
};


template <int NumDimensions>
TensorData<NumDimensions> TensorAccess::getData() const {
    if(NumDimensions != m_shape.getDimensions())
        throw Exception("Dimension mismatch for Eigen tensor in TensorAccess::getData<#Dimension>().");

    // Construct eigen shape
    Eigen::array<int, NumDimensions> sizes;
    for(int i = 0; i < m_shape.getDimensions(); ++i)
        sizes[i] = m_shape[i];

    // Create and return mapped eigen tensor
    return TensorData<NumDimensions>(m_data, sizes);
}



}