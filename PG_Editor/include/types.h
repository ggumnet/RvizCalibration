#ifndef TYPES_H
#define TYPES_H

using pointcloud_tools::NDTOctreeGPU;
using NDTMatcherGPU = pointcloud_tools::NDTMatcherGPU<NDTOctreeGPU>;
constexpr std::size_t DIM = NDTOctreeGPU::DIM;
constexpr std::size_t PARAM_DIM = NDTMatcherGPU::PARAM_DIM;
using Scalar = typename NDTOctreeGPU::Scalar;
using Vector = typename NDTOctreeGPU::Vector;
using Matrix = typename NDTOctreeGPU::Matrix;
using VectorSet = typename NDTOctreeGPU::VectorSet;
using Transform = typename NDTOctreeGPU::Transform;
using ParamMatrix = typename NDTMatcherGPU::ParamMatrix;
using ParamVector = typename NDTMatcherGPU::ParamVector;
using IndexScalar = typename NDTOctreeGPU::IndexScalar;


#endif