#include "BinaryThresholding.hpp"
#include "FAST/Data/Image.hpp"

namespace fast {

void BinaryThresholding::setLowerThreshold(float threshold) {
    mLowerThreshold = threshold;
    mLowerThresholdSet = true;
    mIsModified = true;
}

void BinaryThresholding::setUpperThreshold(float threshold) {
    mUpperThreshold = threshold;
    mUpperThresholdSet = true;
    mIsModified = true;
}

void BinaryThresholding::loadAttributes() {
    auto lowerThreshold = getFloatAttribute("lower-threshold");
    auto upperThreshold = getFloatAttribute("upper-threshold");
    if(!std::isnan(lowerThreshold))
        setLowerThreshold(lowerThreshold);
    if(!std::isnan(upperThreshold))
        setUpperThreshold(upperThreshold);
}

BinaryThresholding::BinaryThresholding() {
    mLowerThresholdSet = false;
    mUpperThresholdSet = false;
    createInputPort<Image>(0);
    createOutputPort<Image>(0);
    createOpenCLProgram(Config::getKernelSourcePath() + "Algorithms/BinaryThresholding/BinaryThresholding3D.cl", "3D");
    createOpenCLProgram(Config::getKernelSourcePath() + "Algorithms/BinaryThresholding/BinaryThresholding2D.cl", "2D");

    createFloatAttribute("lower-threshold", "Lower threshold", "Lower intensity threshold", std::nanf(""));
    createFloatAttribute("upper-threshold", "Upper threshold", "Upper intensity threshold", std::nanf(""));
}

BinaryThresholding::BinaryThresholding(float lowerThreshold, float upperThreshold) : BinaryThresholding() {
    setLowerThreshold(lowerThreshold);
    setUpperThreshold(upperThreshold);
}

void BinaryThresholding::execute() {
    if(!mLowerThresholdSet && !mUpperThresholdSet) {
        throw Exception("BinaryThresholding need at least one threshold to be set.");
    }

    auto input = getInputData<Image>(0);
    auto output = Image::createSegmentationFromImage(input);

    if(getMainDevice()->isHost()) {
        throw Exception("Not implemented yet.");
    } else {
        OpenCLDevice::pointer device = std::dynamic_pointer_cast<OpenCLDevice>(getMainDevice());
        cl::Program program;
        if(input->getDimensions() == 3) {
            program = getOpenCLProgram(device, "3D");
        } else {
            program = getOpenCLProgram(device, "2D");
        }
        cl::Kernel kernel;
        if(mLowerThresholdSet && mUpperThresholdSet) {
            kernel = cl::Kernel(program, "thresholding");
            kernel.setArg(2, mLowerThreshold);
            kernel.setArg(3, mUpperThreshold);
        } else if(mLowerThresholdSet) {
            kernel = cl::Kernel(program, "thresholdingWithOnlyLower");
            kernel.setArg(2, mLowerThreshold);
        } else {
            kernel = cl::Kernel(program, "thresholdingWithOnlyUpper");
            kernel.setArg(2, mUpperThreshold);
        }
        cl::NDRange globalSize;
        OpenCLImageAccess::pointer access = input->getOpenCLImageAccess(ACCESS_READ, device);
        if(input->getDimensions() == 2) {
            OpenCLImageAccess::pointer access2 = output->getOpenCLImageAccess(ACCESS_READ_WRITE, device);
            kernel.setArg(0, *access->get2DImage());
            kernel.setArg(1, *access2->get2DImage());
            globalSize = cl::NDRange(output->getWidth(), output->getHeight());
        } else {
            // TODO no 3d image write support
            OpenCLImageAccess::pointer access2 = output->getOpenCLImageAccess(ACCESS_READ_WRITE, device);
            kernel.setArg(0, *access->get3DImage());
            kernel.setArg(1, *access2->get3DImage());
            globalSize = cl::NDRange(output->getWidth(), output->getHeight(), output->getDepth());
        }

        cl::CommandQueue queue = device->getCommandQueue();
        queue.enqueueNDRangeKernel(
                kernel,
                cl::NullRange,
                globalSize,
                cl::NullRange
        );
    }
    addOutputData(0, output);
}

void BinaryThresholding::waitToFinish() {
    if(!getMainDevice()->isHost()) {
        OpenCLDevice::pointer device = std::static_pointer_cast<OpenCLDevice>(getMainDevice());
        device->getCommandQueue().finish();
    }
}

} // end namespace fast
