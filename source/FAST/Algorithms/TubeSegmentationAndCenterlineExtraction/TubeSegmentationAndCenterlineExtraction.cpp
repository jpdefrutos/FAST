#include "TubeSegmentationAndCenterlineExtraction.hpp"
#include "FAST/Data/Image.hpp"
#include "FAST/Data/Mesh.hpp"
#include "FAST/Algorithms/GaussianSmoothing/GaussianSmoothing.hpp"
#include "FAST/Algorithms/GradientVectorFlow/EulerGradientVectorFlow.hpp"
#include "FAST/Algorithms/GradientVectorFlow/MultigridGradientVectorFlow.hpp"
#include "RidgeTraversalCenterlineExtraction.hpp"
#include "InverseGradientSegmentation.hpp"
#include <stack>

namespace fast {
TubeSegmentationAndCenterlineExtraction::TubeSegmentationAndCenterlineExtraction(float sensitivity, float minimumRadius,
                                                                                 float maximumRadius, float radiusStep,
                                                                                 bool extractBrightTupes,
                                                                                 bool keepLargestTreeOnly) {
    createInputPort<Image>(0);
    createOutputPort<Image>(0);
    createOutputPort<Mesh>(1);
    createOutputPort<Image>(2);

    createOpenCLProgram(Config::getKernelSourcePath() + "Algorithms/TubeSegmentationAndCenterlineExtraction/TubeSegmentationAndCenterlineExtraction.cl");

    mSensitivity = 0.5;
    mMinimumRadius = 1;
    mMaximumRadius = 5;
    mRadiusStep = 0.5;
    mExtractDarkStructures = false;
    mSegmentation = true;
    mThresholdCropping = false;
    mLungCropping = false;
    mMinimumIntensity = -std::numeric_limits<float>::max();
    mMaximumIntensity = std::numeric_limits<float>::max();
    // Blur has to be adapted to noise level in image
    mStDevBlurSmall = 0.5;
    mStDevBlurLarge = 1.0;
    mOnlyKeepLargestTree = false;
    mMinimumTreeSize = -1;
    setSensitivity(sensitivity);
    setMinimumRadius(minimumRadius);
    setMaximumRadius(maximumRadius);
    setRadiusStep(radiusStep);
    if(!extractBrightTupes)
        extractDarkTubes();
    setKeepLargestTree(keepLargestTreeOnly);
}

void TubeSegmentationAndCenterlineExtraction::setMinimumRadius(
        float radius) {
    mMinimumRadius = radius;
}

void TubeSegmentationAndCenterlineExtraction::setMaximumRadius(
        float radius) {
    mMaximumRadius = radius;
}

void TubeSegmentationAndCenterlineExtraction::setRadiusStep(float step) {
    mRadiusStep = step;
}

void TubeSegmentationAndCenterlineExtraction::setSensitivity(
        float sensitivity) {
    mSensitivity = sensitivity;
    if(mSensitivity == 0) {
        mSensitivity = 0.001;
    } else if(mSensitivity == 1) {
        mSensitivity = 0.999;
    }
}

void TubeSegmentationAndCenterlineExtraction::setMinimumIntensity(
        float intensity) {
    mMinimumIntensity = intensity;
}

void TubeSegmentationAndCenterlineExtraction::setMaximumIntensity(
        float intensity) {
    mMaximumIntensity = intensity;
}

void TubeSegmentationAndCenterlineExtraction::extractDarkTubes() {
    mExtractDarkStructures = true;
}

void TubeSegmentationAndCenterlineExtraction::extractBrightTubes() {
    mExtractDarkStructures = false;
}

void TubeSegmentationAndCenterlineExtraction::disableSegmentation() {
    mSegmentation = false;
}

void TubeSegmentationAndCenterlineExtraction::enableSegmentation() {
    mSegmentation = true;
}

void TubeSegmentationAndCenterlineExtraction::disableAutomaticCropping() {
    mThresholdCropping = false;
    mLungCropping = false;
}

void TubeSegmentationAndCenterlineExtraction::enableAutomaticCropping(
        bool lungCropping) {
    if(lungCropping) {
        mLungCropping = true;
    } else {
        mThresholdCropping = true;
    }
}

DataChannel::pointer TubeSegmentationAndCenterlineExtraction::getSegmentationOutputPort() {
    return getOutputPort(0);
}

DataChannel::pointer TubeSegmentationAndCenterlineExtraction::getCenterlineOutputPort() {
    return getOutputPort(1);
}

DataChannel::pointer TubeSegmentationAndCenterlineExtraction::getTDFOutputPort() {
    return getOutputPort(2);
}

inline std::vector<Vector3i> floodFill(ImageAccess::pointer& access, Vector3ui size, Vector3i startPosition) {
    std::vector<Vector3i> thisObject;

    std::stack<Vector3i> stack;
    stack.push(startPosition);
    uchar* segmentation = (uchar*)access->get();
    while(!stack.empty()) {
        Vector3i currentPosition = stack.top();
        stack.pop();
        thisObject.push_back(currentPosition);

        for(int a = -1; a < 2; a++) {
        for(int b = -1; b < 2; b++) {
        for(int c = -1; c < 2; c++) {
            Vector3i position = currentPosition + Vector3i(a,b,c);
            try {
                uchar value = access->getScalar(position); // need out of bounds check here
                if(value == 1) {
                    segmentation[position.x() + position.y()*size.x() + position.z()*size.x()*size.y()] = 0;
                    stack.push(position);
                }
            } catch(Exception &e) {
                // Out of bounds..
            }
        }}}
    }

    return thisObject;
}

void TubeSegmentationAndCenterlineExtraction::keepLargestObjects(Image::pointer segmentation, Mesh::pointer& centerlines) {
    ImageAccess::pointer access = segmentation->getImageAccess(ACCESS_READ_WRITE);
    const int width = segmentation->getWidth();
    const int height = segmentation->getHeight();
    const int depth = segmentation->getDepth();
    Vector3ui size = segmentation->getSize();

    std::vector<Vector3i> largestObject;
    uchar* segmentationArray = (uchar*)access->get();
    // Go through each voxel
    for(int z = 0; z < depth; ++z) {
    for(int y = 0; y < height; ++y) {
    for(int x = 0; x < width; ++x) {
        // If label found, do flood fill, count size, and keep all voxels in a structure
        Vector3i position(x,y,z);
        uchar value = segmentationArray[position.x() + position.y()*size.x() + position.z()*size.x()*size.y()];
        if(value == 1) {
            std::vector<Vector3i> newObject = floodFill(access, size, position);
            if(mOnlyKeepLargestTree) {
                if(newObject.size() > largestObject.size())
                    largestObject = newObject;
            } else {
                if(mMinimumTreeSize > 0) {
                    largestObject.insert(largestObject.end(), newObject.begin(), newObject.end());
                } else {
                    largestObject.insert(largestObject.end(), newObject.begin(), newObject.end());
                }
            }
        }
    }}}

    Reporter::info() << "Size of largest object: " << largestObject.size() << Reporter::end();

    // Store the largest object
    for(Vector3i position : largestObject) {
        segmentationArray[position.x() + position.y()*size.x() + position.z()*size.x()*size.y()] = 1;
    }

    std::vector<MeshVertex> vertices;
    std::vector<MeshLine> lines;
    {
        // Remove centerlines of small objects as well
        MeshAccess::pointer lineAccess = centerlines->getMeshAccess(ACCESS_READ);
        Vector3f spacing = segmentation->getSpacing();
        uint j = 0;
        for(uint i = 0; i < centerlines->getNrOfLines(); ++i) {
            MeshLine line = lineAccess->getLine(i);
            MeshVertex pointA = lineAccess->getVertex(line.getEndpoint1());
            MeshVertex pointB = lineAccess->getVertex(line.getEndpoint2());
            Vector3f pointAinVoxelSpace(pointA.getPosition().x() / spacing.x(), pointA.getPosition().y() / spacing.y(), pointA.getPosition().z() / spacing.z());
            if(access->getScalar(pointAinVoxelSpace.cast<int>()) == 1) {
                vertices.push_back(MeshVertex(pointA));
                vertices.push_back(MeshVertex(pointB));
                lines.push_back(MeshLine(j, j + 1));
                j += 2;
            }
        }
        Reporter::info() << "Size of new centerlines " << vertices.size() << Reporter::end();
    }
    auto newCenterlines = Mesh::create(vertices, lines);
    centerlines = newCenterlines;
    SceneGraph::setParentNode(centerlines, segmentation);
}


void TubeSegmentationAndCenterlineExtraction::execute() {
    Image::pointer input = getInputData<Image>();
    Vector3f spacing = input->getSpacing();
    Vector3ui size = input->getSize();
    float smallestSpacing = spacing.minCoeff();
    float largestSpacing = spacing.maxCoeff();

    // TODO automatic cropping
    if(mLungCropping) {
        // Cut away 20% on all sides
        float fraction = 0.2;
        uint startX = round(input->getWidth()*fraction);
        uint sizeX = input->getWidth() - round(input->getWidth()*fraction*2);
        uint startY = round(input->getHeight()*fraction);
        uint sizeY = input->getHeight() - round(input->getHeight()*fraction*2);
        uint startZ = round(input->getDepth()*fraction);
        uint sizeZ = input->getDepth() - round(input->getDepth()*fraction*2);

        // Make size dividable by 4
        sizeX = sizeX + (4 - sizeX % 4);
        sizeY = sizeY + (4 - sizeY % 4);
        sizeZ = sizeZ + (4 - sizeZ % 4);

        Image::pointer croppedImage = input->crop(Vector3i(startX, startY, startZ), Vector3i(sizeX, sizeY, sizeZ));
        input = croppedImage;
    }

    // If max radius is larger than 2.5 voxels
    Image::pointer gradients;
    Image::pointer largeTDF;
    Image::pointer largeRadius;
    Image::pointer GVFfield;
    if(mMaximumRadius /*/ largestSpacing*/ >= 1.5) {
        reportInfo() << "Running large TDF" << reportEnd();
        // Find large structures, if max radius is large enough
        // Blur
        Image::pointer smoothedImage;
        if(mStDevBlurLarge > 0.1) {
            GaussianSmoothing::pointer filter = GaussianSmoothing::New();
            filter->setInputData(input);
            filter->setStandardDeviation(mStDevBlurLarge);
            //filter->setMaskSize(7);
            filter->setOutputType(TYPE_FLOAT);
            DataChannel::pointer port = filter->getOutputPort();
            filter->update();
            smoothedImage = port->getNextFrame<Image>();
            smoothedImage->setSpacing(spacing);
        } else {
            smoothedImage = input;
        }
        reportInfo() << "finished smoothing" << Reporter::end();

        // Create gradients and cap intensity
        GVFfield = createGradients(smoothedImage);
        reportInfo() << "finished gradients" << Reporter::end();

        // GVF
        GVFfield = runGradientVectorFlow(GVFfield);

        // TDF
        runNonCircularTubeDetectionFilter(GVFfield, 2.5, mMaximumRadius, largeTDF, largeRadius);
    }

    // If min radius is larger than 2.5 voxels
    Image::pointer smallTDF;
    Image::pointer smallRadius;
    if(mMinimumRadius /*/ smallestSpacing*/ < 1.5) {
        reportInfo() << "Running small TDF" << reportEnd();
        // Find small structures
        // Blur
        Image::pointer smoothedImage;
        if(mStDevBlurSmall > 0.1) {
            GaussianSmoothing::pointer filter = GaussianSmoothing::New();
            filter->setInputData(input);
            filter->setStandardDeviation(mStDevBlurSmall);
            //filter->setMaskSize(7);
            filter->setOutputType(TYPE_FLOAT);
            auto port = filter->getOutputPort();
            filter->update();
            smoothedImage = port->getNextFrame<Image>();
            smoothedImage->setSpacing(spacing);
        } else {
            smoothedImage = input;
        }

        // Create gradients and cap intensity
        gradients = createGradients(smoothedImage);

        // TDF
        runTubeDetectionFilter(gradients, mMinimumRadius, 1.5, smallTDF, smallRadius);
    }

    RidgeTraversalCenterlineExtraction::pointer centerlineExtraction = RidgeTraversalCenterlineExtraction::New();
    Image::pointer TDF;
    InverseGradientSegmentation::pointer segmentation = InverseGradientSegmentation::New();
    Mesh::pointer centerline;
    if(smallTDF && largeTDF) {
        // Both small and large TDF has been executed, need to merge the two.
        TDF = largeTDF;
        // First, extract centerlines from largeTDF and GVF
        // Then extract centerlines from smallTDF using large centerlines as input
        centerlineExtraction->setInputData(0, largeTDF);
        centerlineExtraction->setInputData(1, GVFfield);
        centerlineExtraction->setInputData(2, largeRadius);
        centerlineExtraction->setInputData(3, smallTDF);
        centerlineExtraction->setInputData(4, gradients);
        centerlineExtraction->setInputData(5, smallRadius);
        auto port = centerlineExtraction->getOutputPort(0);
        auto port2 = centerlineExtraction->getOutputPort(1);
        centerlineExtraction->update();
        centerline = port->getNextFrame<Mesh>();

        // Segmentation
        // TODO: Use only dilation for smallTDF
        segmentation->setInputData(0, port2->getNextFrame());
        segmentation->setInputData(1, GVFfield);

    } else {
        // Only small or large TDF has been used
        if(smallTDF) {
            TDF = smallTDF;
            centerlineExtraction->setInputData(0, smallTDF);
            centerlineExtraction->setInputData(1, gradients);
            centerlineExtraction->setInputData(2, smallRadius);
        } else {
            TDF = largeTDF;
            centerlineExtraction->setInputData(0, largeTDF);
            centerlineExtraction->setInputData(1, GVFfield);
            centerlineExtraction->setInputData(2, largeRadius);
            gradients = GVFfield;
        }
        auto port = centerlineExtraction->getOutputPort();
        centerlineExtraction->update();
        centerline = port->getNextFrame<Mesh>();

        // Segmentation
        segmentation->setInputConnection(centerlineExtraction->getOutputPort(1));
        segmentation->setInputData(1, gradients);
    }

    auto segPort = segmentation->getOutputPort();
    segmentation->update();
    auto segmentationVolume = segPort->getNextFrame<Image>();

    // TODO get largest segmentation object
    reportInfo() << "Removing small objects..." << Reporter::end();
    keepLargestObjects(segmentationVolume, centerline);

    addOutputData(0, segmentationVolume);
    addOutputData(1, centerline);
    addOutputData(2, TDF);
}


Image::pointer TubeSegmentationAndCenterlineExtraction::runGradientVectorFlow(Image::pointer vectorField) {
    OpenCLDevice::pointer device = std::dynamic_pointer_cast<OpenCLDevice>(getMainDevice());
    reportInfo() << "Running GVF.." << Reporter::end();
    MultigridGradientVectorFlow::pointer gvf = MultigridGradientVectorFlow::New();
    gvf->setInputData(vectorField);
    //gvf->set32bitStorageFormat();
    gvf->set16bitStorageFormat();
    gvf->setIterations(10);
    gvf->setMuConstant(0.199);
    DataChannel::pointer port = gvf->getOutputPort();
    gvf->update();
    reportInfo() << "GVF finished" << Reporter::end();
    return port->getNextFrame<Image>();
}

Image::pointer TubeSegmentationAndCenterlineExtraction::createGradients(Image::pointer image) {
    OpenCLDevice::pointer device = std::dynamic_pointer_cast<OpenCLDevice>(getMainDevice());
    auto floatImage = Image::create(image->getWidth(), image->getHeight(), image->getDepth(), TYPE_FLOAT, 1);
    auto vectorField = Image::create(image->getWidth(), image->getHeight(), image->getDepth(), TYPE_SNORM_INT16, 3);
    //vectorField->create(image->getWidth(), image->getHeight(), image->getDepth(), TYPE_FLOAT, 3);
    vectorField->setSpacing(image->getSpacing());
    SceneGraph::setParentNode(vectorField, image);

    bool no3Dwrite = !device->isWritingTo3DTexturesSupported();

    OpenCLImageAccess::pointer access = image->getOpenCLImageAccess(ACCESS_READ, device);
    cl::Program program =  getOpenCLProgram(device, "", "-DVECTORS_16BIT");
    reportInfo() << "build gradients program" << Reporter::end();

    // Convert to float 0-1
    cl::Kernel toFloatKernel(program, "toFloat");

    float minimumIntensity;
    if(mMinimumIntensity > -std::numeric_limits<float>::max()) {
        minimumIntensity = mMinimumIntensity;
    } else {
        minimumIntensity = image->calculateMinimumIntensity();
    }
    float maximumIntensity;
    if(mMaximumIntensity < std::numeric_limits<float>::max()) {
        maximumIntensity = mMaximumIntensity;
    } else {
        maximumIntensity = image->calculateMaximumIntensity();
    }

    toFloatKernel.setArg(0, *(access->get3DImage()));
    if(no3Dwrite) {
        OpenCLBufferAccess::pointer floatImageAccess = floatImage->getOpenCLBufferAccess(ACCESS_READ_WRITE, device);
        toFloatKernel.setArg(1, *(floatImageAccess->get()));
    } else {
        OpenCLImageAccess::pointer floatImageAccess = floatImage->getOpenCLImageAccess(ACCESS_READ_WRITE, device);
        toFloatKernel.setArg(1, *(floatImageAccess->get3DImage()));
    }
    toFloatKernel.setArg(2, minimumIntensity);
    toFloatKernel.setArg(3, maximumIntensity);
    device->getCommandQueue().enqueueNDRangeKernel(
            toFloatKernel,
            cl::NullRange,
            cl::NDRange(image->getWidth(), image->getHeight(), image->getDepth()),
            cl::NullRange
    );

    // Create vector field
    cl::Kernel vectorFieldKernel(program, "createVectorField");

    // Use sensitivity to set vector maximum (fmax)
    float vectorMaximum = (1 - mSensitivity);
    float sign = mExtractDarkStructures ? -1.0f : 1.0f;

    OpenCLImageAccess::pointer floatImageAccess = floatImage->getOpenCLImageAccess(ACCESS_READ_WRITE, device);
    vectorFieldKernel.setArg(0, *(floatImageAccess->get3DImage()));
    if(no3Dwrite) {
        OpenCLBufferAccess::pointer vectorFieldAccess = vectorField->getOpenCLBufferAccess(ACCESS_READ_WRITE, device);
        vectorFieldKernel.setArg(1, *(vectorFieldAccess->get()));
    } else {
        OpenCLImageAccess::pointer vectorFieldAccess = vectorField->getOpenCLImageAccess(ACCESS_READ_WRITE, device);
        vectorFieldKernel.setArg(1, *(vectorFieldAccess->get3DImage()));
    }
    vectorFieldKernel.setArg(2, vectorMaximum);
    vectorFieldKernel.setArg(3, sign);
    vectorFieldKernel.setArg(4, vectorField->getSpacing().x());
    vectorFieldKernel.setArg(5, vectorField->getSpacing().y());
    vectorFieldKernel.setArg(6, vectorField->getSpacing().z());
    /*
    vectorFieldKernel.setArg(4, 1.0f);
    vectorFieldKernel.setArg(5, 1.0f);
    vectorFieldKernel.setArg(6, 1.0f);
    */

    // Run kernel
    device->getCommandQueue().enqueueNDRangeKernel(
            vectorFieldKernel,
            cl::NullRange,
            cl::NDRange(image->getWidth(), image->getHeight(), image->getDepth()),
            cl::NullRange
    );

    return vectorField;
}

void TubeSegmentationAndCenterlineExtraction::runTubeDetectionFilter(Image::pointer vectorField, float minimumRadius, float maximumRadius, Image::pointer& TDF, Image::pointer& radius) {
    OpenCLDevice::pointer device = std::dynamic_pointer_cast<OpenCLDevice>(getMainDevice());
    TDF = Image::create(vectorField->getSize(), TYPE_FLOAT, 1);
    TDF->setSpacing(vectorField->getSpacing());
    SceneGraph::setParentNode(TDF, vectorField);
    radius = Image::create(vectorField->getSize(), TYPE_FLOAT, 1);

    OpenCLBufferAccess::pointer TDFAccess = TDF->getOpenCLBufferAccess(ACCESS_READ_WRITE, device);
    OpenCLBufferAccess::pointer radiusAccess = radius->getOpenCLBufferAccess(ACCESS_READ_WRITE, device);
    OpenCLImageAccess::pointer vectorFieldAccess = vectorField->getOpenCLImageAccess(ACCESS_READ, device);

    cl::Program program = getOpenCLProgram(device);
    cl::Kernel kernel(program, "circleFittingTDF");

    kernel.setArg(0, *(vectorFieldAccess->get3DImage()));
    kernel.setArg(1, *(TDFAccess->get()));
    kernel.setArg(2, minimumRadius);
    kernel.setArg(3, maximumRadius);
    kernel.setArg(4, mRadiusStep);
    kernel.setArg(5, *(radiusAccess->get()));
    /*
    kernel.setArg(6, vectorField->getSpacing().x());
    kernel.setArg(7, vectorField->getSpacing().y());
    kernel.setArg(8, vectorField->getSpacing().z());
    */
    kernel.setArg(6, 1.0f);
    kernel.setArg(7, 1.0f);
    kernel.setArg(8, 1.0f);

    device->getCommandQueue().enqueueNDRangeKernel(
            kernel,
            cl::NullRange,
            cl::NDRange(TDF->getWidth(), TDF->getHeight(), TDF->getDepth()),
            cl::NullRange
    );
}

void TubeSegmentationAndCenterlineExtraction::runNonCircularTubeDetectionFilter(Image::pointer vectorField, float minimumRadius, float maximumRadius, Image::pointer& TDF, Image::pointer& radius) {
    OpenCLDevice::pointer device = std::dynamic_pointer_cast<OpenCLDevice>(getMainDevice());
    TDF = Image::create(vectorField->getSize(), TYPE_FLOAT, 1);
    TDF->setSpacing(vectorField->getSpacing());
    SceneGraph::setParentNode(TDF, vectorField);
    radius = Image::create(vectorField->getSize(), TYPE_FLOAT, 1);

    OpenCLBufferAccess::pointer TDFAccess = TDF->getOpenCLBufferAccess(ACCESS_READ_WRITE, device);
    OpenCLBufferAccess::pointer radiusAccess = radius->getOpenCLBufferAccess(ACCESS_READ_WRITE, device);
    OpenCLImageAccess::pointer vectorFieldAccess = vectorField->getOpenCLImageAccess(ACCESS_READ, device);

    reportInfo() << "building TDF program" << Reporter::end();
    cl::Program program = getOpenCLProgram(device);
    reportInfo() << "build TDF program" << Reporter::end();
    cl::Kernel kernel(program, "nonCircularTDF");

    kernel.setArg(0, *(vectorFieldAccess->get3DImage()));
    kernel.setArg(1, *(TDFAccess->get()));
    kernel.setArg(2, minimumRadius);
    kernel.setArg(3, maximumRadius);
    kernel.setArg(4, mRadiusStep);
    kernel.setArg(5, 12); // nr of line searches
    kernel.setArg(6, 0.2f); // GVF magnitude threshold
    kernel.setArg(7, *(radiusAccess->get()));
    /*
    kernel.setArg(8, vectorField->getSpacing().x());
    kernel.setArg(9, vectorField->getSpacing().y());
    kernel.setArg(10, vectorField->getSpacing().z());
    */
    kernel.setArg(8, 1.0f);
    kernel.setArg(9, 1.0f);
    kernel.setArg(10, 1.0f);

    device->getCommandQueue().enqueueNDRangeKernel(
            kernel,
            cl::NullRange,
            cl::NDRange(TDF->getWidth(), TDF->getHeight(), TDF->getDepth()),
            cl::NullRange
    );
}

void TubeSegmentationAndCenterlineExtraction::setKeepLargestTree(bool keep) {
    mOnlyKeepLargestTree = keep;
    if(keep)
        mMinimumTreeSize = -1;
}

void TubeSegmentationAndCenterlineExtraction::setMinimumTreeSize(int nrOfVoxels) {
    mMinimumTreeSize = nrOfVoxels;
    mOnlyKeepLargestTree = false;
}

} // end namespace fast
