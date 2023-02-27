#include <FAST/Streamers/RealSenseStreamer.hpp>
#include "Tracking.hpp"
#include "FAST/Data/Image.hpp"
#include "FAST/Data/Mesh.hpp"
#include "FAST/Algorithms/IterativeClosestPoint/IterativeClosestPoint.hpp"
#include <FAST/Exporters/VTKMeshFileExporter.hpp>
#include <QDir>

namespace fast {

Tracking::Tracking() {

    createInputPort<Image>(0);
    createInputPort<Mesh>(1);

    createOutputPort<Image>(0);
    createOutputPort<Image>(1); // Annotation image
    createOutputPort<Mesh>(2); // Target cloud

    // Create annotation image
    mAnnotationImage = Image::create(640, 480, TYPE_UINT8, 1);
    mAnnotationImage->fill(0);

    mTargetCloud = Mesh::create(0, 0, 0, false, false, false);
    mTargetCloudExtracted = false;
    getReporter().setReportMethod(Reporter::COUT);
}

void Tracking::restart() {
    stopRecording();
    mTargetCloudExtracted = false;
    mAnnotationImage->fill(0);
}

void Tracking::startRecording(std::string path) {
    mStoragePath = path;
    mFrameCounter = 0;
    mRecording = true;
}

void Tracking::stopRecording() {
    mRecording = false;
}

void Tracking::execute() {
    std::cout << "Kinect tracking execute.." << std::endl;

    // When target cloud has been extracted, run ICP, and output this mesh
    Mesh::pointer meshInput = getInputData<Mesh>(1);
    mCurrentCloud = meshInput;
    if(mTargetCloudExtracted) {
        reportInfo() << "Running ICP" << reportEnd();
        IterativeClosestPoint::pointer icp = IterativeClosestPoint::New();
        icp->enableRuntimeMeasurements();
        icp->setFixedMesh(meshInput);
        icp->setMovingMesh(mTargetCloud);
        icp->setDistanceThreshold(150); // All points further away than 10 cm from the centroid is removed
        //icp->setMinimumErrorChange(0.5);
        icp->setRandomPointSampling(300);
        icp->getReporter().setReportMethod(Reporter::COUT);
        icp->setMaximumNrOfIterations(10);
        icp->update();
        reportInfo() << "Finished ICP in: " << reportEnd();
        icp->getAllRuntimes()->printAll();
        auto currentTransform = mTargetCloud->getSceneGraphNode()->getTransform();
        auto newTransform = icp->getOutputTransformation();
        mTargetCloud->getSceneGraphNode()->setTransform(newTransform->get() * currentTransform->get());

        if(mRecording) {
            VTKMeshFileExporter::pointer exporter = VTKMeshFileExporter::New();
            exporter->setInputData(meshInput);
            exporter->setWriteNormals(false);
            exporter->setWriteColors(true);
            exporter->setFilename(mStoragePath + std::to_string(mFrameCounter) + ".vtk");
            exporter->update();
            ++mFrameCounter;
        }
    } else {
        Image::pointer input = getInputData<Image>();
        addOutputData(0, input);
    }

    addOutputData(1, mAnnotationImage);
    addOutputData(2, mTargetCloud);
}

void Tracking::calculateTargetCloud(std::shared_ptr<RealSenseStreamer> streamer) {
    std::cout << "Creating target cloud..." << std::endl;
    ImageAccess::pointer access = mAnnotationImage->getImageAccess(ACCESS_READ);
    MeshAccess::pointer meshAccess = mCurrentCloud->getMeshAccess(ACCESS_READ);
    std::vector<MeshVertex> vertices = meshAccess->getVertices();
    std::vector<MeshVertex> outputVertices;
    for(int y = 0; y < mAnnotationImage->getHeight(); ++y) {
        for(int x = 0; x < mAnnotationImage->getWidth(); ++x) {
            try {
                if(access->getScalar(Vector2i(x, y)) == 1) {
                    MeshVertex vertex = streamer->getPoint(x, y);
                    if(!std::isnan(vertex.getPosition().x())) {
                        outputVertices.push_back(vertex);
                    }
                }
            } catch (Exception &e) {

            }
        }
    }
    if(outputVertices.size() == 0)
        throw Exception("No vertices found when creating target cloud");

    mTargetCloud = Mesh::create(outputVertices);
    std::cout << "Created target cloud." << std::endl;
    mTargetCloudExtracted = true;
}

void Tracking::addLine(Vector2i start, Vector2i end) {
    if(mTargetCloudExtracted)
        return;
    std::cout << "Drawing from: " << start.transpose() << " to " << end.transpose() << std::endl;
    // Draw line in some auxillary image
    mAnnotationImage = mAnnotationImage->copy(Host::getInstance());
    ImageAccess::pointer access = mAnnotationImage->getImageAccess(ACCESS_READ_WRITE);
    Vector2f direction = end.cast<float>() - start.cast<float>();
    int length = (end-start).norm();
    int brushSize = 6;
    for(int i = 0; i < length; ++i) {
        float distance = (float)i/length;
        for(int a = -brushSize; a <= brushSize; a++) {
            for(int b = -brushSize; b <= brushSize; b++) {
                Vector2f offset(a, b);
                if(offset.norm() > brushSize)
                    continue;
                Vector2f position = start.cast<float>() + direction*distance + offset;
                try {
                    access->setScalar(position.cast<int>(), 1);
                } catch(Exception &e) {

                }
            }
        }
    }
}

uint Tracking::getFramesStored() const {
    return mFrameCounter;
}

bool Tracking::isRecording() const {
    return mRecording;
}

std::shared_ptr<Mesh> Tracking::getTargetCloud() const {
    return mTargetCloud;
}

void Tracking::setTargetCloud(std::shared_ptr<Mesh> target) {
    mTargetCloud = target;
    mTargetCloudExtracted = true;
}

}