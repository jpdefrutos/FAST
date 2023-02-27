#include <FAST/Visualization/SimpleWindow.hpp>
#include <FAST/Importers/VTKMeshFileImporter.hpp>
#include <FAST/Importers/ImageFileImporter.hpp>
#include "FAST/Exporters/VTKMeshFileExporter.hpp"
#include <FAST/Visualization/ImageRenderer/ImageRenderer.hpp>
#include "FAST/Visualization/VertexRenderer/VertexRenderer.hpp"
#include "FAST/Visualization/TriangleRenderer/TriangleRenderer.hpp"
#include "FAST/Algorithms/SurfaceExtraction/SurfaceExtraction.hpp"
#include "FAST/Testing.hpp"
#include "CoherentPointDrift.hpp"
#include "Rigid.hpp"
#include "Affine.hpp"

#include <random>
#include <iostream>
using namespace fast;

Mesh::pointer getPointCloud(std::string filename=std::string("Surface_LV.vtk")) {
    auto importer = VTKMeshFileImporter::New();
//    importer->setFilename(Config::getTestDataPath() + "Surface_LV.vtk");
    importer->setFilename(Config::getTestDataPath() + filename);
    auto port = importer->getOutputPort();
    importer->update();
    return port->getNextFrame<Mesh>();
}


void modifyPointCloud(Mesh::pointer &pointCloud, double fractionOfPointsToKeep, double noiseSampleRatio=0.0) {
    MeshAccess::pointer accessFixedSet = pointCloud->getMeshAccess(ACCESS_READ);
    std::vector<MeshVertex> vertices = accessFixedSet->getVertices();

    // Sample the preferred amount of points from the point cloud
    auto numVertices = (unsigned int) vertices.size();
    auto numSamplePoints = (unsigned int) ceil(fractionOfPointsToKeep * numVertices);
    std::vector<MeshVertex> newVertices;

    std::unordered_set<int> movingIndices;
    unsigned int sampledPoints = 0;
    std::default_random_engine distributionEngine;
    std::uniform_int_distribution<unsigned int> distribution(0, numVertices-1);
    while (sampledPoints < numSamplePoints) {
        unsigned int index = distribution(distributionEngine);
        if (movingIndices.count(index) < 1 && vertices.at(index).getPosition().array().isNaN().sum() == 0 ) {
            newVertices.push_back(vertices.at(index));
            movingIndices.insert(index);
            ++sampledPoints;
        }
    }

    // Add noise to point cloud
    auto numNoisePoints = (unsigned int) ceil(noiseSampleRatio * numSamplePoints);
    float minX, minY, minZ;
    Vector3f position0 = vertices[0].getPosition();
    minX = position0[0];
    minY = position0[1];
    minZ = position0[2];
    float maxX = minX, maxY = minY, maxZ = minZ;
    for (auto &vertex : vertices) {
        Vector3f position = vertex.getPosition();
        if (position[0] < minX) {minX = position[0]; }
        if (position[0] > maxX) {maxX = position[0]; }
        if (position[1] < minY) {minY = position[1]; }
        if (position[1] > maxY) {maxY = position[1]; }
        if (position[2] < minZ) {minZ = position[2]; }
        if (position[2] > maxZ) {maxZ = position[2]; }
    }

    std::uniform_real_distribution<float> distributionNoiseX(minX, maxX);
    std::uniform_real_distribution<float> distributionNoiseY(minY, maxY);
    std::uniform_real_distribution<float> distributionNoiseZ(minZ, maxZ);

    for (int noiseAdded = 0; noiseAdded < numNoisePoints; noiseAdded++) {
        float noiseX = distributionNoiseX (distributionEngine);
        float noiseY = distributionNoiseY (distributionEngine);
        float noiseZ = distributionNoiseZ (distributionEngine);
        Vector3f noisePosition = Vector3f(noiseX, noiseY, noiseZ);
        MeshVertex noise = MeshVertex(noisePosition, Vector3f(1, 0, 0), Color::Black());
        newVertices.push_back(noise);
    }

    // Update point cloud to include the removed points and added noise
    pointCloud->create(newVertices);
}

void saveAbdominalSurfaceExtraction(int threshold=-500) {
    // Import CT image
    ImageFileImporter::pointer importer = ImageFileImporter::New();
    importer->setFilename(Config::getTestDataPath() + "CT/CT-Abdomen.mhd");

    // Extract surface mesh using a threshold value
    SurfaceExtraction::pointer extraction = SurfaceExtraction::create();
    extraction->setInputConnection(importer->getOutputPort());
    extraction->setThreshold(threshold);

    auto exporter = VTKMeshFileExporter::New();
    exporter->setFilename(Config::getTestDataPath() + "AbdominalModel.vtk");
    exporter->setInputConnection(extraction->getOutputPort());
    exporter->update();
}

void visualizeSurfaceExtraction(int threshold=-500) {

    // Import CT image
    ImageFileImporter::pointer importer = ImageFileImporter::New();
    importer->setFilename(Config::getTestDataPath() + "CT/CT-Abdomen.mhd");

    // Extract surface mesh using a threshold value
    SurfaceExtraction::pointer extraction = SurfaceExtraction::create();
    extraction->setInputConnection(importer->getOutputPort());
    extraction->setThreshold(threshold);

    TriangleRenderer::pointer surfaceRenderer = TriangleRenderer::New();
    surfaceRenderer->setInputConnection(extraction->getOutputPort());
    SimpleWindow::pointer window = SimpleWindow::New();
    window->addRenderer(surfaceRenderer);
    window->start();
}


TEST_CASE("cpd", "[fast][coherentpointdrift][visual][cpd]") {

    auto dataset1 = "Surface_LV.vtk";
    auto dataset2 = "Surface_LV.vtk";

    // Load point clouds
    auto cloud1 = getPointCloud(dataset1);
    auto cloud2 = getPointCloud(dataset2);
    auto cloud3 = getPointCloud(dataset2);

    // Modify point clouds
    modifyPointCloud(cloud1, 0.80, 0.25);
    modifyPointCloud(cloud2, 0.75, 0.15);
    modifyPointCloud(cloud3, 0.75, 0.15);

    // Set registration settings
    float uniformWeight = 0.5;
    double tolerance = 1e-2;
    bool applyTransform = true;

    // Transform one of the point clouds
    Vector3f translation(-0.052f, 0.005f, -0.001f);
    auto transform = Transform::create();
    MatrixXf shearing = Matrix3f::Identity();
    shearing(0, 0) = 0.5;
    shearing(0, 1) = 1.2;
    shearing(1, 0) = 0.0;
    Affine3f affine = Affine3f::Identity();
    affine.rotate(Eigen::AngleAxisf(3.141592f / 180.0f * 50.0f, Eigen::Vector3f::UnitY()));
    affine.scale(0.5);
    affine.translate(translation);
    affine.linear() += shearing;
    transform->set(affine);

    if (applyTransform) {
        // Apply transform to one point cloud
        cloud2->getSceneGraphNode()->setTransform(transform);
        // Apply transform to a point cloud not registered (for reference)
        cloud3->getSceneGraphNode()->setTransform(transform);
    }

    // Run for different numbers of iterations
    std::vector<unsigned char> iterations = {50};
    for(auto maxIterations : iterations) {

        // Run Coherent Point Drift
        auto cpd = CoherentPointDriftAffine::New();
        cpd->setFixedMesh(cloud1);
        cpd->setMovingMesh(cloud2);
        cpd->setMaximumIterations(maxIterations);
        cpd->setTolerance(tolerance);
        cpd->setUniformWeight(uniformWeight);

        auto renderer = VertexRenderer::New();
        renderer->addInputData(cloud1, Color::Green(), 3.0);                        // Fixed points
        renderer->addInputData(cloud3, Color::Blue(), 2.0);                         // Moving points
        renderer->addInputConnection(cpd->getOutputPort(), Color::Red(), 2.0);      // Moving points registered

        auto window = SimpleWindow::New();
        window->addRenderer(renderer);
        window->setTimeout(1000);
        window->start();
    }

}