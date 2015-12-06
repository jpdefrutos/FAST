#include "FAST/Testing.hpp"
#include "AirwaySegmentation.hpp"
#include "FAST/Importers/ImageFileImporter.hpp"

#include "FAST/Algorithms/SurfaceExtraction/SurfaceExtraction.hpp"
#include "FAST/Visualization/MeshRenderer/MeshRenderer.hpp"
#include "FAST/Visualization/SimpleWindow.hpp"

using namespace fast;

TEST_CASE("Airway segmentation", "[fast][AirwaySegmentation]") {
	Reporter::setGlobalReportMethod(Reporter::COUT);
    ImageFileImporter::pointer importer = ImageFileImporter::New();
    //importer->setFilename(std::string(FAST_TEST_DATA_DIR) + "CT-Thorax.mhd");
    importer->setFilename("/home/smistad/Data/lunge_datasett/pasient06.mhd");

    AirwaySegmentation::pointer segmentation = AirwaySegmentation::New();
    segmentation->setInputConnection(importer->getOutputPort());

    SurfaceExtraction::pointer extraction = SurfaceExtraction::New();
    extraction->setInputConnection(segmentation->getOutputPort());

    MeshRenderer::pointer renderer = MeshRenderer::New();
    renderer->addInputConnection(extraction->getOutputPort());

    SimpleWindow::pointer window = SimpleWindow::New();
    window->addRenderer(renderer);
    window->start();

}
