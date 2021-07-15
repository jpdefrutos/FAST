/**
 * @example importImageFromFile.cpp
 * An example of importing and visualizing an image from file using the ImageFileImporter
 */
#include <FAST/Tools/CommandLineParser.hpp>
#include "FAST/Importers/ImageFileImporter.hpp"
#include "FAST/Visualization/ImageRenderer/ImageRenderer.hpp"
#include "FAST/Visualization/SimpleWindow.hpp"

using namespace fast;

int main(int argc, char** argv) {
    CommandLineParser parser("Import image from file example");
    parser.addPositionVariable(1, "filename", Config::getTestDataPath() + "US/US-2D.jpg");
    parser.parse(argc, argv);

    // Import image from file using the ImageFileImporter
    auto importer = ImageFileImporter::New();
    importer->setFilename(parser.get(1));

    // Render
    auto renderer = ImageRenderer::New();
    renderer->addInputConnection(importer->getOutputPort());

    // Setup window
    auto window = SimpleWindow::New();
    window->addRenderer(renderer);
    window->set2DMode();
#ifdef FAST_CONTINUOUS_INTEGRATION
	// This will automatically close the window after 5 seconds, used for CI testing
    window->setTimeout(5*1000);
#endif
    // Run entire pipeline and display window
    window->start();
}
