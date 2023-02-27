#include <FAST/Visualization/SliceRenderer/SliceRenderer.hpp>
#include <FAST/Visualization/SimpleWindow.hpp>
#include "FAST/Testing.hpp"
#include "ImageResizer.hpp"
#include "FAST/Importers/ImageFileImporter.hpp"
#include "FAST/Data/Image.hpp"

using namespace fast;

TEST_CASE("ImageResizer 2D", "[fast][ImageResizer]") {
	ImageFileImporter::pointer importer = ImageFileImporter::New();
	importer->setFilename(Config::getTestDataPath() + "US/Heart/ApicalFourChamber/US-2D_9.mhd");
	ImageResizer::pointer resizer = ImageResizer::create(64, 64)->connect(importer);
	auto port = resizer->getOutputPort();
	resizer->update();

	Image::pointer result = port->getNextFrame<Image>();
	CHECK(result->getWidth() == 64);
	CHECK(result->getHeight() == 64);
}

TEST_CASE("ImageResizer 2D preserve aspect", "[fast][ImageResizer]") {
	ImageFileImporter::pointer importer = ImageFileImporter::New();
	importer->setFilename(Config::getTestDataPath() + "US/Heart/ApicalFourChamber/US-2D_9.mhd");
	ImageResizer::pointer resizer = ImageResizer::create(256,256)->connect(importer);
	resizer->setPreserveAspectRatio(true);
	auto port = resizer->getOutputPort();
	resizer->update();

	Image::pointer result = port->getNextFrame<Image>();
	CHECK(result->getWidth() == 256);
	CHECK(result->getHeight() == 256);

	/*
	ImageRenderer::pointer renderer = ImageRenderer::New();
	renderer->addInputConnection(resizer->getOutputPort());
	SimpleWindow::pointer window = SimpleWindow::New();
	window->addRenderer(renderer);
	window->start();
	 */
}

TEST_CASE("ImageResizer 3D", "[fast][ImageResizer]") {
	ImageFileImporter::pointer importer = ImageFileImporter::New();
	importer->setFilename(Config::getTestDataPath() + "US/Ball/US-3Dt_0.mhd");
	ImageResizer::pointer resizer = ImageResizer::create(64, 64, 64)->connect(importer);
	auto port = resizer->getOutputPort();
	resizer->update();

	Image::pointer result = port->getNextFrame<Image>();
	CHECK(result->getWidth() == 64);
	CHECK(result->getHeight() == 64);
	CHECK(result->getDepth() == 64);

	/*
	auto renderer = SliceRenderer::New();
	renderer->addInputConnection(resizer->getOutputPort());
	auto window = SimpleWindow::New();
	window->addRenderer(renderer);
	window->start();
	 */
}
