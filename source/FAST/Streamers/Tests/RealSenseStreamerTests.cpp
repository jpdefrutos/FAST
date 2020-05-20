#include <QShortcut>

#include "FAST/Testing.hpp"
#include "FAST/Streamers/RealSenseStreamer.hpp"
#include "FAST/Visualization/ImageRenderer/ImageRenderer.hpp"
#include "FAST/Visualization/SimpleWindow.hpp"
#include "FAST/Visualization/VertexRenderer/VertexRenderer.hpp"
#include "FAST/Visualization/MultiViewWindow.hpp"

using namespace fast;

TEST_CASE("Stream data using RealSense", "[RealSenseStreamer][fast][RealSense][visual]") {
    Config::setStreamingMode(STREAMING_MODE_NEWEST_FRAME_ONLY);
    auto streamer = RealSenseStreamer::New();

    // Renderer RGB image
    auto renderer = ImageRenderer::New();
    renderer->addInputConnection(streamer->getOutputPort(0));

    // Renderer depth image
    auto renderer2 = ImageRenderer::New();
    renderer2->addInputConnection(streamer->getOutputPort(1));
    renderer2->setIntensityLevel(1000);
    renderer2->setIntensityWindow(500);

    // Render point cloud
    auto renderer3 = VertexRenderer::New();
    renderer3->addInputConnection(streamer->getOutputPort(2));
    renderer3->setDefaultSize(1.5);

    // Setup window
    auto window = MultiViewWindow::New();
    window->setTitle("FAST Real Sense Streaming");
    window->setHeight(512);
    window->setWidth(1920);
    window->setNrOfViews(3);
    window->addRenderer(0, renderer);
    window->getView(0)->set2DMode();
    window->getView(0)->setBackgroundColor(Color::Black());
    window->addRenderer(1, renderer2);
    window->getView(1)->set2DMode();
    window->getView(1)->setBackgroundColor(Color::Black());
    window->addRenderer(2, renderer3);
    // Adjust camera
    window->getView(2)->setLookAt(Vector3f(0,-500,-500), Vector3f(0,0,1000), Vector3f(0,-1,0), 1, 5000);
    window->getView(2)->setBackgroundColor(Color::Black());
    window->setTimeout(3000);
    CHECK_NOTHROW(window->start());
}
