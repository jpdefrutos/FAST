#include "FAST/Testing.hpp"
#include "FAST/Exporters/MetaImageExporter.hpp"
#include "FAST/Importers/MetaImageImporter.hpp"
#include "FAST/Data/Image.hpp"
#include "FAST/Tests/DataComparison.hpp"

using namespace fast;

TEST_CASE("No filename given to the MetaImageExporter", "[fast][MetaImageExporter]") {
    Image::pointer image = Image::create(32, 32, TYPE_UINT8, 1);
    MetaImageExporter::pointer exporter = MetaImageExporter::New();
    exporter->setInputData(image);
    CHECK_THROWS(exporter->update());
}

TEST_CASE("No input given to the MetaImageExporter", "[fast][MetaImageExporter]") {
    MetaImageExporter::pointer exporter = MetaImageExporter::New();
    exporter->setFilename("asd");
    CHECK_THROWS(exporter->update());
}

TEST_CASE("Write a 2D image with the MetaImageExporter", "[fast][MetaImageExporter]") {
    // Create some metadata
    Vector3f spacing;
    spacing[0] = 1.2;
    spacing[1] = 2.3;
    spacing[2] = 1;
    Vector3f offset;
    offset[0] = 2.2;
    offset[1] = 3.3;
    offset[2] = 3.1;
    Matrix3f transformMatrix;
    transformMatrix(0,0) = 0.2;
    transformMatrix(1,0) = 1.3;
    transformMatrix(2,0) = 2.0;
    transformMatrix(0,1) = 3.0;
    transformMatrix(1,1) = 4.0;
    transformMatrix(2,1) = 5.0;
    transformMatrix(0,2) = 6.0;
    transformMatrix(1,2) = 7.0;
    transformMatrix(2,2) = 8.0;

    auto T = Affine3f::Identity();
    T.translation() = offset;
    T.linear() = transformMatrix;

    unsigned int width = 32;
    unsigned int height = 46;
    for(unsigned int channels = 1; channels <= 4; channels++) {
        for(unsigned int typeNr = 0; typeNr < 5; typeNr++) { // for all types
            DataType type = (DataType)typeNr;

            void* data = allocateRandomData(width*height*channels, type);
            Image::pointer image = Image::create(width, height, type, channels, Host::getInstance(), data);

            // Set metadata
            image->setSpacing(spacing);
            image->getSceneGraphNode()->setTransform(T);

            // Export image
            MetaImageExporter::pointer exporter = MetaImageExporter::New();
            exporter->setFilename("MetaImageExporterTest2D.mhd");
            exporter->setInputData(image);
            exporter->update();

            // Import image back again
            MetaImageImporter::pointer importer = MetaImageImporter::New();
            importer->setFilename("MetaImageExporterTest2D.mhd");
            DataChannel::pointer port = importer->getOutputPort();
            importer->update();
            Image::pointer image2 = port->getNextFrame<Image>();
            auto T2 = image2->getSceneGraphNode()->getTransform()->get();

            // Check that the image properties are correct
            for(unsigned int i = 0; i < 3; i++) {
                CHECK(spacing[i] == Approx(image2->getSpacing()[i]));
                CHECK(offset[i] == Approx(T2.translation()[i]));
            }
            for(unsigned int i = 0; i < 3; i++) {
            for(unsigned int j = 0; j < 3; j++) {
                CHECK(transformMatrix(i,j) == Approx(T2.linear()(i,j)));
            }}


            CHECK(image2->getWidth() == width);
            CHECK(image2->getHeight() == height);
            CHECK(image2->getDepth() == 1);
            CHECK(image2->getDataType() == type);
            CHECK(image2->getNrOfChannels() == channels);
            CHECK(image2->getDimensions() == 2);

            ImageAccess::pointer access = image2->getImageAccess(ACCESS_READ);
            void* data2 = access->get();
            CHECK(compareDataArrays(data, data2, width*height*channels, type) == true);
            deleteArray(data, type);
        }
    }
}


TEST_CASE("Write a 3D image with the MetaImageExporter", "[fast][MetaImageExporter]") {
    // Create some metadata
    Vector3f spacing;
    spacing[0] = 1.2;
    spacing[1] = 2.3;
    spacing[2] = 1;
    Vector3f offset;
    offset[0] = 2.2;
    offset[1] = 3.3;
    offset[2] = 3.1;
    Matrix3f transformMatrix;
    transformMatrix(0,0) = 0.2;
    transformMatrix(1,0) = 1.3;
    transformMatrix(2,0) = 2.0;
    transformMatrix(0,1) = 3.0;
    transformMatrix(1,1) = 4.0;
    transformMatrix(2,1) = 5.0;
    transformMatrix(0,2) = 6.0;
    transformMatrix(1,2) = 7.0;
    transformMatrix(2,2) = 8.0;

    auto T = Affine3f::Identity();
    T.translation() = offset;
    T.linear() = transformMatrix;

    unsigned int width = 32;
    unsigned int height = 22;
    unsigned int depth = 20;
    for(unsigned int channels = 1; channels <= 4; channels++) {
        for(unsigned int typeNr = 0; typeNr < 5; typeNr++) { // for all types
            DataType type = (DataType)typeNr;

            void* data = allocateRandomData(width*height*depth*channels, type);
            Image::pointer image = Image::create(width, height, depth, type, channels, Host::getInstance(), data);

            // Set metadata
            image->setSpacing(spacing);
            image->getSceneGraphNode()->setTransform(T);

            // Export image
            MetaImageExporter::pointer exporter = MetaImageExporter::New();
            exporter->setFilename("MetaImageExporterTest3D.mhd");
            exporter->setInputData(image);
            exporter->update();

            // Import image back again
            MetaImageImporter::pointer importer = MetaImageImporter::New();
            importer->setFilename("MetaImageExporterTest3D.mhd");
            DataChannel::pointer port = importer->getOutputPort();
            importer->update();
            Image::pointer image2 = port->getNextFrame<Image>();
            auto T2 = image2->getSceneGraphNode()->getTransform()->get();

            // Check that the image properties are correct
            for(unsigned int i = 0; i < 3; i++) {
                CHECK(spacing[i] == Approx(image2->getSpacing()[i]));
                CHECK(offset[i] == Approx(T2.translation()[i]));
            }
            for(unsigned int i = 0; i < 3; i++) {
            for(unsigned int j = 0; j < 3; j++) {
                CHECK(transformMatrix(i,j) == Approx(T2.linear()(i,j)));
            }}

            CHECK(image2->getWidth() == width);
            CHECK(image2->getHeight() == height);
            CHECK(image2->getDepth() == depth);
            CHECK(image2->getDataType() == type);
            CHECK(image2->getNrOfChannels() == channels);
            CHECK(image2->getDimensions() == 3);

            ImageAccess::pointer access = image2->getImageAccess(ACCESS_READ);
            void* data2 = access->get();
            CHECK(compareDataArrays(data, data2, width*height*depth*channels, type) == true);
            deleteArray(data, type);
        }
    }
}

TEST_CASE("Write a compressed 2D image with the MetaImageExporter", "[fast][MetaImageExporter]") {
    // Create some metadata
    Vector3f spacing;
    spacing[0] = 1.2;
    spacing[1] = 2.3;
    spacing[2] = 1;
    Vector3f offset;
    offset[0] = 2.2;
    offset[1] = 3.3;
    offset[2] = 3.1;
    Matrix3f transformMatrix;
    transformMatrix(0,0) = 0.2;
    transformMatrix(1,0) = 1.3;
    transformMatrix(2,0) = 2.0;
    transformMatrix(0,1) = 3.0;
    transformMatrix(1,1) = 4.0;
    transformMatrix(2,1) = 5.0;
    transformMatrix(0,2) = 6.0;
    transformMatrix(1,2) = 7.0;
    transformMatrix(2,2) = 8.0;

    auto T = Affine3f::Identity();
    T.translation() = offset;
    T.linear() = transformMatrix;

    unsigned int width = 32;
    unsigned int height = 46;
    for(unsigned int channels = 1; channels <= 4; channels++) {
        INFO("Nr of channels: " << channels);
        for(unsigned int typeNr = 0; typeNr < 5; typeNr++) { // for all types
            INFO("Type nr: " << typeNr);
            DataType type = (DataType)typeNr;

            void* data = allocateRandomData(width*height*channels, type);
            Image::pointer image = Image::create(width, height, type, channels, Host::getInstance(), data);

            // Set metadata
            image->setSpacing(spacing);
            image->getSceneGraphNode()->setTransform(T);

            // Export image
            MetaImageExporter::pointer exporter = MetaImageExporter::New();
            exporter->setFilename("MetaImageExporterTest2D.mhd");
            exporter->setInputData(image);
            exporter->enableCompression();
            exporter->update();

            // Import image back again
            MetaImageImporter::pointer importer = MetaImageImporter::New();
            importer->setFilename("MetaImageExporterTest2D.mhd");
            auto port = importer->getOutputPort();
            importer->update();
            Image::pointer image2 = port->getNextFrame<Image>();
            auto T2 = image2->getSceneGraphNode()->getTransform()->get();

            // Check that the image properties are correct
            for(unsigned int i = 0; i < 3; i++) {
                CHECK(spacing[i] == Approx(image2->getSpacing()[i]));
                CHECK(offset[i] == Approx(T2.translation()[i]));
            }
            for(unsigned int i = 0; i < 3; i++) {
            for(unsigned int j = 0; j < 3; j++) {
                CHECK(transformMatrix(i,j) == Approx(T2.linear()(i,j)));
            }}

            CHECK(image2->getWidth() == width);
            CHECK(image2->getHeight() == height);
            CHECK(image2->getDepth() == 1);
            CHECK(image2->getDataType() == type);
            CHECK(image2->getNrOfChannels() == channels);
            CHECK(image2->getDimensions() == 2);

            ImageAccess::pointer access = image2->getImageAccess(ACCESS_READ);
            void* data2 = access->get();
            CHECK(compareDataArrays(data, data2, width*height*channels, type) == true);
            deleteArray(data, type);
        }
    }
}

TEST_CASE("Write a compressed 3D image with the MetaImageExporter", "[fast][MetaImageExporter]") {
    // Create some metadata
    Vector3f spacing;
    spacing[0] = 1.2;
    spacing[1] = 2.3;
    spacing[2] = 1;
    Vector3f offset;
    offset[0] = 2.2;
    offset[1] = 3.3;
    offset[2] = 3.1;
    Matrix3f transformMatrix;
    transformMatrix(0,0) = 0.2;
    transformMatrix(1,0) = 1.3;
    transformMatrix(2,0) = 2.0;
    transformMatrix(0,1) = 3.0;
    transformMatrix(1,1) = 4.0;
    transformMatrix(2,1) = 5.0;
    transformMatrix(0,2) = 6.0;
    transformMatrix(1,2) = 7.0;
    transformMatrix(2,2) = 8.0;


    auto T = Affine3f::Identity();
    T.translation() = offset;
    T.linear() = transformMatrix;

    unsigned int width = 32;
    unsigned int height = 22;
    unsigned int depth = 20;
    for(unsigned int channels = 1; channels <= 4; channels++) {
        for(unsigned int typeNr = 0; typeNr < 5; typeNr++) { // for all types
            DataType type = (DataType)typeNr;

            void* data = allocateRandomData(width*height*depth*channels, type);
            Image::pointer image = Image::create(width, height, depth, type, channels, Host::getInstance(), data);

            // Set metadata
            image->setSpacing(spacing);
            image->getSceneGraphNode()->setTransform(T);

            // Export image
            MetaImageExporter::pointer exporter = MetaImageExporter::New();
            exporter->setFilename("MetaImageExporterTest3D.mhd");
            exporter->setInputData(image);
            exporter->enableCompression();
            exporter->update();

            // Import image back again
            MetaImageImporter::pointer importer = MetaImageImporter::New();
            importer->setFilename("MetaImageExporterTest3D.mhd");
            auto port = importer->getOutputPort();
            importer->update();
            Image::pointer image2 = port->getNextFrame<Image>();
            auto T2 = image2->getSceneGraphNode()->getTransform()->get();

            // Check that the image properties are correct
            for(unsigned int i = 0; i < 3; i++) {
                CHECK(spacing[i] == Approx(image2->getSpacing()[i]));
                CHECK(offset[i] == Approx(T2.translation()[i]));
            }
            for(unsigned int i = 0; i < 3; i++) {
            for(unsigned int j = 0; j < 3; j++) {
                CHECK(transformMatrix(i,j) == Approx(T2.linear()(i,j)));
            }}

            CHECK(image2->getWidth() == width);
            CHECK(image2->getHeight() == height);
            CHECK(image2->getDepth() == depth);
            CHECK(image2->getDataType() == type);
            CHECK(image2->getNrOfChannels() == channels);
            CHECK(image2->getDimensions() == 3);

            ImageAccess::pointer access = image2->getImageAccess(ACCESS_READ);
            void* data2 = access->get();
            CHECK(compareDataArrays(data, data2, width*height*depth*channels, type) == true);
            deleteArray(data, type);
        }
    }
}
