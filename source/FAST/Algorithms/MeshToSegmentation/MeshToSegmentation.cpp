#include "MeshToSegmentation.hpp"
#include "FAST/Data/Mesh.hpp"
#include "FAST/Data/Image.hpp"
#include "FAST/Utility.hpp"

namespace fast {

MeshToSegmentation::MeshToSegmentation(Vector3i size, Vector3f spacing) {
	createInputPort<Mesh>(0);
	createInputPort<Image>(1, false);
	createOutputPort<Image>(0);
	createOpenCLProgram(Config::getKernelSourcePath() + "Algorithms/MeshToSegmentation/MeshToSegmentation.cl");

	mResolution = size;
	m_spacing = spacing;
}

void MeshToSegmentation::setOutputImageResolution(uint x, uint y, uint z) {
	mResolution = Vector3i(x, y, z);
}

void MeshToSegmentation::execute() {
	auto mesh = getInputData<Mesh>(0);
	Image::pointer image;
    bool is2D = true;
    try{
        image = getInputData<Image>(1);
        is2D = image->getDimensions() == 2;
    } catch(std::exception &e) {
        if(mResolution == Vector3i::Zero())
            throw Exception("MeshToSegmentation need either a file (input 1) or the output resolution specified before running");
        is2D = mResolution.z() == 1;
    }

    Image::pointer segmentation;
	// Initialize output segmentation image and size
    if(mResolution == Vector3i::Zero()) {
        // If resolution is not specified, use input image resolution
		segmentation = Image::createSegmentationFromImage(image);
	} else {
		// Use specified resolution
        if(is2D) {
			segmentation = Image::create(mResolution.x(), mResolution.y(), TYPE_UINT8, 1);
		} else {
			segmentation = Image::create(mResolution.x(), mResolution.y(), mResolution.z(), TYPE_UINT8, 1);
		}

		// Set correct spacing
		float widthInMM, heightInMM, depthInMM;
		if(image) {
            widthInMM = image->getSpacing().x() * image->getWidth();
            heightInMM = image->getSpacing().y() * image->getHeight();
            depthInMM = image->getSpacing().z() * image->getDepth();
            SceneGraph::setParentNode(segmentation, image);
            segmentation->setSpacing(widthInMM/mResolution.x(), heightInMM/mResolution.y(), depthInMM/mResolution.z());
        } else {
            segmentation->setSpacing(m_spacing);
		}
	}

	ExecutionDevice::pointer mainDevice = std::dynamic_pointer_cast<OpenCLDevice>(getMainDevice());

	if(mainDevice->isHost()) {
		throw Exception("Not implemented");
	}

	OpenCLDevice::pointer device = std::dynamic_pointer_cast<OpenCLDevice>(mainDevice);

	// TODO image and mesh scene graph has to be taken into account

    auto meshAccess = mesh->getOpenCLAccess(ACCESS_READ, device);
    reportInfo() << "Got mesh opencl access" << reportEnd();
	cl::Program program = getOpenCLProgram(device);
	cl::CommandQueue queue = device->getCommandQueue();
	if(is2D) {
		cl::Kernel kernel(program, "mesh_to_segmentation_2d");
		OpenCLImageAccess::pointer outputAccess = segmentation->getOpenCLImageAccess(ACCESS_READ_WRITE, device);
		kernel.setArg(0, *meshAccess->getCoordinatesBuffer());
		kernel.setArg(1, *meshAccess->getLineBuffer());
        kernel.setArg(2, mesh->getNrOfLines());
		kernel.setArg(3, *outputAccess->get2DImage());
		kernel.setArg(4, segmentation->getSpacing().x());
		kernel.setArg(5, segmentation->getSpacing().y());
		queue.enqueueNDRangeKernel(
				kernel,
				cl::NullRange,
				cl::NDRange(segmentation->getWidth(), segmentation->getHeight()),
				cl::NullRange
		);
	} else {
		cl::Kernel kernel(program, "mesh_to_segmentation_3d");
		auto outputAccess = segmentation->getOpenCLBufferAccess(ACCESS_READ_WRITE, device);
		kernel.setArg(0, *meshAccess->getCoordinatesBuffer());
		kernel.setArg(1, *meshAccess->getTriangleBuffer());
		kernel.setArg(2, mesh->getNrOfTriangles());
		kernel.setArg(3, *outputAccess->get());
		kernel.setArg(4, segmentation->getSpacing().x());
		kernel.setArg(5, segmentation->getSpacing().y());
		kernel.setArg(6, segmentation->getSpacing().z());
		queue.enqueueNDRangeKernel(
				kernel,
				cl::NullRange,
				cl::NDRange(segmentation->getWidth(), segmentation->getHeight(), segmentation->getDepth()),
				cl::NullRange
		);
	}
    queue.finish();
	addOutputData(0, segmentation);
}

}
