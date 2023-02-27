#include "AlphaBlendingVolumeRenderer.hpp"
#include "FAST/Data/Image.hpp"

namespace fast {


void AlphaBlendingVolumeRenderer::draw(Matrix4f perspectiveMatrix, Matrix4f viewingMatrix, float zNear, float zFar,
                                       bool mode2D, int viewWidth,
                                       int viewHeight) {
    // Get window/viewport size
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    const float aspectRatio = (float)viewport[2] / viewport[3];
    const int height = std::min(768, viewport[3]);
    const Vector2i gridSize(aspectRatio*height, height);

    OpenCLDevice::pointer device = std::dynamic_pointer_cast<OpenCLDevice>(getMainDevice());
    auto queue = device->getCommandQueue();
    auto mKernel = cl::Kernel(getOpenCLProgram(device), "volumeRender");

    // Get color data from the main FBO to use as input
    int mainFBO;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &mainFBO);
    int colorTextureID, depthTextureID;
    glGetFramebufferAttachmentParameteriv(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &colorTextureID);
    glGetFramebufferAttachmentParameteriv(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &depthTextureID);

    // Resize OpenGL textures to avoid issues when viewport is very large (4k screens for instance)
    // This also deals with issues related to gridSize being different than the viewport size giving problems when rendering geometry
    auto newTextures = resizeOpenGLTexture(mainFBO, colorTextureID, depthTextureID, gridSize, viewport[2], viewport[3]);
    colorTextureID = std::get<0>(newTextures);
    depthTextureID = std::get<1>(newTextures);

    std::vector<cl::Memory> v;
    // Image objects must exist until kernel has executed
    cl::Image2D inputColor;
    cl::Image2D inputDepth;
    cl::ImageGL inputColorGL;

	bool useGLInterop = false;
	if (device->isOpenGLInteropSupported()) {
		try {
			inputColorGL = textureToCLimageInterop(colorTextureID, gridSize.x(), gridSize.y(), device, false);
			v.push_back(inputColorGL);
			queue.enqueueAcquireGLObjects(&v);
			//cl::ImageGL inputDepth = textureToCLimageInterop(depthTextureID, viewport[2], viewport[3], device, true); // Can't to interop on depth texture..
			inputDepth = textureToCLimage(depthTextureID, gridSize.x(), gridSize.y(), device, true);
			mKernel.setArg(4, inputColorGL);
			mKernel.setArg(5, inputDepth);
			useGLInterop = true;
		}
		catch (cl::Error &e) {
			reportError() << "Failed to perform GL interop in volume renderer even though it is enabled on device." << reportEnd();
		}
	}

	if (!useGLInterop) {
		inputColor = textureToCLimage(colorTextureID, gridSize.x(), gridSize.y(), device, false);
		inputDepth = textureToCLimage(depthTextureID, gridSize.x(), gridSize.y(), device, true);
		mKernel.setArg(4, inputColor);
		mKernel.setArg(5, inputDepth);
	}
	glDeleteTextures(1, (uint*)&colorTextureID);
	glDeleteTextures(1, (uint*)&depthTextureID);

    // Create a FBO
    if(m_FBO == 0)
        glGenFramebuffers(1, &m_FBO);

    // Bind the framebuffer to render to it
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_FBO);
    // TODO CL-GL interop

    auto image = cl::Image2D(
            device->getContext(),
            CL_MEM_READ_WRITE,
            cl::ImageFormat(CL_RGBA, CL_FLOAT),
            gridSize.x(), gridSize.y()
    );
    mKernel.setArg(1, image);

    auto input = std::dynamic_pointer_cast<Image>(getDataToRender()[0]);
    if(m_transferFunction.getSize() == 0) {
        // No transfer function selected, choose default based on data type
        switch(input->getDataType()) {
            case TYPE_UINT8:
                m_transferFunction = TransferFunction::Ultrasound();
                break;
            case TYPE_INT16:
                m_transferFunction = TransferFunction::CT_Blood_And_Bone();
                break;
            default:
                throw Exception("Please provide a TransferFunction to the AlphaBlendingVolumeRenderer");
        }
    }
    auto access = input->getOpenCLImageAccess(ACCESS_READ, device);
    cl::Image3D *clImage = access->get3DImage();

    Affine3f modelMatrix = SceneGraph::getEigenTransformFromData(input);
    modelMatrix.scale(input->getSpacing());
    Matrix4f invModelViewMatrix = (viewingMatrix*modelMatrix.matrix()).inverse();
    
    auto inverseModelViewMatrixBuffer = cl::Buffer(
            device->getContext(),
            CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
            16*sizeof(float),
            invModelViewMatrix.data()
    );

	Matrix4f invViewMatrix = viewingMatrix.inverse();
	// Remove translation
	invViewMatrix(0, 3) = 0;
	invViewMatrix(1, 3) = 0;
	invViewMatrix(2, 3) = 0;
    auto inverseViewMatrixBuffer = cl::Buffer(
            device->getContext(),
            CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
            16*sizeof(float),
            invViewMatrix.data()
    );

    auto buffer = m_transferFunction.getAsOpenCLBuffer(device);
    mKernel.setArg(0, *clImage);
    mKernel.setArg(2, inverseModelViewMatrixBuffer);
    mKernel.setArg(3, inverseViewMatrixBuffer);
    mKernel.setArg(6, zNear);
    mKernel.setArg(7, zFar);
    mKernel.setArg(8, buffer);
    mKernel.setArg(9, m_transferFunction.getSize());
    queue.enqueueNDRangeKernel(
            mKernel,
            cl::NullRange,
            cl::NDRange(gridSize.x(), gridSize.y()),
            cl::NullRange
    );

    if(useGLInterop) {
        queue.enqueueReleaseGLObjects(&v);
    }

    // Attach texture to framebuffer
    if(m_texture == 0)
        glGenTextures(1, &m_texture);

    auto data = make_uninitialized_unique<float[]>(gridSize.x()*gridSize.y()*4);
    queue.enqueueReadImage(
            image,
            CL_TRUE,
            createOrigoRegion(),
            createRegion(gridSize.x(), gridSize.y(), 1),
            0, 0,
            data.get()
    );

    // Transfer texture data
    glBindTexture(GL_TEXTURE_2D, m_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, gridSize.x(), gridSize.y(), 0, GL_RGBA, GL_FLOAT, data.get());

    // Set texture to FBO
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_texture, 0);

    // Blit/copy the framebuffer to the default framebuffer (window)
    glBindFramebuffer(GL_READ_FRAMEBUFFER, m_FBO);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, mainFBO);
    glBlitFramebuffer(0, 0, gridSize.x(), gridSize.y(), viewport[0], viewport[1], viewport[2], viewport[3], GL_COLOR_BUFFER_BIT, GL_LINEAR);

    // Reset framebuffer to default framebuffer
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, mainFBO);
}

AlphaBlendingVolumeRenderer::AlphaBlendingVolumeRenderer(TransferFunction transferFunction) {
    createOpenCLProgram(Config::getKernelSourcePath() + "/Visualization/VolumeRenderer/AlphaBlendingVolumeRenderer.cl");
    setTransferFunction(transferFunction);
}

void AlphaBlendingVolumeRenderer::setTransferFunction(TransferFunction transferFunction) {
    m_transferFunction = transferFunction;
}

}