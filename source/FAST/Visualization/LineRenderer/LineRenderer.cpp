#include "LineRenderer.hpp"
#include "FAST/Data/Access/MeshAccess.hpp"
#include "FAST/Data/Mesh.hpp"
#include "FAST/SceneGraph.hpp"
#include <FAST/Visualization/View.hpp>

#if defined(__APPLE__) || defined(__MACOSX)
#else
#include <GL/gl.h>
#endif

namespace fast {

void LineRenderer::draw(Matrix4f perspectiveMatrix, Matrix4f viewingMatrix, float zNear, float zFar, bool mode2D,
                        int viewWidth,
                        int viewHeight) {

    std::string shaderName = mode2D ? "2D" : "3D";
    activateShader(shaderName);
    setShaderUniform("perspectiveTransform", perspectiveMatrix, shaderName);
    setShaderUniform("viewTransform", viewingMatrix, shaderName);
    // For all input data
    auto dataToRender = getDataToRender();
    for(auto it : dataToRender) {
        Mesh::pointer points = std::static_pointer_cast<Mesh>(it.second);

        // Delete old VAO
        if(mVAO.count(it.first) > 0) {
            glDeleteVertexArrays(1, &mVAO[it.first]);
        }
        // Create VAO
        uint VAO_ID;
        glGenVertexArrays(1, &VAO_ID);
        mVAO[it.first] = VAO_ID;
        glBindVertexArray(VAO_ID);

        Affine3f transform = Affine3f::Identity();
        // If rendering is in 2D mode we skip any transformations
        if(!mode2D) {
            transform = SceneGraph::getEigenTransformFromData(it.second);
        }
        setShaderUniform("transform", transform, shaderName);

        if(mode2D) {
            setShaderUniform("thickness", mDefaultLineWidth, shaderName);
            //setShaderUniform("viewportWidth", viewWidth, shaderName);
            //setShaderUniform("viewportHeight", viewHeight, shaderName);
        }
        bool useGlobalColor = false;
        Color color = Color::Green();
        if(mInputColors.count(it.first) > 0) {
            color = mInputColors[it.first];
            useGlobalColor = true;
        } else if(mDefaultColorSet) {
            color = mDefaultColor;
            useGlobalColor = true;
        }
        bool drawOnTop;
        if(mInputDrawOnTop.count(it.first) > 0) {
            drawOnTop = mInputDrawOnTop[it.first];
        } else {
            drawOnTop = mDefaultDrawOnTop;
        }
        if(drawOnTop)
            glDisable(GL_DEPTH_TEST);

        
        VertexBufferObjectAccess::pointer access = points->getVertexBufferObjectAccess(ACCESS_READ);
        
        // Coordinates
        GLuint* coordinatesVBO = access->getCoordinateVBO();
        glBindBuffer(GL_ARRAY_BUFFER, *coordinatesVBO);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // Color buffer
        if(access->hasColorVBO() && !useGlobalColor) {
            GLuint *colorVBO = access->getColorVBO();
            glBindBuffer(GL_ARRAY_BUFFER, *colorVBO);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
            glEnableVertexAttribArray(1);
        } else {
            useGlobalColor = true;
        }
        setShaderUniform("useGlobalColor", useGlobalColor, shaderName);
        setShaderUniform("globalColor", color.asVector(), shaderName);

        if(access->hasEBO()) {
            GLuint* EBO = access->getLineEBO();
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *EBO);
            glDrawElements(GL_LINES, points->getNrOfLines() * 2, GL_UNSIGNED_INT, NULL);
        } else {
            // No EBO available; assume all vertices belong to lines consecutively
            glDrawArrays(GL_LINES, 0, points->getNrOfLines() * 2);
        }
        glBindVertexArray(0);

        if(drawOnTop)
            glEnable(GL_DEPTH_TEST);
    }
    deactivateShader();
    if(mode2D) {
        glEnable(GL_POINT_SPRITE); // Circles created in fragment shader will not work without this
        glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
        // Draw joints as points
        shaderName = "2Djoints";
        activateShader(shaderName);
        setShaderUniform("perspectiveTransform", perspectiveMatrix, shaderName);
        setShaderUniform("viewTransform", viewingMatrix, shaderName);
        setShaderUniform("viewportWidth", viewWidth, shaderName);
        auto dataToRender = getDataToRender();
        for(auto it : dataToRender) {
            // Delete old VAO
            if(mVAO.count(it.first) > 0)
                glDeleteVertexArrays(1, &mVAO[it.first]);
            // Create VAO
            uint VAO_ID;
            glGenVertexArrays(1, &VAO_ID);
            mVAO[it.first] = VAO_ID;
            glBindVertexArray(VAO_ID);

            Mesh::pointer points = std::static_pointer_cast<Mesh>(it.second);
            float pointSize = mDefaultLineWidth;
            auto access = points->getVertexBufferObjectAccess(ACCESS_READ);
            bool useGlobalColor = false;
            Color color = Color::Green();
            if(mInputColors.count(it.first) > 0) {
                color = mInputColors[it.first];
                useGlobalColor = true;
            } else if(!mDefaultColor.isNull()) {
                color = mDefaultColor;
                useGlobalColor = true;
            } else if(!access->hasColorVBO()) {
                color = Color::Green();
                useGlobalColor = true;
            }
            Affine3f transform = Affine3f::Identity();
            // If rendering is in 2D mode we skip any transformations
            if(!mode2D) {
                transform = SceneGraph::getEigenTransformFromData(it.second);
            }
            setShaderUniform("transform", transform, shaderName);
            setShaderUniform("pointSize", pointSize, shaderName);

            GLuint* coordinateVBO = access->getCoordinateVBO();

            // Coordinates buffer
            glBindBuffer(GL_ARRAY_BUFFER, *coordinateVBO);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);

            // Color buffer
            if(access->hasColorVBO() && !useGlobalColor) {
                GLuint *colorVBO = access->getColorVBO();
                glBindBuffer(GL_ARRAY_BUFFER, *colorVBO);
                glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
                glEnableVertexAttribArray(1);
            } else {
                useGlobalColor = true;
            }
            setShaderUniform("useGlobalColor", useGlobalColor, shaderName);
            setShaderUniform("globalColor", color.asVector(), shaderName);

            glDrawArrays(GL_POINTS, 0, points->getNrOfVertices());

            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);
        }
        deactivateShader();
    }
    glFinish(); // Fixes random crashes in OpenGL on NVIDIA windows due to some interaction with the text renderer. Suboptimal solution as glFinish is a blocking sync operation.
}

LineRenderer::LineRenderer(Color color, float lineWidth, bool drawOnTop) {
    createInputPort<Mesh>(0, false);
    mDefaultLineWidth = lineWidth;
    mDefaultColor = color;
    mDefaultDrawOnTop = drawOnTop;
    mDefaultColorSet = true;
    createShaderProgram({
        Config::getKernelSourcePath() + "Visualization/LineRenderer/LineRenderer.vert",
        Config::getKernelSourcePath() + "Visualization/LineRenderer/LineRenderer.frag",
        Config::getKernelSourcePath() + "Visualization/LineRenderer/LineRenderer.geom",
    }, "2D");
    createShaderProgram({
        Config::getKernelSourcePath() + "Visualization/LineRenderer/LineRendererJoints.vert",
        Config::getKernelSourcePath() + "Visualization/LineRenderer/LineRendererJoints.frag",
        }, "2Djoints");
    // Drop geom shader for 3D, not supporting thick lines here yet.
    createShaderProgram({
        Config::getKernelSourcePath() + "Visualization/LineRenderer/LineRenderer3D.vert",
        Config::getKernelSourcePath() + "Visualization/LineRenderer/LineRenderer3D.frag",
        }, "3D");
}

uint LineRenderer::addInputConnection(DataChannel::pointer port) {
    return Renderer::addInputConnection(port);
}

uint LineRenderer::addInputConnection(DataChannel::pointer port, Color color,
        float width) {
    uint nr = addInputConnection(port);
    setColor(nr, color);
    setWidth(nr, width);
    return nr;
}

void LineRenderer::setDefaultColor(Color color) {
    mDefaultColor = color;
    mDefaultColorSet = true;
}

void LineRenderer::setDefaultLineWidth(float width) {
    mDefaultLineWidth = width;
}

void LineRenderer::setDefaultDrawOnTop(bool drawOnTop) {
    mDefaultDrawOnTop = drawOnTop;
}

void LineRenderer::setDrawOnTop(uint inputNr, bool drawOnTop) {
    mInputDrawOnTop[inputNr] = drawOnTop;
}

void LineRenderer::setColor(uint nr, Color color) {
    mInputColors[nr] = color;
}

void LineRenderer::setWidth(uint nr, float width) {
    mInputWidths[nr] = width;
}

} // end namespace fast
