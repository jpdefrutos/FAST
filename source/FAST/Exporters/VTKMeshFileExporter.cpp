#include "VTKMeshFileExporter.hpp"
#include "FAST/Data/Mesh.hpp"
#include <fstream>
#include "FAST/SceneGraph.hpp"

namespace fast {


VTKMeshFileExporter::VTKMeshFileExporter() {
    createInputPort<Mesh>(0);
    mWriteNormals = false;
    mWriteColors = false;
}

VTKMeshFileExporter::VTKMeshFileExporter(std::string filename, bool writeNormals, bool writeColors) : FileExporter(filename) {
    createInputPort<Mesh>(0);
    setWriteNormals(writeNormals);
    setWriteColors(writeColors);
}

void VTKMeshFileExporter::setWriteNormals(bool writeNormals) {
    mWriteNormals = writeNormals;
    setModified(true);
}

void VTKMeshFileExporter::setWriteColors(bool writeColors)  {
    mWriteColors = writeColors;
    setModified(true);
}

void VTKMeshFileExporter::execute() {
    if(m_filename == "")
        throw Exception("No filename given to the VTKMeshFileExporter");

    Mesh::pointer mesh = getInputData<Mesh>();

    // Get transformation
    auto transform = SceneGraph::getEigenTransformFromData(mesh);

    std::ofstream file(m_filename.c_str());

    if(!file.is_open())
        throw Exception("Unable to open the file " + m_filename);

    // Write header
    file << "# vtk DataFile Version 3.0\n"
            "vtk output\n"
            "ASCII\n"
            "DATASET POLYDATA\n";

    // Write vertices
    MeshAccess::pointer access = mesh->getMeshAccess(ACCESS_READ);
    std::vector<MeshVertex> vertices = access->getVertices();
    file << "POINTS " << vertices.size() << " float\n";
    for(int i = 0; i < vertices.size(); i++) {
        MeshVertex vertex = vertices[i];
        vertex.getPosition() = (transform.matrix()*vertex.getPosition().homogeneous()).head(3);
        file << vertex.getPosition().x() << " " << vertex.getPosition().y() << " " << vertex.getPosition().z() << "\n";
    }

    if(mesh->getNrOfTriangles() > 0) {
        std::vector<MeshTriangle> triangles = access->getTriangles();
        // Write triangles
        file << "POLYGONS " << mesh->getNrOfTriangles() << " " << mesh->getNrOfTriangles() * 4 << "\n";
        for(MeshTriangle triangle : triangles) {
            file << "3 " << triangle.getEndpoint1() << " " << triangle.getEndpoint2() << " " << triangle.getEndpoint3()
                 << "\n";
        }
    }
    if(mesh->getNrOfLines() > 0) {
    	// Write lines
		std::vector<MeshLine> lines = access->getLines();
        file << "LINES " << mesh->getNrOfLines() << " " << mesh->getNrOfLines() * 3 << "\n";
        for(MeshLine line : lines) {
            file << "2 " << line.getEndpoint1() << " " << line.getEndpoint2() << "\n";
        }
    }

    if(mWriteNormals) {
        file << "POINT_DATA " << vertices.size() << "\n";
        file << "NORMALS Normals float\n";
        for(int i = 0; i < vertices.size(); i++) {
            MeshVertex vertex = vertices[i];
            VectorXf normal = vertex.getNormal();

            normal = transform.linear() * normal; // Transform the normal

            // Normalize it
            float length = normal.norm();
            if(length == 0) { // prevent NaN situations
                file << "0 1 0\n";
            } else {
                normal.normalize();
                file << normal.x() << " " << normal.y() << " " << normal.z() << "\n";
            }
        }
    }

    if(mWriteColors) {
        file << "POINT_DATA " << vertices.size() << "\n";
        file << "VECTORS vertex_colors float\n";
        for(int i = 0; i < vertices.size(); i++) {
            MeshVertex vertex = vertices[i];
            Color color = vertex.getColor();
            file << color.getRedValue() << " " << color.getGreenValue() << " " << color.getBlueValue() << "\n";
        }
    }

    file.close();
}

}
