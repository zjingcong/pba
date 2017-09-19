//
// Created by jingcoz on 9/14/17.
//

# include "Tools.h"
# include <iostream>
# include <fstream>
# include <sstream>

using namespace pba;
using namespace std;


void Draw::DrawTriangles(GeometryPtr geom)
{
    size_t face_count = geom->get_nb();

    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < face_count; ++i)
    {
        glColor3f( geom->get_face_colors()[i].red(), geom->get_face_colors()[i].green(), geom->get_face_colors()[i].blue() );
        TrianglePtr tri= geom->get_triangles().at(i);
        Vector v0 = tri->getP0();
        Vector v1 = tri->getP1();
        Vector v2 = tri->getP2();
        glVertex3f( GLfloat(v0.X()), GLfloat(v0.Y()), GLfloat(v0.Z()) );
        glVertex3f( GLfloat(v1.X()), GLfloat(v1.Y()), GLfloat(v1.Z()) );
        glVertex3f( GLfloat(v2.X()), GLfloat(v2.Y()), GLfloat(v2.Z()) );
    }
    glEnd();
}


void LoadMesh::LoadObj(std::string obj_path, GeometryPtr geom)
{
    cout << "Load model " << obj_path << "..." << endl;
    // load model file
    const char* file_path = obj_path.c_str();
    ifstream modelFile(file_path);

    string line;
    int vertex_num = 0;
    int face_num = 0;

    std::vector<Vector> vertices;

    while (getline(modelFile, line))
    {
        istringstream iss(line);
        string tag;
        float a, b, c;
        if (iss >> tag >> a >> b >> c)
        {
            // parse the vertices
            if (tag == "v")
            {
                vertices.push_back(Vector(a, b, c));
                vertex_num++;
            }
            // parse the faces
            if (tag == "f")
            {
                // construct triangle geometry
                Vector P0 = vertices.at(size_t(a - 1)); // obj face index starts with 1
                Vector P1 = vertices.at(size_t(b - 1));
                Vector P2 = vertices.at(size_t(c - 1));
                TrianglePtr tri = new Triangle(P0, P1, P2);
                geom->add_triangle(tri);
                face_num++;
            }
        }
    }
    // log the model info
    cout << "vertex_num: " << vertex_num << endl;
    cout << "face_num: " << face_num << endl;
    cout << "Load model success." << endl;
}


void LoadMesh::LoadBox(const float l, GeometryPtr geom)
{
    cout << "Load default box with side length = " << l << "..." << endl;
    std::vector<Vector> vertices;
    std::vector<Vector> face_indices;
    // verts
    float dl = float(l * 0.5);
    vertices.push_back(Vector(-dl, -dl, dl));
    vertices.push_back(Vector(dl, -dl, dl));
    vertices.push_back(Vector(-dl, dl, dl));
    vertices.push_back(Vector(dl, dl, dl));
    vertices.push_back(Vector(-dl, dl, -dl));
    vertices.push_back(Vector(dl, dl, -dl));
    vertices.push_back(Vector(-dl, -dl, -dl));
    vertices.push_back(Vector(dl, -dl, -dl));

    // face indices
    face_indices.push_back(Vector(0, 1, 2));
    face_indices.push_back(Vector(2, 1, 3));
    face_indices.push_back(Vector(2, 3, 4));
    face_indices.push_back(Vector(4, 3, 5));
    face_indices.push_back(Vector(4, 5, 6));
    face_indices.push_back(Vector(6, 5, 7));
    face_indices.push_back(Vector(6, 7, 0));
    face_indices.push_back(Vector(0, 7, 1));
    face_indices.push_back(Vector(1, 7, 3));
    face_indices.push_back(Vector(3, 7, 5));
    face_indices.push_back(Vector(6, 0, 4));
    face_indices.push_back(Vector(4, 0, 2));

    // construct triangle geometry
    geom->gen_triangles(vertices, face_indices);

    cout << "Load model success." << endl;
}
