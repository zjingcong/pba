//
// Created by jingcoz on 9/14/17.
//

# include "Tools.h"
# include <iostream>
# include <fstream>
# include <sstream>

using namespace pba;
using namespace std;


void Draw::DrawTriangles(std::vector<Vector> vertices, std::vector<Vector> face_indices, std::vector<Color> face_colors)
{
    size_t face_count = face_indices.size();

    glBegin(GL_TRIANGLES);
    for (size_t i = 0; i < face_count; ++i)
    {
        glColor3f( face_colors[i].red(), face_colors[i].green(), face_colors[i].blue() );
        Vector v0 = vertices.at(size_t(face_indices[i].X()));
        Vector v1 = vertices.at(size_t(face_indices[i].Y()));
        Vector v2 = vertices.at(size_t(face_indices[i].Z()));

        glVertex3f( v0.X(), v0.Y(), v0.Z() );
        glVertex3f( v1.X(), v1.Y(), v1.Z() );
        glVertex3f( v2.X(), v2.Y(), v2.Z() );
    }
    glEnd();
}


void LoadMesh::LoadObj(std::string obj_path, std::vector<Vector>& vertices, std::vector<Vector>& face_indices)
{
    cout << "Load model " << obj_path << "..." << endl;
    // load model file
    const char* file_path = obj_path.c_str();
    ifstream modelFile(file_path);

    string line;
    int vertex_num = 0;
    int face_num = 0;

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
                face_indices.push_back(Vector(a - 1, b - 1, c - 1));    // obj face index starts with 1
                face_num++;
            }
        }
    }
    // log the model info
    cout << "vertex_num: " << vertex_num << endl;
    cout << "face_num: " << face_num << endl;
    cout << "Load model success." << endl;
}


void LoadMesh::LoadBox(const float l, std::vector<Vector>& vertices, std::vector<Vector>& face_indices)
{
    cout << "Load default box with side length = " << l << "..." << endl;
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
}
