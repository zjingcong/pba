//
// Created by jingcoz on 9/14/17.
//

# include "Tools.h"
# include <stdio.h>
# include <iostream>
# include <string.h>
# include <fstream>
# include <sstream>
# include <algorithm>

using namespace pba;
using namespace std;


void Draw::DrawTriangles(GeometryPtr geom, bool showCollision)
{
    glBegin(GL_TRIANGLES);
    for (auto it = geom->get_triangles().cbegin(); it != geom->get_triangles().cend(); ++it)
    {
        TrianglePtr tri = *it;
        if (tri == nullptr) {continue;}
        if (tri->getVisibility())
        {
            Color color = tri->getColor();
            glColor3f( color.red(), color.green(), color.blue());
            if (showCollision)
            {
                // set collision face color
                if (tri->getCollisionStatus())   {glColor3f(0.9, 0.9, 0.9);}
            }

            Vector v0 = tri->getP0();
            Vector v1 = tri->getP1();
            Vector v2 = tri->getP2();
            glVertex3f( GLfloat(v0.X()), GLfloat(v0.Y()), GLfloat(v0.Z()) );
            glVertex3f( GLfloat(v1.X()), GLfloat(v1.Y()), GLfloat(v1.Z()) );
            glVertex3f( GLfloat(v2.X()), GLfloat(v2.Y()), GLfloat(v2.Z()) );
        }
    }
    glEnd();
}


void LoadMesh::LoadObj(std::string obj_path, GeometryPtr geom)
{
    cout << "Load model " << obj_path << "..." << endl;
    // load model file
    const char* file_path = obj_path.c_str();

    int vertex_num = 0;
    int face_num = 0;
    std::vector<Vector> vertices;

    std::vector<float> xvec;
    std::vector<float> yvec;
    std::vector<float> zvec;

    FILE * file = fopen(file_path, "r");
    if( file == NULL ){
        cout << "Impossible to open the file !" << endl;
        return;
    }

    char lineHeader[128];
    int index_num[3] = {0, 0, 0};
    while (true)
    {
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break; // EOF = End Of File. Quit the loop

        // parse the vertices
        if (strcmp(lineHeader, "v") == 0)
        {
            index_num[0] = 1;
            float x, y, z;
            fscanf(file, "%f %f %f\n", &x, &y, &z);

            xvec.push_back(x);
            yvec.push_back(y);
            zvec.push_back(z);
            vertices.push_back(Vector(x, y, z));
            vertex_num++;
        }
        if ( strcmp( lineHeader, "vt" ) == 0 ) {index_num[1] = 1;}
        if ( strcmp( lineHeader, "vn" ) == 0 ) {index_num[2] = 1;}
    }

    file = fopen(file_path, "r");
    while (true)
    {
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break; // EOF = End Of File. Quit the loop

        if (strcmp(lineHeader, "f") == 0)
        {
            unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
            if (index_num[0] == 1 && index_num[1] == 1 && index_num[2] == 1)
            {
                fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0],
                       &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2],
                       &normalIndex[2]);
            }
            else
            {
                fscanf(file, "%d %d %d\n", &vertexIndex[0], &vertexIndex[1], &vertexIndex[2]);
            }
            // construct triangle geometry
            Vector P0 = vertices.at(size_t(vertexIndex[0] - 1)); // obj face index starts with 1
            Vector P1 = vertices.at(size_t(vertexIndex[1] - 1));
            Vector P2 = vertices.at(size_t(vertexIndex[2] - 1));
            TrianglePtr tri = new Triangle(P0, P1, P2);
            geom->add_triangle(tri);
            face_num++;
        }
    }

    // log the model info
    cout << "vertex_num: " << vertex_num << endl;
    cout << "face_num: " << face_num << endl;
    cout << "Load model success." << endl;
    // get the model bounding box
    std::sort(xvec.begin(), xvec.end());
    float x_min = xvec.front();
    float x_max = xvec.back();
    std::sort(yvec.begin(), yvec.end());
    float y_min = yvec.front();
    float y_max = yvec.back();
    std::sort(zvec.begin(), zvec.end());
    float z_min = zvec.front();
    float z_max = zvec.back();
    Vector llc = Vector(x_min, y_min, z_min);
    Vector urc = Vector(x_max, y_max, z_max);
    geom->setBBox(llc, urc);
    cout << "Geometry Bounding Box: " << endl;
    cout << "     LLC - (" << llc.X() << ", " << llc.Y() << ", " << llc.Z() << ")" << endl;
    cout << "     URC - (" << urc.X() << ", " << urc.Y() << ", " << urc.Z() << ")" << endl;
    Vector center = geom->getBBox().getCenter();
    cout << "  Center - (" << center.X() << ", " << center.Y() << ", " << center.Z() << ")" << endl;
}


void LoadMesh::LoadObj(std::string obj_path, std::vector<Vector> &vertices, AABB& bbox)
{
    cout << "Load model " << obj_path << "..." << endl;
    // load model file
    const char* file_path = obj_path.c_str();

    int vertex_num = 0;

    std::vector<float> xvec;
    std::vector<float> yvec;
    std::vector<float> zvec;

    FILE * file = fopen(file_path, "r");
    if( file == NULL ){
        cout << "Impossible to open the file !" << endl;
        return;
    }

    char lineHeader[128];
    while (true)
    {
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break; // EOF = End Of File. Quit the loop

        // parse the vertices
        if (strcmp(lineHeader, "v") == 0)
        {
            float x, y, z;
            fscanf(file, "%f %f %f\n", &x, &y, &z);

            xvec.push_back(x);
            yvec.push_back(y);
            zvec.push_back(z);
            vertices.push_back(Vector(x, y, z));
            vertex_num++;
        }
    }

    // log the model info
    cout << "vertex_num: " << vertex_num << endl;
    // get the model bounding box
    std::sort(xvec.begin(), xvec.end());
    float x_min = xvec.front();
    float x_max = xvec.back();
    std::sort(yvec.begin(), yvec.end());
    float y_min = yvec.front();
    float y_max = yvec.back();
    std::sort(zvec.begin(), zvec.end());
    float z_min = zvec.front();
    float z_max = zvec.back();
    Vector llc = Vector(x_min, y_min, z_min);
    Vector urc = Vector(x_max, y_max, z_max);
    bbox.setLLC(llc);
    bbox.setURC(urc);
    cout << "Geometry Bounding Box: " << endl;
    cout << "     LLC - (" << llc.X() << ", " << llc.Y() << ", " << llc.Z() << ")" << endl;
    cout << "     URC - (" << urc.X() << ", " << urc.Y() << ", " << urc.Z() << ")" << endl;
    Vector center = bbox.getCenter();
    cout << "  Center - (" << center.X() << ", " << center.Y() << ", " << center.Z() << ")" << endl;
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
    face_indices.push_back(Vector(6, 7, 0));
    face_indices.push_back(Vector(0, 7, 1));
    face_indices.push_back(Vector(1, 7, 3));
    face_indices.push_back(Vector(3, 7, 5));
    face_indices.push_back(Vector(6, 0, 4));
    face_indices.push_back(Vector(4, 0, 2));
    face_indices.push_back(Vector(4, 5, 6));
    face_indices.push_back(Vector(6, 5, 7));

    // construct triangle geometry
    geom->build_triangles(vertices, face_indices);
    geom->get_triangles().at(10)->setVisible(false);
    geom->get_triangles().at(11)->setVisible(false);
    Vector llc = Vector(-dl - 0.0001, -dl - 0.0001, -dl - 0.0001);
    Vector urc = Vector(dl + 0.0001, dl + 0.0001, dl + 0.0001);
    geom->setBBox(llc, urc);

    cout << "Load model success." << endl;
    cout << "Geometry Bounding Box: " << endl;
    cout << "     LLC - (" << llc.X() << ", " << llc.Y() << ", " << llc.Z() << ")" << endl;
    cout << "     URC - (" << urc.X() << ", " << urc.Y() << ", " << urc.Z() << ")" << endl;
    Vector center = geom->getBBox().getCenter();
    cout << "  Center - (" << center.X() << ", " << center.Y() << ", " << center.Z() << ")" << endl;
}


void LoadMesh::LoadPlane(const Vector& center, const double &size, const int &division, GeometryPtr geom)
{
    cout << "Load plane size = " << size << ", division number = " << division << "..." << endl;
    double grid_size = size / division;
    double l_x = center.X() - size * 0.5;
    double l_z = center.Z() - size * 0.5;
    for (int m = 0; m < division; ++m)  // z
    {
        for (int n = 0; n < division; ++n)  // x
        {
            Vector p00 = Vector(l_x + m * grid_size, center.Y(), l_z + n * grid_size);
            Vector p01 = Vector(l_x + (m + 1) * grid_size, center.Y(), l_z + (n + 1) * grid_size);
            Vector p02 = Vector(l_x + m * grid_size, center.Y(), l_z + (n + 1) * grid_size);
            geom->add_triangle(p00, p01, p02);

            Vector p10 = p00;
            Vector p11 = Vector(l_x + (m + 1) * grid_size, center.Y(), l_z + n * grid_size);
            Vector p12 = p01;
            geom->add_triangle(p10, p11, p12);
        }
    }
    ///////////////////
    // p00     p10-011
    //  | \      \ |
    // p02-p01   p12
    //////////////////
    Vector llc = Vector(l_x, center.Y() - 0.0001, l_z);
    Vector urc = Vector(center.X() + size * 0.5, center.Y() + 0.0001 + 0.0001, center.Z() + size * 0.5);
    geom->setBBox(llc, urc);
    cout << "Load plane success." << endl;
    cout << "Triangles number: " << geom->get_triangles().size() << endl;
    cout << "Geometry Bounding Box: " << endl;
    cout << "     LLC - (" << llc.X() << ", " << llc.Y() << ", " << llc.Z() << ")" << endl;
    cout << "     URC - (" << urc.X() << ", " << urc.Y() << ", " << urc.Z() << ")" << endl;
}
