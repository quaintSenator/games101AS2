//
// Created by LEI XU on 4/11/19.
//

#include "Triangle.hpp"
#include <algorithm>
#include <array>
#include <iostream>

Triangle::Triangle()
{
    v[0] << 0, 0, 0;
    v[1] << 0, 0, 0;
    v[2] << 0, 0, 0;

    color[0] << 217.0, 238.0, 185.0;
    color[1] << 238.0, 217.0, 185.0;
    color[2] << 10.0, 217.0, 238.0;

    tex_coords[0] << 0.0, 0.0;
    tex_coords[1] << 0.0, 0.0;
    tex_coords[2] << 0.0, 0.0;
}

void Triangle::setVertex(int ind, Vector3f ver)
{
    v[ind] = ver;
}
void Triangle::setNormal(int ind, Vector3f n)
{
    normal[ind] = n;
}
void Triangle::setColor(int ind, float r, float g, float b)
{
    if ((r < 0.0) || (r > 255.) ||
        (g < 0.0) || (g > 255.) ||
        (b < 0.0) || (b > 255.))
    {
        fprintf(stderr, "ERROR! Invalid color values");
        fflush(stderr);
        exit(-1);
    }

    color[ind] = Vector3f((float)r / 255., (float)g / 255., (float)b / 255.);
    return;
}
Vector3f Triangle::getColor(int ind)
{
    float x = color[ind].x()*255;
    float y = color[ind].y()*255;
    float z = color[ind].z()*255;
    //std::cout << "getColor:" << x << " " << y << " " << z << std::endl;
    return Vector3f(x, y, z);
}
void Triangle::setTexCoord(int ind, float s, float t)
{
    tex_coords[ind] = Vector2f(s, t);
}

std::array<Vector4f, 3> Triangle::toVector4() const
{
    std::array<Eigen::Vector4f, 3> res;
    std::transform(std::begin(v), std::end(v), res.begin(), [](auto &vec)
                   { return Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 1.f); });
    return res;
}
