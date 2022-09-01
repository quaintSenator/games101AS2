// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <iostream>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h> 



rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f p(x,y,1);
    return ((_v[1]-_v[0]).cross(p-_v[0])).z()>0 &&
    ((_v[2]-_v[1]).cross(p-_v[1])).z()>0 &&
    ((_v[0]-_v[2]).cross(p-_v[2])).z()>0;
  

}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

float my_interpolation(float x,float y,const Triangle t,float weight0,float weight1,float weight2){
    auto v = t.toVector4();
    float x0 = v[0].x() / v[0].w();
    float y0 = v[0].y() / v[0].w();

    float x1 = v[1].x() / v[1].w();
    float y1 = v[1].y() / v[1].w();

    float x2 = v[2].x() / v[2].w();
    float y2 = v[2].y() / v[2].w();

    float my_z_interpolation = (x - x0) * (y1 - y0) * (weight2 - weight0) + 
    (y - y0) * (weight1 - weight0) * (x2 - x0)
     - (x1 - x0) * (y - y0) * (weight2 - weight0) 
     - (x - x0) * (y2 - y0) * (weight1 - weight0);
    my_z_interpolation /= ((x2 - x0) * (y1 - y0) - (x1 - x0) * (y2 - y0));
    my_z_interpolation += weight0;
    return my_z_interpolation;
}
//Screen space rasterization
void rst::rasterizer::rasterize_triangle(Triangle &t)
{
    auto v = t.toVector4();
    float l = INT_MAX;
    float r = INT_MIN;
    float top = INT_MIN;
    float b = INT_MAX;
    for (auto k : v)
    {
        l = k.x() < l ? k.x() : l;
        r = k.x() > r ? k.x() : r;
        top = k.y() > top ? k.y() : top;
        b = k.y() < b ? k.y() : b;
    }

    for (int i = (int)l; i <= r; i++)
    {
        for (int j = (int)b; j <= top; j++)
        {

            //maintain 4 subpixel z
            //if inside,count+=0.25
            //queue 4 samples
            //sample[0] (0.25,0.25)
            //sample[1] (0.25,0.75)
            //sample[2] (0.75,0.75)
            //sample[3] (0.75,0.25)
            float a[5]={0.25,0.25,0.75,0.75,0.25};
            float count=0;
            float min_z=INT_MAX;
            for(int k=0;k<=3;k++){
                float x=i+a[k];
                float y=j+a[k+1];
                if(insideTriangle(x,y,t.v)){
                    count+=0.25;
                    float tocul0=t.v[0].z();
                    float tocul1=t.v[1].z();
                    float tocul2=t.v[2].z();
                    float z_interpolation = my_interpolation(x,y,t,tocul0,tocul1,tocul2);
                    if(-z_interpolation<min_z)min_z=-z_interpolation;
                }
            }
            Eigen::Vector3f c0=t.getColor(0);
            Eigen::Vector3f c1=t.getColor(1);
            Eigen::Vector3f c2=t.getColor(2);
                //std::cout<<c0.x()<<" "<<c0.y()<<" "<<c0.z()<<std::endl;
                //std::cout<<c1.x()<<" "<<c1.y()<<" "<<c1.z()<<std::endl;
                //std::cout<<c2.x()<<" "<<c2.y()<<" "<<c2.z()<<std::endl;
            float x=i+0.5;
            float y=j+0.5;
            float my_r=my_interpolation(x,y,t,c0.x(),c1.x(),c2.x());
            float my_g=my_interpolation(x,y,t,c0.y(),c1.y(),c2.y());
            float my_b=my_interpolation(x,y,t,c0.z(),c1.z(),c2.z());
  
            Eigen::Vector3f myColor((int)my_r*count,(int)my_g*count,(int)my_b*count);
                
                if(min_z<depth_buf[get_index(i,j)]){
                    //The larger depth, the nearer the point is, which should be repainted
                    depth_buf[get_index(i,j)]=min_z;
                    set_pixel(Eigen::Vector3f(i, j, 1), myColor);
                    //std::cout<<myColor.x()<<" "<<myColor.y()<<" "<<myColor.z()<<std::endl;

                    //Depth_buf store positive number
                } 
            
                              
            
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

// clang-format on