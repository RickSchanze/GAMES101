// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
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


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f p(x, y, 0);
    Vector3f ba = _v[0] - _v[1], bp = p - _v[1];

    Vector3f cb = _v[1] - _v[2], cp = p - _v[2];

    Vector3f ac = _v[2] - _v[0], ap = p - _v[0];

    Vector3f a = ba.cross(bp), b = cb.cross(cp), c = ac.cross(ap);
    return a.z() > 0 && b.z() > 0 && c.z() > 0;
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

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // 创建bounding box
    auto max_x = std::max({ v[0].x(), v[1].x(), v[2].x() });
    auto min_x = std::min({ v[0].x(), v[1].x(), v[2].x() });
    auto max_y = std::max({ v[0].y(), v[1].y(), v[2].y() });
    auto min_y = std::min({ v[0].y(), v[1].y(), v[2].y() });
    // 遍历
    for (int x = (int)min_x; x < (int)max_x; x++) {
        for (int y = (int)min_y; y < (int)max_y; y++) {
#ifdef MSAA
            int percent = 0;
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 2; j++) {
                    float offsetx = x + (i - 1) * 0.5;
                    float offsety = y + (j - 1) * 0.5;
                    int index = i + j * 2;
                    if (insideTriangle(offsetx, offsety, t.v)) {
                        auto [alpha, beta, gamma] = computeBarycentric2D(offsetx, offsety, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        if (z_interpolated < depth_buf[get_index(x, y)](index)) {
                            depth_buf[get_index(x, y)](index) = z_interpolated;
                            percent += 1;
#ifdef NOBLACKEDGE
                            Vector3f pos(x, y, 0);
                            SetPixelNoBlackEdge(pos, t.getColor(), index);
#endif // NOBLACKEDGE
                        }
                    }
                }
            }
            if (percent > 0) {
                Vector3f point(x, y, 0);
#ifdef NOBLACKEDGE
                Vector3f color = (frame_color_buf[get_index(x, y)](0) +
                    frame_color_buf[get_index(x, y)](1) + 
                    frame_color_buf[get_index(x, y)](2) +
                    frame_color_buf[get_index(x, y)](3)) / 4;
#else
                Vector3f color = t.getColor() * percent / 4;
#endif // NOBLACKEDGE
                set_pixel(point, color);
            }
#else
            if (insideTriangle(x, y, t.v)) {
                auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                if (z_interpolated < depth_buf[get_index(x, y)]) {
                    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                    //深度存入缓存
                    depth_buf[get_index(x, y)] = z_interpolated;
                    Vector3f point = { (float)x,(float)y,z_interpolated };
                    Vector3f color = t.getColor();
                    //着色
                    set_pixel(point, color);
                }
            }
#endif // MSAA
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
#ifdef NOBLACKEDGE
        std::fill(frame_color_buf.begin(), frame_color_buf.end(), Matrix<Vector3f, 1, 4>{Vector3f{ 0, 0, 0 }, Vector3f{ 0, 0, 0 }, Vector3f{ 0, 0, 0 }, Vector3f{ 0, 0, 0 }});
#endif
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }

    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
#ifdef MSAA
        std::fill(depth_buf.begin(), depth_buf.end(), Vector4f(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()));
#else 
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
#endif // MSAA  
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
#ifdef NOBLACKEDGE
    frame_color_buf.resize(w * h);
#endif // NOBLACKEDGE
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

void rst::rasterizer::SetPixelNoBlackEdge(const Eigen::Vector3f& point, const Eigen::Vector3f& color, int index)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_color_buf[ind](index) = color;
}

// clang-format on