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


static bool insideTrianglePixel(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f A = _v[0], B = _v[1], C = _v[2];
    Eigen::Vector3f ab, bc, ca, ap, bp, cp;
    float x_f = x + 0.5, y_f = y + 0.5;
    ab << B.x() - A.x(), B.y() - A.y(), 0;
    bc << C.x() - B.x(), C.y() - B.y(), 0;
    ca << A.x() - C.x(), A.y() - C.y(), 0;
    ap << x_f - A.x(), y_f - A.y(), 0;
    bp << x_f - B.x(), y_f - B.y(), 0;
    cp << x_f - C.x(), y_f - C.y(), 0;

    // std::cout << A << " \n \n "  << B  << "\n \n" << C << std::endl;

    // std::cout << ab.cross(-ca).z() << "\n" <<bc.cross(-ab).z() << std::endl;
    // exit(0);

    bool f1, f2, f3;
    f1 = ( ab.cross(-ca).z() * ab.cross(ap).z() > 0 );
    f2 = ( bc.cross(-ab).z() * bc.cross(bp).z() > 0 );
    f3 = ( ca.cross(-bc).z() * ca.cross(cp).z() > 0 );

    return f1 && f2 && f3;

}


static bool insideTriangleCoordinate(Eigen::Vector3f P, const Eigen::Vector3f* t)
{
    Eigen::Vector3f A = t[0], B = t[1], C = t[2];
    Eigen::Vector3f ab, bc, ca, ap, bp, cp;

    ab = B-A;
    bc = C-B;
    ca = A-C;
    ap = P-A;
    bp = P-B;
    cp = P-C;

    bool f1, f2, f3;
    f1 = ( ab.cross(-ca).z() * ab.cross(ap).z() > 0 );
    f2 = ( bc.cross(-ab).z() * bc.cross(bp).z() > 0 );
    f3 = ( ca.cross(-bc).z() * ca.cross(cp).z() > 0 );

    return f1 && f2 && f3;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    // 求解线性方程使得ax0+bx1+cx2=x;ay0+by1+cy2=y;a+b+c=1;
    // 将三阶线性方程组直接解出来就是这样，没什么好说的。
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) 
                / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) 
                / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) 
                / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
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
    
    // TODO : Find out the bounding box Sof current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    // What can I say?
    Eigen::Vector3f v1 = Eigen::Vector3f(v[0].head<3>()) / v[0].w();
    Eigen::Vector3f v2 = Eigen::Vector3f(v[1].head<3>()) / v[1].w();
    Eigen::Vector3f v3 = Eigen::Vector3f(v[2].head<3>()) / v[2].w();
    // Eigen::Vector3f triangle[] = {v1, v2, v3};

    // x, y scale to pixel space:
    int w, h;
    w = this->width;
    h = this->height;
    // v1.x() = (v1.x() + 1) * w / 2;
    // v1.y() = (v1.y() + 1) * h / 2;
    // record the (foating point) range of the triangle
    float x_max_f = v1.x(), x_min_f = x_max_f, y_max_f = v1.y(), y_min_f = y_max_f;

    // v2.x() = (v2.x() + 1) * w / 2;
    // v2.y() = (v2.y() + 1) * h / 2;
    if (v2.x() > x_max_f) x_max_f=v2.x();
    if (v2.y() > y_max_f) y_max_f=v2.y();
    if (v2.x() < x_min_f) x_min_f=v2.x();
    if (v2.y() < y_min_f) y_min_f=v2.y();

    // v3.x() = (v3.x() + 1) * w / 2;
    // v3.y() = (v3.y() + 1) * h / 2;
    if (v3.x() > x_max_f) x_max_f=v3.x();
    if (v3.y() > y_max_f) y_max_f=v3.y();
    if (v3.x() < x_min_f) x_min_f=v3.x();
    if (v3.y() < y_min_f) y_min_f=v3.y();

    // change the floating point coordinates into pixel coordinates
    int x_min = (int) x_min_f, x_max = (int)x_max_f + 1, y_min = (int) y_min_f, y_max = (int) y_max_f + 1;
    // std::cout << x_min_f << "  " <<x_max_f << "  " <<y_min_f << "  " << y_max_f << std::endl;
    for (int x = x_min; x < x_max; x++){
        if (x==w)break;
        for (int y = y_min; y < y_max; y++){
            if (y==h)break;

            /* Non-AntiAliasing Version:*/
            if (! insideTrianglePixel(x, y, t.v)) continue;
            // calculate the "z" coordinate:
            float x_f = x+0.5, y_f=y+0.5;
            auto [alpha, beta, gamma] = computeBarycentric2D(x_f, y_f, t.v);
            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;
            int index_buf = get_index(x, y);
            if (depth_buf[index_buf] > -z_interpolated)
            {
                set_pixel(Eigen::Vector3f(x,y, z_interpolated), t.getColor());
                depth_buf[index_buf] = -z_interpolated;
            }


            /* AntiAliasing Version: */
            // Eigen::Vector3f p{x,y,1.0f}, color {0,0,0};
            // bool flag = false;
            // int index_buf = get_index(x, y);
            // float z_interpolated;
            // for (auto p_diff: {Eigen::Vector3f(0.25, 0.25, 0), 
            //                 Eigen::Vector3f(0.75, 0.25, 0),
            //                 Eigen::Vector3f(0.25, 0.75, 0),
            //                 Eigen::Vector3f(0.75, 0.75, 0)})
            // {
            //     if (insideTriangleCoordinate(p+p_diff, t.v))
            //     {
            //         color += t.getColor();
            //         // calculate z once, sorry 'bout that
            //         // cannot deal with the strange black margin in the intersection of two triangles
            //         if (!flag)
            //         {
            //             auto [alpha, beta, gamma] = computeBarycentric2D((p+p_diff).x(), (p+p_diff).y(), t.v);
            //             float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            //             z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            //             z_interpolated *= w_reciprocal;
            //         }
            //         flag = true;
            //     };
            // }
            // color /= 4;
            // if (flag && (depth_buf[index_buf] > -z_interpolated))
            // {
            //     depth_buf[index_buf] = -z_interpolated;
                
            //     set_pixel(Eigen::Vector3f(x,y, z_interpolated), color);
            // }



            // set_pixel(Eigen::Vector3f(x,y,))

        }
    } 
    
    

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
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