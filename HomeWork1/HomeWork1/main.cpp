#include "Triangle.h"
#include "rasterizer.h"
#include <Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <format>
#include "Windows.h"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    model << std::cos(rotation_angle / 180 * MY_PI), -std::sin(rotation_angle / 180 * MY_PI), 0, 0,
             std::sin(rotation_angle / 180 * MY_PI), std::cos(rotation_angle / 180 * MY_PI), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
    float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    // 参数准备
    auto t = zNear * std::tan(eye_fov / 180 * MY_PI);
    auto b = -t;
    auto l = t * aspect_ratio;
    auto r = -l;
    // 首先是转换到nNear这个屏幕
    projection << zNear, 0, 0, 0,
                  0, zNear, 0, 0,
                  0, 0, zNear + zFar, -zNear * zFar,
                  0, 0, 1, 0;

    // 然后转换到[-1,1]范围内
    Eigen::Matrix4f orth1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f orth2 = Eigen::Matrix4f::Identity();
    orth1 << (2. / (r - l)), 0, 0, 0,
             0, (2. / (t - b)), 0, 0,
             0, 0, zNear - zFar, 0,
             0, 0, 0, 1.;
    orth2 << 1., 0, 0, -((r + l) / 2.),
             0, 1., 0, -((t + b) / 2.),
             0, 0, 1., -((zNear + zFar) / 2.),
             0, 0, 0, 1.;
    Eigen::Matrix4f orth = orth1 * orth2;

    //// 之后是视口转换
    //Eigen::Matrix4f viewport = Eigen::Matrix4f::Identity();
    //viewport << l, 0, 0, l,
    //            0, t, 0, t,
    //            0, 0, 1, 0,
    //            0, 0, 0, 1;

    return  orth * projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    // 将转轴转到x轴
    float alpha = axis(0) / 180 * MY_PI, beta = axis(1) / 180 * MY_PI, gamma = axis(2) / 180 * MY_PI;
    Eigen::Matrix4f x_to_origin = Matrix4f::Identity();
    // 绕z转-alpha
    x_to_origin << 1, 0, 0, 0,
                   0, cos(-alpha), -sin(-alpha), 0,
                   0, sin(-alpha), cos(-alpha), 0,
                   0, 0, 0, 1;
    // 绕y轴转
    Matrix4f y_to_origin;
    y_to_origin << cos(-gamma), 0, sin(gamma), 0,
                   0, 1, 0, 0,
                   -sin(gamma), 0 ,cos(gamma), 0,
                   0, 0, 0, 1;

    // 先绕x再绕z
    Matrix4f to_origin = y_to_origin * x_to_origin;
    //std::cout << to_origin << std::endl << std::endl;;
    //Sleep(1000);
    // 转回去
    Matrix4f to_origin_inverse = to_origin.inverse();

    // 再绕Z转
    Matrix4f z_rotation;
    z_rotation << cos(angle / 180 * MY_PI), -sin(angle / 180 * MY_PI), 0, 0,
        sin(angle / 180 * MY_PI), cos(angle / 180 * MY_PI), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    auto rtn = to_origin_inverse * z_rotation * to_origin;
    //std::cout << rtn << std::endl << gamma << std::endl;
    return rtn;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = { 0, 0, 5 };

    std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };

    std::vector<Eigen::Vector3i> ind{ {0, 1, 2} };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    Vector3f axis;
    axis << 0, 0, 0;
    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        
        r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        //std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
            std::cout << "angle: " << angle << '\n';
        }
        else if (key == 'd') {
            std::cout << "angle: " << angle << '\n';
            angle -= 10;
        }
        else if (key == 'x') {
            axis(0) += 10;
            std::cout << "axis: (" << axis(0) << ", " << axis(1) << ", " << axis(2) << ")" << '\n';
        }
        else if (key == 'y') {
            axis(1) += 10;
            std::cout << "axis: (" << axis(0) << ", " << axis(1) << ", " << axis(2) << ")" << '\n';
        }
        else if (key == 'z') {
            axis(2) += 10;
            std::cout << "axis: (" << axis(0) << ", " << axis(1) << ", " << axis(2) << ")" << '\n';
        }
    }

    return 0;
}
