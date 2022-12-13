//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f GetInterpolationColor(float u, float v) {
        if (u < 0) u = 0;
        if (v < 0) v = 0;
        if (u > 1) u = 1;
        if (v > 1) v = 1;
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto lerp = [](float x, cv::Vec3b color1, cv::Vec3b color2) {
            return color1 + x * (color2 - color1);
        };
        float v_min = std::floor(v_img);
        float u_min = std::floor(u_img);
        float v_max = std::min((float)height, std::ceil(v_img));
        float u_max = std::min((float)width, std::ceil(u_img));

        auto u00 = image_data.at<cv::Vec3b>(v_max, u_min);
        auto u10 = image_data.at<cv::Vec3b>(v_min, u_min);
        auto u01 = image_data.at<cv::Vec3b>(v_max, u_max);
        auto u11 = image_data.at<cv::Vec3b>(v_min, u_max);
        float s = (u_img - u_min) / (u_max - u_min), t = (v_img - v_min) / (v_max - v_min);
        auto u0 = lerp(s, u00, u01);
        auto u1 = lerp(s, u10, u11);
        auto color = lerp(t, u1, u0);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
