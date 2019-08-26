#ifndef UTILS_H
#define UTILS_H

#include <iostream>

#include <opencv2/core/core.hpp>


namespace utils
{

typedef cv::Point3_<uint8_t> Pixel;


bool compareImages(const cv::Mat& img, const cv::Mat& ground_truth)
{
    cv::Size size_img = img.size();
    cv::Size size_ground_truth = ground_truth.size();

    if (size_img != size_img)
    {
        std::cerr << "[Utils][compareImages] Image size is different." << std::endl;
        std::cerr << "[Utils][compareImages] Image size is (" << size_img.width << ", " << size_img.height << ")." << std::endl;
        std::cerr << "[Utils][compareImages] Should be (" << size_ground_truth.width << ", " << size_ground_truth.height << ")." << std::endl;

        return false;
    }

    for (std::size_t i = 0; i < img.rows; ++i)
    {
        for (std::size_t j = 0; j < img.cols; ++j)
        {
            Pixel pixel_img = img.at<Pixel>(i, j);
            Pixel pixel_ground_truth = ground_truth.at<Pixel>(i, j);

            if (pixel_img != pixel_ground_truth)
            {
                std::cerr << "[Utils][compareImages] Found non-equal pixel at (" << i << ", " << j << ")." << std::endl;
                std::cerr << "[Utils][compareImages] Input pixel is [" << static_cast<unsigned int>(pixel_img.x) << ", " << static_cast<unsigned int>(pixel_img.y) << ", " << static_cast<unsigned int>(pixel_img.z) << "]." << std::endl;
                std::cerr << "[Utils][compareImages] Ground truth is [" << static_cast<unsigned int>(pixel_ground_truth.x) << ", " << static_cast<unsigned int>(pixel_ground_truth.y) << ", " << static_cast<unsigned int>(pixel_ground_truth.z) << "]." << std::endl;

                return false;
            }
        }
    }

    return true;
}

}

#endif /* UTILS_H */