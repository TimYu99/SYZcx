// array_data.cpp

#include "array_data.h"
#include <iostream>
#include <vector>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
// 全局数组定义和初始化
//const int rows = 379;
//const int cols = 401;

std::vector<Centroid> trajectory;
Centroid last_centroid;

int process_frame_difference(cv::Mat frame1, cv::Mat frame2, int frame_index, std::vector<Centroid>& XX, std::vector<Centroid>& YY, int& large_movement_count, int& no_target)
{
    // 初始化
    if (frame_index <= 3) {
        trajectory.clear();
        last_centroid = { 0, 0 };
    }

    // 计算帧间差异
    cv::Mat diff;
    cv::absdiff(frame2, frame1, diff);

    // 计算差异的统计特性
    cv::Scalar mean_diff, std_diff;
    cv::meanStdDev(diff, mean_diff, std_diff);

    // 设置动态阈值
    double threshold = mean_diff[0] + 1.5 * std_diff[0];

    // 二值化差分图像
    cv::Mat binary_image;
    cv::threshold(diff, binary_image, threshold, 255, cv::THRESH_BINARY);

    // 后处理：形态学操作去噪
    cv::Mat cleaned_image;
    cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(binary_image, cleaned_image, cv::MORPH_OPEN, se);

    // 去掉小区域
    cv::Mat labeled_image;
    cv::connectedComponents(cleaned_image, labeled_image);

    // 可视化差分图像
    cv::imshow("Frame Difference", diff);
    cv::imshow("Binary Image", binary_image);
    cv::imshow("Cleaned Image", cleaned_image);

    // 获取目标区域的质心
    std::vector<Centroid> centroids;
    for (int i = 0; i < labeled_image.rows; ++i) {
        for (int j = 0; j < labeled_image.cols; ++j) {
            if (labeled_image.at<int>(i, j) > 0) {
                Centroid c = { static_cast<float>(j), static_cast<float>(i) };
                centroids.push_back(c);
            }
        }
    }

    // 检查是否检测到目标
    if (centroids.empty()) {
        std::cout << "No target detected!" << std::endl;
        no_target = 1;
        large_movement_count = 0;
        return 0;
    }

    // 如果 trajectory 为空，初始化为当前质心
    if (trajectory.empty()) {
        trajectory = centroids;
        last_centroid = centroids.front();
        large_movement_count = 0;
    }
    else {
        // 计算当前质心与上一帧质心的距离
        float min_distance = FLT_MAX;
        Centroid closest_centroid;

        for (const auto& c : centroids) {
            float distance = std::sqrt(std::pow(c.x - last_centroid.x, 2) + std::pow(c.y - last_centroid.y, 2));
            if (distance < min_distance) {
                min_distance = distance;
                closest_centroid = c;
            }
        }

        // 判断是否有较大位移
        float large_movement_threshold = 5.0f;  // 5像素阈值
        if (min_distance > large_movement_threshold) {
            large_movement_count = 1;
        }
        else
        {
            large_movement_count = 0;
        }

        trajectory.push_back(closest_centroid);
        last_centroid = closest_centroid;
    }
    

    no_target = 0;

    return 1;
}





// 打印某个位置的值函数定义
void printValueAt(int row, int col) {
    if (row >= 0 && row < rows && col >= 0 && col < cols) {
        //std::cout << "Value at (" << row << ", " << col << ") is: " << beijingtu21[row][col] << std::endl;
    }
    else {
        std::cout << "Index out of bounds." << std::endl;
    }
}
// 矩阵相减函数定义subtractArrays(array1, array2, result);
void subtractArrays(const int arr1[rows][cols], const int arr2[rows][cols], int result[rows][cols]) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[i][j] = arr1[i][j] - arr2[i][j];
        }
    }
}
// 统计数组里存在各区域值阈值
std::vector<int> countIntervals(const std::vector<int>& arr) {
    const int interval = 10000;
    const int numIntervals = 7;
    std::vector<int> counts(numIntervals, 0);

    for (int value : arr) {
        if (value >= 0 && value <= 65535) {
            int index = value / interval;
            if (index >= numIntervals) {
                index = numIntervals - 1;
            }
            counts[index]++;
        }
    }

    return counts;
}
struct IntervalCounts {
    int count_0_9999;
    int count_10000_19999;
    int count_20000_29999;
    int count_30000_39999;
    int count_40000_49999;
    int count_50000_59999;
    int count_60000_65535;
};
// 函数用于计算每个区间的累计数量
IntervalCounts accumulateCounts(const std::vector<int>& counts) {
    IntervalCounts accumulatedCounts = { 0, 0, 0, 0, 0, 0, 0 };
    accumulatedCounts.count_0_9999 = counts[0];
    accumulatedCounts.count_10000_19999 = accumulatedCounts.count_0_9999 + counts[1];
    accumulatedCounts.count_20000_29999 = accumulatedCounts.count_10000_19999 + counts[2];
    accumulatedCounts.count_30000_39999 = accumulatedCounts.count_20000_29999 + counts[3];
    accumulatedCounts.count_40000_49999 = accumulatedCounts.count_30000_39999 + counts[4];
    accumulatedCounts.count_50000_59999 = accumulatedCounts.count_40000_49999 + counts[5];
    accumulatedCounts.count_60000_65535 = accumulatedCounts.count_50000_59999 + counts[6];
    return accumulatedCounts;
}
// 根据输入角度计算对应的列索引
int getColumnIndex(double angle) {
    if (angle < 0.0 || angle > 360.0) {
        throw std::out_of_range("Angle must be between 0 and 360 degrees.");
    }
    // 每列代表0.9度
    int colIndex = static_cast<int>(std::round(angle / 0.9));
    if (colIndex >= cols) {
        colIndex = cols - 1; // 防止索引越界
    }
    return colIndex;
}

// 获取对应角度的那一列数据
std::vector<int> getColumnData(double angle) {
    int colIndex = getColumnIndex(angle);
    std::vector<int> columnData(rows);
    for (int i = 0; i < rows; ++i) {
        //columnData[i] = beijingtu21[i][colIndex];
    }
    return columnData;
}