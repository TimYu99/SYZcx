// array_data.h

#ifndef ARRAY_DATA_H
#define ARRAY_DATA_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
// 结构体定义
struct Centroid {
    float x;
    float y;
};
// 常量定义
const int rows = 379;
const int cols = 401;


// 函数声明
void printValueAt(int row, int col);
void subtractArrays(const int arr1[rows][cols], const int arr2[rows][cols], int result[rows][cols]);
int process_frame_difference(cv::Mat frame1, cv::Mat frame2, int frame_index,
    std::vector<Centroid>& XX, std::vector<Centroid>& YY,
    int& large_movement_count, int& no_target);
#endif // ARRAY_DATA_H