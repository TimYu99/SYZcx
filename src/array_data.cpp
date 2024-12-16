// array_data.cpp

#include "array_data.h"
#include <iostream>
#include <vector>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
// ȫ�����鶨��ͳ�ʼ��
//const int rows = 379;
//const int cols = 401;

std::vector<Centroid> trajectory;
Centroid last_centroid;

int process_frame_difference(cv::Mat frame1, cv::Mat frame2, int frame_index, std::vector<Centroid>& XX, std::vector<Centroid>& YY, int& large_movement_count, int& no_target)
{
    // ��ʼ��
    if (frame_index <= 3) {
        trajectory.clear();
        last_centroid = { 0, 0 };
    }

    // ����֡�����
    cv::Mat diff;
    cv::absdiff(frame2, frame1, diff);

    // ��������ͳ������
    cv::Scalar mean_diff, std_diff;
    cv::meanStdDev(diff, mean_diff, std_diff);

    // ���ö�̬��ֵ
    double threshold = mean_diff[0] + 1.5 * std_diff[0];

    // ��ֵ�����ͼ��
    cv::Mat binary_image;
    cv::threshold(diff, binary_image, threshold, 255, cv::THRESH_BINARY);

    // ������̬ѧ����ȥ��
    cv::Mat cleaned_image;
    cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(binary_image, cleaned_image, cv::MORPH_OPEN, se);

    // ȥ��С����
    cv::Mat labeled_image;
    cv::connectedComponents(cleaned_image, labeled_image);

    // ���ӻ����ͼ��
    cv::imshow("Frame Difference", diff);
    cv::imshow("Binary Image", binary_image);
    cv::imshow("Cleaned Image", cleaned_image);

    // ��ȡĿ�����������
    std::vector<Centroid> centroids;
    for (int i = 0; i < labeled_image.rows; ++i) {
        for (int j = 0; j < labeled_image.cols; ++j) {
            if (labeled_image.at<int>(i, j) > 0) {
                Centroid c = { static_cast<float>(j), static_cast<float>(i) };
                centroids.push_back(c);
            }
        }
    }

    // ����Ƿ��⵽Ŀ��
    if (centroids.empty()) {
        std::cout << "No target detected!" << std::endl;
        no_target = 1;
        large_movement_count = 0;
        return 0;
    }

    // ��� trajectory Ϊ�գ���ʼ��Ϊ��ǰ����
    if (trajectory.empty()) {
        trajectory = centroids;
        last_centroid = centroids.front();
        large_movement_count = 0;
    }
    else {
        // ���㵱ǰ��������һ֡���ĵľ���
        float min_distance = FLT_MAX;
        Centroid closest_centroid;

        for (const auto& c : centroids) {
            float distance = std::sqrt(std::pow(c.x - last_centroid.x, 2) + std::pow(c.y - last_centroid.y, 2));
            if (distance < min_distance) {
                min_distance = distance;
                closest_centroid = c;
            }
        }

        // �ж��Ƿ��нϴ�λ��
        float large_movement_threshold = 5.0f;  // 5������ֵ
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





// ��ӡĳ��λ�õ�ֵ��������
void printValueAt(int row, int col) {
    if (row >= 0 && row < rows && col >= 0 && col < cols) {
        //std::cout << "Value at (" << row << ", " << col << ") is: " << beijingtu21[row][col] << std::endl;
    }
    else {
        std::cout << "Index out of bounds." << std::endl;
    }
}
// ���������������subtractArrays(array1, array2, result);
void subtractArrays(const int arr1[rows][cols], const int arr2[rows][cols], int result[rows][cols]) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[i][j] = arr1[i][j] - arr2[i][j];
        }
    }
}
// ͳ����������ڸ�����ֵ��ֵ
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
// �������ڼ���ÿ��������ۼ�����
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
// ��������Ƕȼ����Ӧ��������
int getColumnIndex(double angle) {
    if (angle < 0.0 || angle > 360.0) {
        throw std::out_of_range("Angle must be between 0 and 360 degrees.");
    }
    // ÿ�д���0.9��
    int colIndex = static_cast<int>(std::round(angle / 0.9));
    if (colIndex >= cols) {
        colIndex = cols - 1; // ��ֹ����Խ��
    }
    return colIndex;
}

// ��ȡ��Ӧ�Ƕȵ���һ������
std::vector<int> getColumnData(double angle) {
    int colIndex = getColumnIndex(angle);
    std::vector<int> columnData(rows);
    for (int i = 0; i < rows; ++i) {
        //columnData[i] = beijingtu21[i][colIndex];
    }
    return columnData;
}