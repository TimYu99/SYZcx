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

//�������̣�
//����֡����죺���ȣ����� frame1 �� frame2 �Ĳ���ͼ��
//Ŀ���⣺ͨ����ֵ������̬ѧ����ȥ�����ȡĿ����������ġ�
//λ���жϣ������һ֡�����Ĵ��ڣ����㵱ǰ֡��������һ֡������֮��ľ��룬�ж��Ƿ��нϴ�λ�ơ�
//������һ֡�����ģ�����ǰ֡��������Ϊ��һ֡����һ֡���ģ�������ʹ�á�
//����˵����
//frame1 �� frame2����֡ͼ�񣬱�ʾ��������Ƶ֡��
//no_target������Ƿ��⵽Ŀ�꣬0 ��ʾδ��⵽Ŀ�꣬1 ��ʾ��⵽Ŀ�ꡣ
//centroid�����ص�Ŀ�����ģ����� x �� y ���ꡣ
int process_frame_difference(cv::Mat frame1, cv::Mat frame2,int& no_target, Centroid& centroid)
{
    int large_movement_count=0;
    // ��ʼ��
    static Centroid last_centroid = { 0, 0 };  // ��¼��һ�ε�����
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
    bool result = cv::imwrite("D:/ceshi/binary_image.png", binary_image);
    if (result) {
        std::cout << "binary_imageͼ���ѱ��浽�ļ���!" << std::endl;
    }
    else {
        std::cerr << "����ͼ��ʧ��!" << std::endl;
    }

    // ������̬ѧ����ȥ��2025��2��13���޸�
    cv::Mat cleaned_image1;
    cv::Mat se1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(binary_image, cleaned_image1, cv::MORPH_OPEN, se1);
    cv::Mat cleaned_image;
    cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
    cv::morphologyEx(binary_image, cleaned_image, cv::MORPH_OPEN, se);
    // ������̬ѧ����ȥ��
    //cv::Mat cleaned_image1;
    //cv::Mat se1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
    //cv::morphologyEx(binary_image, cleaned_image1, cv::MORPH_OPEN, se1);
    //cv::Mat cleaned_image;
    //cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    //cv::morphologyEx(binary_image, cleaned_image, cv::MORPH_OPEN, se);
    bool result1 = cv::imwrite("D:/ceshi/cleaned_image.png", cleaned_image);
    if (result1) {
        std::cout << "cleaned_imageͼ���ѱ��浽�ļ���!" << std::endl;
    }
    else {
        std::cerr << "����ͼ��ʧ��!" << std::endl;
    }
    bool result2 = cv::imwrite("D:/ceshi/cleaned_image1.png", cleaned_image1);
    if (result2) {
        std::cout << "cleaned_image1ͼ���ѱ��浽�ļ���!" << std::endl;
    }
    else {
        std::cerr << "����ͼ��ʧ��!" << std::endl;
    }
    // ȥ��С����
    cv::Mat labeled_image;
    int num_labels = cv::connectedComponents(cleaned_image1, labeled_image);

    // ��ȡĿ�����������
    std::vector<Centroid> centroids;
    for (int label = 1; label < num_labels; ++label) {  // ��ǩ��1��ʼ��0�Ǳ���
        cv::Moments moments = cv::moments(labeled_image == label, true);
        if (moments.m00 > 0) {  // ������������Ϊ��
            Centroid c = { static_cast<float>(moments.m10 / moments.m00), static_cast<float>(moments.m01 / moments.m00) };
            centroids.push_back(c);
        }
    }

    // ����Ƿ��⵽Ŀ��
    if (centroids.empty()) {
        no_target = 0;  // û��Ŀ���⵽
        large_movement_count = 0;  // û��Ŀ��ʱ��λ�Ƽ�������
        return 0;
    }

    // ѡȡ��һ��������ΪĿ��
    centroid = centroids.front();
    no_target = 1;  // Ŀ���Ѽ�⵽
    
    if (last_centroid.x != 0 && last_centroid.y != 0) {  // �����һ֡������Ч������
        float distance = std::sqrt(std::pow(centroid.x - last_centroid.x, 2) + std::pow(centroid.y - last_centroid.y, 2));

        // �ж��Ƿ��нϴ�λ��
        float large_movement_threshold = 5.0f;  // 5������ֵ
        if (distance > large_movement_threshold) {
            large_movement_count = 1;  // �нϴ�λ��
        }
        else {
            large_movement_count = 0;  // λ�ƽ�С
        }
    }
    else {
        large_movement_count = 0;  // ����ǵ�һ֡����û��λ��
    }
    // ������һ֡������
    last_centroid = centroid;

    

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