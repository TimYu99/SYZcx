// array_data.h

#ifndef ARRAY_DATA_H
#define ARRAY_DATA_H

#include <iostream>

// 常量定义
const int rows = 379;
const int cols = 401;

// 全局数组声明
extern int beijingtu21[rows][cols];

// 函数声明
void printValueAt(int row, int col);
void subtractArrays(const int arr1[rows][cols], const int arr2[rows][cols], int result[rows][cols]);
#endif // ARRAY_DATA_H