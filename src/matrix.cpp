/**
 ******************************************************************************
 * @file    matrix.cpp/h
 * @brief   Matrix/vector calculation. 矩阵/向量运算
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "matrix.h"

// hat of vector
Matrixf<3, 3> vector3f::hat(Matrixf<3, 1> vec) {
  float hat[9] = {0,          -vec[2][0], vec[1][0], vec[2][0], 0,
                  -vec[0][0], -vec[1][0], vec[0][0], 0};
  return Matrixf<3, 3>(hat);
}

// cross product
Matrixf<3, 1> vector3f::cross(Matrixf<3, 1> vec1, Matrixf<3, 1> vec2) {
  return vector3f::hat(vec1) * vec2;
}
