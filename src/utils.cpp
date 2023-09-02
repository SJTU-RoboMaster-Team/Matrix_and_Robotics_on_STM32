/**
 ******************************************************************************
 * @file    utils.cpp/h
 * @brief   General math utils. 常用数学工具函数
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "utils.h"

float math::limit(float val, const float& min, const float& max) {
  if (min > max)
    return val;
  else if (val < min)
    val = min;
  else if (val > max)
    val = max;
  return val;
}

float math::limitMin(float val, const float& min) {
  if (val < min)
    val = min;
  return val;
}

float math::limitMax(float val, const float& max) {
  if (val > max)
    val = max;
  return val;
}

float math::loopLimit(float val, const float& min, const float& max) {
  if (min >= max)
    return val;
  if (val > max) {
    while (val > max)
      val -= (max - min);
  } else if (val < min) {
    while (val < min)
      val += (max - min);
  }
  return val;
}

float math::sign(const float& val) {
  if (val > 0)
    return 1;
  else if (val < 0)
    return -1;
  return 0;
}

