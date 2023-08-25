/**
 ******************************************************************************
 * @file    math.cpp/h
 * @brief   General math utils. 常用数学工具函数
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "math.h"
#include "arm_math.h"

// Value limitation
// 限幅
float math::limit(float val, const float& min, const float& max) {
  if (min > max)
    return val;
  else if (val < min)
    val = min;
  else if (val > max)
    val = max;
  return val;
}

// Value limitation
// 限幅
float math::limitMin(float val, const float& min) {
  if (val < min)
    val = min;
  return val;
}

// Value limitation
// 限幅
float math::limitMax(float val, const float& max) {
  if (val > max)
    val = max;
  return val;
}

// Get sign of value
// 符号
float math::sign(const float& val) {
  if (val > 0)
    return 1;
  else if (val < 0)
    return -1;
  return 0;
}

// Voop limitation
// 循环限幅
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

// Dead band
// 死区
float math::deadBand(float val, const float& min, const float& max) {
  if (val < max && val > min) {
    val = 0;
  }
  return val;
}

// Angle unit conversion
// 角度单位换算
float math::rad2deg(const float& angle) {
  return angle * 57.2957795f;
}

float math::deg2rad(const float& angle) {
  return angle * 0.01745329f;
}

float math::ecd2deg(const float& ecd, const float& ecd_range) {
  return ecd * 360.f / ecd_range;
}

float math::ecd2rad(const float& ecd, const float& ecd_range) {
  return ecd * 2.f * PI / ecd_range;
}

// Rotational speed unit conversion(rpm-revolution/min, radps-rad/s, dps-°/s)
// 角速度单位换算(rpm-转/min, radps-rad/s, dps-°/s)
float math::rpm2radps(const float& w) {
  return w * 0.10471976f;
}

float math::radps2rpm(const float& w) {
  return w * 9.54929658f;
}

float math::radps2dps(const float& w) {
  return w * 57.2957795f;
}

float math::dps2radps(const float& w) {
  return w * 0.01745329f;
}

float math::rpm2dps(const float& w) {
  return w * 6.f;
}

float math::dps2rpm(const float& w) {
  return w * 0.16666667f;
}

// Angle normalization(rad->[-PI,PI], deg->[-180,180])
// 角度规范化(rad->[-PI,PI], deg->[-180,180])
float math::radNormalizePI(const float& angle) {
  return math::loopLimit(angle, -PI, PI);
}

float math::degNormalize180(const float& angle) {
  return math::loopLimit(angle, -180.f, 180.f);
}
