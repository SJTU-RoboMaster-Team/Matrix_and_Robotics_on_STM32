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

#ifndef MATH_H
#define MATH_H

#include <stdint.h>

namespace math {

// Value limitation 限幅
float limit(float val, const float& min, const float& max);
float limitMin(float val, const float& min);
float limitMax(float val, const float& max);

// Voop limitation 循环限幅
float loopLimit(float val, const float& min, const float& max);

// Get sign of value 符号
float sign(const float& val);

// Dead band 死区
float deadBand(float val, const float& min, const float& max);

// Angle unit conversion 角度单位换算
float rad2deg(const float& angle);
float deg2rad(const float& angle);
float ecd2deg(const float& ecd, const float& ecd_range);
float ecd2rad(const float& ecd, const float& ecd_range);

// Rotational speed unit conversion(rpm-revolution/min, radps-rad/s, dps-°/s)
// 角速度单位换算(rpm-转/min, radps-rad/s, dps-°/s)
float rpm2radps(const float& w);
float radps2rpm(const float& w);
float rpm2dps(const float& w);
float dps2rpm(const float& w);
float radps2dps(const float& w);
float dps2radps(const float& w);
float dps2rpm(const float& w);

// Angle normalization(rad->[-PI,PI], deg->[-180,180])
// 角度规范化(rad->[-PI,PI], deg->[-180,180])
float radNormalizePI(const float& angle);
float degNormalize180(const float& angle);

}  // namespace math

#endif  // MATH_H
