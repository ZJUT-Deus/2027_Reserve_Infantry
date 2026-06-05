/**
 * @file    struct_typedef.h
 * @brief   项目基础类型定义
 * @author  kk
 * @date    2026-05-22
 */

#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

/** @brief 有符号 8 位整数 */
typedef signed char int8_t;
/** @brief 有符号 16 位整数 */
typedef signed short int int16_t;
/** @brief 有符号 32 位整数 */
typedef signed int int32_t;
/** @brief 有符号 64 位整数 */
typedef signed long long int64_t;

/** @brief 无符号 8 位整数 */
typedef unsigned char uint8_t;
/** @brief 无符号 16 位整数 */
typedef unsigned short int uint16_t;
/** @brief 无符号 32 位整数 */
typedef unsigned int uint32_t;
/** @brief 无符号 64 位整数 */
typedef unsigned long long uint64_t;
/** @brief 布尔类型 */
typedef unsigned char bool_t;
/** @brief 单精度浮点类型 */
typedef float fp32;
/** @brief 双精度浮点类型 */
typedef double fp64;

/** @brief 布尔假 */
#define FALSE 0
/** @brief 布尔真 */
#define TRUE 1

/** @brief 圆周率 */
#define PI 3.1415926f

#endif
