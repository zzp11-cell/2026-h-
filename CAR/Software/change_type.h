/**
 * @file  change_type.h
 * @brief 数据类型转换工具
 *
 * 来源: mspm0g3507_car/Software/Change_Type.h
 * 移植: 2026-07-23 → CAR/Software/
 * 适配: 无引脚依赖, 仅去掉 #include "main.h"
 */
#ifndef __CHANGE_TYPE_H__
#define __CHANGE_TYPE_H__

/* 将二进制字符串转为 int (如 "1010" -> 10) */
int String_to_Int(char *str);

#endif /* __CHANGE_TYPE_H__ */
