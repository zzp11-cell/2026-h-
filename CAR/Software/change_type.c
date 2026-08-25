/**
 * @file  change_type.c
 * @brief 数据类型转换工具实现
 *
 * 来源: mspm0g3507_car/Software/Change_Type.c
 * 移植: 2026-07-23 → CAR/Software/
 * 适配: 去掉 #include "main.h"
 */
#include "change_type.h"
#include <stdlib.h>

/* 将字符串按 base=2 解析为 int (二进制字符串 -> 十进制整数) */
int String_to_Int(char *str)
{
    return (int)strtol(str, NULL, 2);
}
