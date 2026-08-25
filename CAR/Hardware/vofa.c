#include "vofa.h"
#include "encoder.h"    /* Velcity_Kp/Ki/Kff (速度环) */
#include "track.h"      /* TRACK_KP_T1/KD_T1, TRACK_KP_T2/KD_T2 (循迹) */
#include <math.h>       /* powf */
#include <string.h>

/* OLED 当前显示页 (定义在 main.c, VOFA 命令 V1=N! 在线切换) */
extern volatile uint8_t oled_page;

/* 速度档目标脉冲数 (定义在 main.c, VOFA 命令 T1=N! 调整) */
extern volatile int target_pulse;

/* ============================================================
 * UART 环形接收缓冲区 (无 zf_common FIFO, 自实现)
 * ============================================================ */
#define VOFA_BUF_SIZE  128
static uint8_t rx_buf[VOFA_BUF_SIZE];
static volatile uint8_t rx_head = 0;   /* 写入位置 (ISR 写, main 读) */
static volatile uint8_t rx_tail = 0;   /* 读取位置 (main 写, ISR 读溢出检查) */

/* 推入 1 字节 (ISR 调用, 满则丢弃最旧的) */
static void ringbuf_push(uint8_t byte)
{
    uint8_t next = (rx_head + 1) % VOFA_BUF_SIZE;
    if (next == rx_tail) {
        /* 缓冲区满: 丢弃最旧字节 (简单溢出处理, 文本协议容忍丢帧) */
        rx_tail = (rx_tail + 1) % VOFA_BUF_SIZE;
    }
    rx_buf[rx_head] = byte;
    rx_head = next;
}

/* 从 tail 开始, 复制到线性数组 (最大 len 字节), 返回实际复制数 */
static uint8_t ringbuf_copy(uint8_t *dst, uint8_t max_len)
{
    uint8_t count = 0;
    uint8_t pos = rx_tail;
    while (pos != rx_head && count < max_len) {
        dst[count++] = rx_buf[pos];
        pos = (pos + 1) % VOFA_BUF_SIZE;
    }
    return count;
}

/* 丢弃 n 字节 (消费已解析的数据) */
static void ringbuf_discard(uint8_t n)
{
    uint8_t avail;
    if (rx_head >= rx_tail) {
        avail = rx_head - rx_tail;
    } else {
        avail = VOFA_BUF_SIZE - rx_tail + rx_head;
    }
    if (n > avail) n = avail;
    rx_tail = (rx_tail + n) % VOFA_BUF_SIZE;
}

/* ============================================================
 * JustFloat 协议 (MCU → VOFA+)
 * ============================================================ */

/* IEEE 754 float → 4 字节小端 (和参考工程 Float_to_Byte 一致) */
typedef union {
    float    f;
    uint32_t u;
} FloatU32;

static void float_to_bytes(float val, uint8_t out[4])
{
    FloatU32 fu;
    fu.f = val;
    out[0] = (uint8_t)(fu.u);
    out[1] = (uint8_t)(fu.u >> 8);
    out[2] = (uint8_t)(fu.u >> 16);
    out[3] = (uint8_t)(fu.u >> 24);
}

/* 帧尾: IEEE 754 +Inf 小端 = 0x00 0x00 0x80 0x7F */
static const uint8_t justfloat_tail[4] = {0x00, 0x00, 0x80, 0x7F};

/* 通用 N 通道发送 */
void Vofa_JustFloat_Send(float *data, uint8_t count)
{
    uint8_t bytes[4];
    for (uint8_t i = 0; i < count; i++) {
        float_to_bytes(data[i], bytes);
        for (uint8_t j = 0; j < 4; j++) {
            DL_UART_Main_transmitDataBlocking(VOFA_UART_INST, bytes[j]);
        }
    }
    /* 帧尾 */
    for (uint8_t j = 0; j < 4; j++) {
        DL_UART_Main_transmitDataBlocking(VOFA_UART_INST, justfloat_tail[j]);
    }
}

void Vofa_JustFloat_One(float a)
{
    Vofa_JustFloat_Send(&a, 1);
}

void Vofa_JustFloat_Two(float a, float b)
{
    float arr[2] = {a, b};
    Vofa_JustFloat_Send(arr, 2);
}

void Vofa_JustFloat_Three(float a, float b, float c)
{
    float arr[3] = {a, b, c};
    Vofa_JustFloat_Send(arr, 3);
}

/* ============================================================
 * 文本命令解析 (VOFA+ → MCU) — 移植自参考工程
 * ============================================================ */

/* 从缓冲区解析 "=value!" 中的 float 值 (参考工程 Get_Data 算法) */
static float parse_float(uint8_t *buf, uint8_t start, uint8_t end)
{
    uint8_t minus_flag = 0;
    float   result     = 0.0f;
    uint8_t cursor     = start;

    /* 处理负号 */
    if (cursor <= end && buf[cursor] == '-') {
        minus_flag = 1;
        cursor++;
    }

    /* 逐位解析整数 + 小数 */
    uint8_t decimal_pos  = 0;
    uint8_t has_decimal  = 0;

    for (uint8_t i = cursor; i <= end; i++) {
        if (buf[i] == '.') {
            has_decimal = 1;
            decimal_pos = i - cursor;
            continue;
        }

        uint8_t digit = buf[i] - '0';
        if (digit > 9) return 0.0f;   /* 非法字符, 返回 0 */

        if (has_decimal) {
            result += (float)digit * powf(0.1f, (float)(i - cursor - decimal_pos));
        } else {
            result = result * 10.0f + (float)digit;
        }
    }

    return minus_flag ? -result : result;
}

/* 扫描缓冲区, 找 "XX=value!" 并解析 + 执行 PID 调参
 * 参考工程 Get_Data + USART_PID_Adjust
 * PID 映射:
 *   P1/-/D1 -> 循迹转向环 (TRACK_KP/KD, 复用原角度环位)
 *   P2/I2/D2 -> 速度环 (Velcity_Kp/Ki/Kff, encoder.c)
 *   P3/-/D3 -> 循迹转向环 (同 P1/D1, 两套都可用)
 */
void Vofa_PollCommand(void)
{
    uint8_t scan[VOFA_BUF_SIZE];
    uint8_t len = ringbuf_copy(scan, VOFA_BUF_SIZE);
    if (len < 3) return;   /* 最短命令 "X=0!" = 4 字节 */

    /* 查找 '=' 和结束标记 ('!' / '\r' / '\n') */
    int8_t eq_pos  = -1;
    int8_t end_pos = -1;
    for (uint8_t i = 0; i < len; i++) {
        if (scan[i] == '=')       eq_pos  = (int8_t)i;
        if (eq_pos >= 1 && (scan[i] == '!' || scan[i] == '\r' || scan[i] == '\n')) {
            end_pos = (int8_t)i;
            break;
        }
    }

    if (eq_pos < 2 || end_pos < 0) return;   /* 无完整命令 */
    if ((uint8_t)end_pos <= (uint8_t)(eq_pos + 1)) return;  /* 空值 "XX=!" */

    /* 解析数值 (eq_pos+1 ~ end_pos-1) */
    float val = parse_float(scan, (uint8_t)(eq_pos + 1), (uint8_t)(end_pos - 1));

    /* 解析命令字符 (eq_pos-2 和 eq_pos-1 两个字符) */
    uint8_t c0 = scan[eq_pos - 2];   /* 第一个命令字符 */
    uint8_t c1 = scan[eq_pos - 1];   /* 第二个命令字符 */

    /* PID 映射:
     *   P1/D1 → T2 循迹 PD (TRACK_KP_T1/KD_T1)
     *   P2/I2/D2 → 速度环 (Velcity_Kp/Ki/Kff)
     *   P3/D3 → T4/T5 循迹 PD (TRACK_KP_T2/KD_T2) */
    if (c0 == 'P') {
        if      (c1 == '1') TRACK_KP_T1 = val;   /* 任务1 P */
        else if (c1 == '2') Velcity_Kp  = val;   /* 速度环 P */
        else if (c1 == '3') TRACK_KP_T2 = val;   /* 任务2/3 P */
    } else if (c0 == 'I') {
        if      (c1 == '1') {}                   /* 占位 */
        else if (c1 == '2') Velcity_Ki  = val;   /* 速度环 I */
    } else if (c0 == 'D') {
        if      (c1 == '1') TRACK_KD_T1 = val;   /* 任务1 D */
        else if (c1 == '2') Velcity_Kff = val;   /* 速度环前馈 */
        else if (c1 == '3') TRACK_KD_T2 = val;   /* 任务2/3 D */
    } else if (c0 == 'V') {
        if (c1 == '1') oled_page = (uint8_t)val;   /* V1=N! 切 OLED 页面 (0/1/2) */
    } else if (c0 == 'T') {
        if (c1 == '1') target_pulse = (int)val;    /* T1=N! 调速度档 (脉冲数/10ms) */
    }

    /* 消费已处理的字节. \r\n 序列多跳 1 字节 */
    uint8_t n = (uint8_t)(end_pos + 1);
    if (scan[end_pos] == '\r' && end_pos + 1 < len && scan[end_pos + 1] == '\n')
        n += 1;
    ringbuf_discard(n);
}

/* ============================================================
 * VOFA RX 中断回调 (由 VOFA_UART_INST_IRQHandler 调用)
 * ============================================================ */
void Vofa_RX_ISR(uint8_t byte)
{
    ringbuf_push(byte);
}
