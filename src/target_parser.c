/**
 * @file    target_parser.c
 * @brief   靶子定位协议解析器实现
 *
 * 状态机解析 $TARGET,<h>,<v>,<n>\r\n 和 $TARGET,NONE\r\n
 *
 * 解析策略：
 *   - 环形行缓冲收集字符，遇 \n 触发解析
 *   - 不依赖 sscanf（newlib-nano 的 sscanf 带 float 支持可能很大）
 *   - 手动解析：跳过 "$TARGET," → 判断 NONE 或解析三个数字
 */

#include "target_parser.h"
#include <string.h>

/* ===================================================================
 *  行缓冲（最大一帧约 30 字符）
 * =================================================================== */

#define LINE_BUF_SIZE  64

static char  s_line[LINE_BUF_SIZE];
static uint8_t s_len = 0;

/* ===================================================================
 *  手动解析浮点（跳过标准库，省 flash）
 * =================================================================== */

/** 解析有符号浮点字符串，返回解析后的字符数 */
static int parse_float(const char *s, float *val)
{
    if (!s || !val) return 0;

    float sign = 1.0f;
    float int_part = 0.0f;
    float frac_part = 0.0f;
    float frac_div = 1.0f;
    int   pos = 0;
    bool  in_frac = false;

    /* 符号 */
    if (s[pos] == '-') { sign = -1.0f; pos++; }
    else if (s[pos] == '+') { pos++; }

    /* 整数部分 */
    while (s[pos] >= '0' && s[pos] <= '9') {
        int_part = int_part * 10.0f + (float)(s[pos] - '0');
        pos++;
    }

    /* 小数部分 */
    if (s[pos] == '.') {
        pos++;
        in_frac = true;
        while (s[pos] >= '0' && s[pos] <= '9') {
            frac_part = frac_part * 10.0f + (float)(s[pos] - '0');
            frac_div *= 10.0f;
            pos++;
        }
    }

    if (pos == 0 || (pos == 1 && (s[0] == '-' || s[0] == '+')))
        return 0;  /* 无有效数字 */

    *val = sign * (int_part + (in_frac ? frac_part / frac_div : 0.0f));
    return pos;
}

/* ===================================================================
 *  帧解析
 * =================================================================== */

static bool parse_line(const char *line, TargetData_t *out)
{
    /* 检查帧头 */
    if (strncmp(line, "$TARGET,", 8) != 0) return false;

    const char *p = line + 8;

    /* 判断是否 NONE */
    if (strncmp(p, "NONE", 4) == 0) {
        out->valid   = false;
        out->h_deg   = 0.0f;
        out->v_deg   = 0.0f;
        out->circles = 0;
        return true;
    }

    /* 解析 H,V 和可选的 N（兼容 2 字段和 3 字段两种 K230 固件版本） */
    int n = 0;
    p += parse_float(p, &out->h_deg);
    if (*p != ',') return false;
    p++;

    p += parse_float(p, &out->v_deg);

    /* 如果有第三个字段（同心圆数），解析它 */
    if (*p == ',') {
        p++;
        n = 0;
        while (*p >= '0' && *p <= '9') {
            n = n * 10 + (*p - '0');
            p++;
        }
        if (n < 0 || n > 5) n = 0;
    }

    out->valid   = true;
    out->circles = n;
    return true;
}

/* ===================================================================
 *  公开 API
 * =================================================================== */

int TargetParser_Feed(uint8_t byte, TargetData_t *out)
{
    if (!out) return 0;

    /* 忽略 '\r'（不存入缓冲） */
    if (byte == '\r') return 0;

    /* 遇到 '\n' → 解析一行 */
    if (byte == '\n') {
        s_line[s_len] = '\0';
        s_len = 0;
        if (parse_line(s_line, out))
            return 1;   /* 有效 $TARGET 帧 */
        else
            return -1;  /* 完整行但不是 $TARGET */
    }

    /* 普通字符 → 存入行缓冲 */
    if (s_len < LINE_BUF_SIZE - 1) {
        s_line[s_len++] = (char)byte;
    } else {
        /* 缓冲溢出 → 丢弃这行 */
        s_len = 0;
    }

    return 0;
}

const char *TargetParser_GetLastLine(void)
{
    return s_line;
}

void TargetParser_Reset(void)
{
    s_len = 0;
}
