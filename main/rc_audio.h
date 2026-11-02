#pragma once

#include "lvgl.h"

/**
 * @brief 创建 RC 音频功能页（在地平仪画布 + 底部状态文字）。
 *        在给定的 parent（通常是应用界面 icon_in_obj）上创建 UI，
 *        并启动 PWM 输入、音频合成任务和姿态刷新定时器。
 */
void rc_audio_page(lv_obj_t *parent);

/**
 * @brief 退出 RC 音频功能页，停止音频任务、卸载 PWM 中断、
 *        关闭姿态传感器并释放画布缓冲。
 *        调用方负责随后删除 parent（icon_in_obj）。
 */
void rc_audio_exit(void);

/**
 * @brief 进入测试模式：开启功放并启动音频合成任务（不读取 PPM）。
 *        供「声音测试」菜单调用。
 */
void rc_audio_test_start(void);

/**
 * @brief 退出测试模式：停止所有声音并关闭功放。
 */
void rc_audio_test_stop(void);

/**
 * @brief 测试播放某一路声音（独占，再次调用同一路则停止）。
 * @param sound 0=引擎 1=警笛 2=救护 3=消防
 */
void rc_audio_test_play(int sound);
